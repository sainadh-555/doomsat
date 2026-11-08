"""Master multi-physics simulation orchestrator for DOOMSat 1U CubeSat.

Coordinates coupled physical domains:
Orbit -> Sun/Eclipse -> Earth B-Field -> ADCS Dynamics & Sensors -> OBC Flight Software ->
EPS -> Thermal (8-Node) -> Comms/RF Channel -> Ground Station -> Dashboard.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any, Callable
import math
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import (
    EARTH_MU,
    EARTH_RADIUS_EQUATORIAL,
    EARTH_OMEGA_ROTATION,
    SOLAR_CONSTANT,
    RF_BAND_ITU_AMATEUR_UHF_HZ,
    RF_BAND_WPC_INDIA_DELICENSED_HZ,
)
from doomsat.core.coordinates import (
    ecef_to_geodetic,
    eci_to_ecef,
    quaternion_to_dcm,
    quaternion_rotate_vector,
)
from doomsat.core.schemas import TelemetryBeacon
from doomsat.mechanical.assembly import build_reference_doomsat_1u
from doomsat.orbit.propagator import NumericalOrbitPropagator, keplerian_to_cartesian
from doomsat.orbit.sun import compute_sun_vector_eci, compute_solar_flux
from doomsat.orbit.magnetic import compute_geomagnetic_field_eci
from doomsat.orbit.ground_pass import GroundStation, LookAngles
from doomsat.adcs.dynamics import (
    RigidBodyAttitudeDynamics,
    compute_gravity_gradient_torque,
    compute_aerodynamic_torque,
    compute_magnetic_disturbance_torque,
    compute_srp_torque,
)
from doomsat.adcs.sensors import (
    MPU6050IMU,
    LIS3MDLMagnetometer,
    SunSensorArray,
)
from doomsat.adcs.control import BDotController, TRIADEstimator
from doomsat.obc.flight_software import OBCFlightSoftware, FlightMode
from doomsat.eps.solar import SolarArray
from doomsat.eps.battery import LiIonBattery
from doomsat.eps.power_bus import PowerDistributionBus
from doomsat.thermal.nodal_model import ThermalNetwork8Node
from doomsat.comms.sx1276 import LoRaConfig
from doomsat.comms.drivers import SimulatedSX1276Driver, PhysicalESP32SerialDriver, RadioDriver
from doomsat.ground_station.receiver import GroundStationReceiver
from doomsat.ground_station.telemetry_store import TelemetryStore
from doomsat.ground_station.image_reconstruction import GroundStationImageManager


@dataclass
class SimulationConfig:
    """Master simulation configuration parameters."""
    dt_s: float = 1.0                              # Simulation time step [s]
    duration_s: float = 5700.0                     # Total simulation run duration (~95 min orbit) [s]
    altitude_m: float = 500_000.0                  # Initial orbit altitude above equatorial radius [m]
    inclination_deg: float = 51.6                  # Orbit inclination (ISS reference) [deg]
    ground_station_lat_deg: float = 17.3850        # Hyderabad Ground Station Latitude [deg]
    ground_station_lon_deg: float = 78.4867        # Hyderabad Ground Station Longitude [deg]
    ground_station_alt_m: float = 542.0            # Hyderabad elevation [m]
    initial_tumble_deg_s: Tuple[float, float, float] = (15.0, -10.0, 12.0)  # Initial tumble rate [deg/s]
    initial_soc_pct: float = 95.0                  # Initial battery State of Charge [%]
    radio_mode: str = "SIMULATION"                 # "SIMULATION" or "HIL"
    hil_serial_port: Optional[str] = None          # Physical serial port for HIL
    hil_baud_rate: int = 115200                    # HIL serial baud rate


@dataclass
class SimulationSnapshot:
    """Telemetry and state snapshot generated at each master simulation step."""
    time_s: float
    r_eci_km: Tuple[float, float, float]
    v_eci_km_s: Tuple[float, float, float]
    altitude_km: float
    latitude_deg: float
    longitude_deg: float
    q_body_eci: Tuple[float, float, float, float]
    omega_body_deg_s: Tuple[float, float, float]
    omega_norm_deg_s: float
    in_eclipse: bool
    solar_flux_w_m2: float
    b_field_body_ut: Tuple[float, float, float]
    flight_mode: str
    battery_soc_pct: float
    battery_voltage_v: float
    solar_power_w: float
    system_load_w: float
    temp_battery_c: float
    temp_electronics_c: float
    gs_visible: bool
    gs_elevation_deg: float
    gs_slant_range_km: float
    gs_doppler_khz: float
    link_margin_db: Optional[float] = None


class MissionOrchestrator:
    """Master Multi-Physics Simulator integrating all DOOMSat subsystems."""

    def __init__(self, config: Optional[SimulationConfig] = None) -> None:
        self.config = config if config is not None else SimulationConfig()
        self.current_time_s: float = 0.0

        # 1. Mechanical Subsystem (Mass & Inertia)
        self.assembly = build_reference_doomsat_1u()
        self.inertia_tensor = self.assembly.compute_inertia_tensor()
        self.total_mass_kg = self.assembly.total_mass_kg

        # 2. Orbital Mechanics & Propagator
        r0, v0 = keplerian_to_cartesian(
            a_m=EARTH_RADIUS_EQUATORIAL + self.config.altitude_m,
            e=0.001,
            inc_rad=math.radians(self.config.inclination_deg),
            raan_rad=0.0,
            argp_rad=0.0,
            nu_rad=0.0,
        )
        self.propagator = NumericalOrbitPropagator(initial_r_eci=r0, initial_v_eci=v0)
        self.r_eci: NDArray[np.float64] = r0.copy()
        self.v_eci: NDArray[np.float64] = v0.copy()

        # 3. Ground Station (Hyderabad)
        self.ground_station = GroundStation(
            latitude_deg=self.config.ground_station_lat_deg,
            longitude_deg=self.config.ground_station_lon_deg,
            altitude_m=self.config.ground_station_alt_m,
            min_elevation_deg=10.0,
        )

        # 4. ADCS Attitude Dynamics & Sensors
        initial_q = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
        initial_omega = np.radians(np.array(self.config.initial_tumble_deg_s, dtype=np.float64))
        self.dynamics = RigidBodyAttitudeDynamics(
            inertia_tensor=self.inertia_tensor,
            initial_q=initial_q,
            initial_omega_radps=initial_omega,
        )
        self.imu = MPU6050IMU()
        self.mag = LIS3MDLMagnetometer()
        self.sun_sensors = SunSensorArray()
        self.bdot_controller = BDotController(gain_k=2.0e5, max_dipole_am2=0.15)
        self.triad = TRIADEstimator()

        # 5. Electrical Power Subsystem (EPS)
        self.solar_array = SolarArray(panel_area_m2=0.007, cell_efficiency=0.285, mppt_efficiency=0.92)
        self.battery = LiIonBattery(initial_soc_pct=self.config.initial_soc_pct)
        self.power_bus = PowerDistributionBus(battery=self.battery)

        # 6. Thermal Subsystem (8-Node Lumped Network)
        self.thermal = ThermalNetwork8Node()

        # 7. OBC Flight Software
        self.obc = OBCFlightSoftware(initial_mode=FlightMode.BOOT)

        # 8. Radio Driver & Communications
        if self.config.radio_mode.upper() == "HIL" and self.config.hil_serial_port:
            self.radio_driver: RadioDriver = PhysicalESP32SerialDriver(
                port=self.config.hil_serial_port,
                baud_rate=self.config.hil_baud_rate,
            )
        else:
            lora_cfg = LoRaConfig(frequency_hz=RF_BAND_ITU_AMATEUR_UHF_HZ)
            self.radio_driver = SimulatedSX1276Driver(radio_config=lora_cfg)

        # 9. Ground Station Segment
        self.telemetry_store = TelemetryStore()
        self.image_manager = GroundStationImageManager()
        self.receiver = GroundStationReceiver(
            radio_driver=self.radio_driver,
            telemetry_store=self.telemetry_store,
            image_manager=self.image_manager,
        )

        # Internal state metrics
        self.beacon_seq_counter: int = 1
        self.last_look_angles: Optional[LookAngles] = None

    def step(self) -> SimulationSnapshot:
        """Advance the master multi-physics simulation clock by dt_s."""
        dt = self.config.dt_s
        t = self.current_time_s

        # ----------------------------------------------------------------------
        # 1. Orbit Propagation (Cowell RK4 with Earth J2)
        # ----------------------------------------------------------------------
        self.r_eci, self.v_eci = self.propagator.step(dt)
        r_mag = float(np.linalg.norm(self.r_eci))
        altitude_m = r_mag - EARTH_RADIUS_EQUATORIAL

        # Sub-satellite coordinates via ECEF and Bowring
        r_ecef = eci_to_ecef(self.r_eci, t)
        lat_rad, lon_rad, _ = ecef_to_geodetic(r_ecef)
        lat_deg = math.degrees(lat_rad)
        lon_deg = math.degrees(lon_rad)

        # ----------------------------------------------------------------------
        # 2. Solar Ephemeris, Eclipse, and Earth Magnetic Field
        # ----------------------------------------------------------------------
        sun_unit_eci = compute_sun_vector_eci(t)
        solar_flux, nu, in_shadow = compute_solar_flux(self.r_eci, sun_unit_eci)
        b_eci = compute_geomagnetic_field_eci(self.r_eci, t)

        # ----------------------------------------------------------------------
        # 3. Ground Station Pass Geometry (Hyderabad)
        # ----------------------------------------------------------------------
        gmst_rad = (EARTH_OMEGA_ROTATION * t) % (2.0 * math.pi)
        look = self.ground_station.compute_look_angles(self.r_eci, self.v_eci, gmst_rad)
        self.last_look_angles = look

        # ----------------------------------------------------------------------
        # 4. Body Frame Transformations & ADCS Sub-Stepped Dynamics
        # ----------------------------------------------------------------------
        # Fast attitude rotational dynamics require numerical sub-stepping (<= 0.25s)
        # to prevent derivative aliasing in B-dot estimation and Runge-Kutta truncation errors.
        if dt > 0.25:
            n_substeps = max(1, int(round(dt / 0.25)))
        else:
            n_substeps = 1
        sub_dt = dt / n_substeps

        for sub_idx in range(n_substeps):
            t_sub = t + sub_idx * sub_dt
            dcm_sub = quaternion_to_dcm(self.dynamics.q)
            b_body_sub = dcm_sub @ b_eci
            sun_body_sub = dcm_sub @ sun_unit_eci

            # ADCS Control: B-dot rate damping active in BOOT, SAFE_HOLD, DETUMBLE, and NOMINAL
            tau_ctrl = np.zeros(3, dtype=np.float64)
            if self.obc.fsm.current_mode in (
                FlightMode.BOOT,
                FlightMode.SAFE_HOLD,
                FlightMode.DETUMBLE,
                FlightMode.NOMINAL,
            ):
                _, tau_ctrl = self.bdot_controller.compute_control_torque(b_body_sub, t_sub)

            # Environmental disturbance torques in body frame
            r_body_sub = dcm_sub @ self.r_eci
            v_body_sub = dcm_sub @ self.v_eci
            tau_gg = compute_gravity_gradient_torque(r_body_sub, self.inertia_tensor)
            tau_aero = compute_aerodynamic_torque(v_body_sub)
            tau_mag = compute_magnetic_disturbance_torque(np.array([0.005, -0.005, 0.002]), b_body_sub)
            tau_srp = compute_srp_torque(sun_body_sub, nu)
            total_torque = tau_ctrl + tau_gg + tau_aero + tau_mag + tau_srp

            # Step 6-DoF attitude dynamics
            self.dynamics.step_rk4(sub_dt, total_torque)

        # Update final body frame vectors and metrics post-sub-stepping
        q = self.dynamics.q
        dcm_eci_to_body = quaternion_to_dcm(q)
        sun_body_unit = dcm_eci_to_body @ sun_unit_eci
        b_body = dcm_eci_to_body @ b_eci
        omega_deg_s = np.degrees(self.dynamics.omega)
        omega_norm_deg_s = float(np.linalg.norm(omega_deg_s))

        # ----------------------------------------------------------------------
        # 5. ADCS Sensors Sampling
        # ----------------------------------------------------------------------
        gyro_meas, accel_meas = self.imu.measure(self.dynamics.omega, np.zeros(3), dt)
        mag_meas = self.mag.measure(b_body)
        sun_voltages = self.sun_sensors.measure_voltages(sun_body_unit, nu)
        sun_est_vec, is_sun_detected = self.sun_sensors.estimate_sun_vector(sun_voltages)

        # ----------------------------------------------------------------------
        # 6. Electrical Power Subsystem (EPS)
        # ----------------------------------------------------------------------
        # Extract face temperatures from thermal model for efficiency derating
        face_temps = {
            "+X": self.thermal.temperatures[0],
            "-X": self.thermal.temperatures[1],
            "+Y": self.thermal.temperatures[2],
            "-Y": self.thermal.temperatures[3],
            "+Z": self.thermal.temperatures[4],
        }
        solar_gen_w, _ = self.solar_array.compute_generation(
            sun_body_unit=sun_body_unit,
            face_temperatures_k=face_temps,
            solar_flux_w_m2=SOLAR_CONSTANT,
            eclipse_factor=nu,
        )

        # Step power bus and battery
        power_state, joule_heat_w = self.power_bus.step(
            solar_power_w=solar_gen_w,
            flight_mode=self.obc.fsm.current_mode.value,
            dt_s=dt,
            battery_temp_k=self.thermal.temperatures[7],
        )

        # ----------------------------------------------------------------------
        # 7. Thermal Subsystem (8-Node Network)
        # ----------------------------------------------------------------------
        nadir_body = dcm_eci_to_body @ (-self.r_eci / r_mag)
        thermal_state = self.thermal.step(
            sun_body_unit=sun_body_unit,
            nadir_body_unit=nadir_body,
            r_sat_m=r_mag,
            eclipse_factor=nu,
            p_load_w=power_state.system_load_w,
            battery_joule_heat_w=joule_heat_w,
            dt_s=dt,
        )

        # ----------------------------------------------------------------------
        # 8. OBC Flight Software Execution
        # ----------------------------------------------------------------------
        self.obc.watchdog.kick(t)
        new_mode = self.obc.step(
            current_time_s=t,
            battery_soc_pct=power_state.battery_soc_pct,
            omega_norm_radps=float(np.linalg.norm(self.dynamics.omega)),
            gs_visible=look.is_visible,
        )

        # ----------------------------------------------------------------------
        # 9. RF Channel & Ground Station Pass Telemetry
        # ----------------------------------------------------------------------
        link_margin_db = None
        if look.is_visible and isinstance(self.radio_driver, SimulatedSX1276Driver):
            # Update physical channel propagation
            lb = self.radio_driver.set_channel_conditions(
                slant_range_m=look.slant_range_m,
                elevation_rad=math.radians(look.elevation_deg),
                radial_velocity_mps=look.radial_velocity_mps,
            )
            link_margin_db = lb.link_margin_db

            # Periodically broadcast telemetry beacon (every 5 seconds)
            if int(t) % 5 == 0:
                beacon = TelemetryBeacon(
                    sequence_id=self.beacon_seq_counter,
                    timestamp_s=round(t, 2),
                    flight_mode=new_mode.value,
                    battery_soc=power_state.battery_soc_pct,
                    battery_voltage=power_state.battery_voltage_v,
                    solar_power=power_state.solar_power_w,
                    obc_temp_k=thermal_state.temp_electronics_k,
                    battery_temp_k=thermal_state.temp_battery_k,
                    angular_rate_norm_radps=float(np.linalg.norm(self.dynamics.omega)),
                    in_eclipse=in_shadow,
                    rssi_dbm=lb.received_power_dbm,
                    snr_db=lb.snr_db,
                )
                self.beacon_seq_counter += 1
                self.obc.record_telemetry(beacon)

                # Transmit over radio link
                from doomsat.comms.cstp import CSTPPacket, MessageType, NodeID, slip_encode
                raw_pkt = CSTPPacket(
                    src_id=NodeID.OBC,
                    dst_id=NodeID.GROUND_STATION,
                    msg_type=MessageType.TELEMETRY_BEACON,
                    sequence_num=beacon.sequence_id,
                    payload=beacon.pack_binary(),
                )
                slip_frame = slip_encode(raw_pkt.pack())
                self.radio_driver.send_packet(slip_frame)

                # Ground station ingests transmitted packet
                self.receiver.process_raw_frame(
                    slip_frame,
                    rssi_dbm=lb.received_power_dbm,
                    snr_db=lb.snr_db,
                )

        # Doppler shift calculation [kHz]
        doppler_khz = (-look.radial_velocity_mps / 299792458.0) * (RF_BAND_ITU_AMATEUR_UHF_HZ / 1000.0)

        # Advance master clock
        self.current_time_s += dt

        # Construct and return snapshot
        return SimulationSnapshot(
            time_s=round(t, 2),
            r_eci_km=(round(self.r_eci[0] / 1000.0, 3), round(self.r_eci[1] / 1000.0, 3), round(self.r_eci[2] / 1000.0, 3)),
            v_eci_km_s=(round(self.v_eci[0] / 1000.0, 3), round(self.v_eci[1] / 1000.0, 3), round(self.v_eci[2] / 1000.0, 3)),
            altitude_km=round(altitude_m / 1000.0, 2),
            latitude_deg=round(lat_deg, 3),
            longitude_deg=round(lon_deg, 3),
            q_body_eci=(round(q[0], 4), round(q[1], 4), round(q[2], 4), round(q[3], 4)),
            omega_body_deg_s=(round(omega_deg_s[0], 3), round(omega_deg_s[1], 3), round(omega_deg_s[2], 3)),
            omega_norm_deg_s=round(omega_norm_deg_s, 3),
            in_eclipse=in_shadow,
            solar_flux_w_m2=round(solar_flux, 1),
            b_field_body_ut=(round(b_body[0] * 1e6, 2), round(b_body[1] * 1e6, 2), round(b_body[2] * 1e6, 2)),
            flight_mode=new_mode.value,
            battery_soc_pct=power_state.battery_soc_pct,
            battery_voltage_v=power_state.battery_voltage_v,
            solar_power_w=power_state.solar_power_w,
            system_load_w=power_state.system_load_w,
            temp_battery_c=round(thermal_state.temp_battery_k - 273.15, 2),
            temp_electronics_c=round(thermal_state.temp_electronics_k - 273.15, 2),
            gs_visible=look.is_visible,
            gs_elevation_deg=round(look.elevation_deg, 2),
            gs_slant_range_km=round(look.slant_range_m / 1000.0, 2),
            gs_doppler_khz=round(doppler_khz, 3),
            link_margin_db=round(link_margin_db, 2) if link_margin_db is not None else None,
        )

    def run(
        self,
        duration_s: Optional[float] = None,
        callback: Optional[Callable[[SimulationSnapshot], None]] = None,
    ) -> List[SimulationSnapshot]:
        """Execute simulation run over the specified duration."""
        target_duration = duration_s if duration_s is not None else self.config.duration_s
        total_steps = int(target_duration / self.config.dt_s)
        snapshots: List[SimulationSnapshot] = []

        for _ in range(total_steps):
            snap = self.step()
            snapshots.append(snap)
            if callback:
                callback(snap)

        return snapshots
