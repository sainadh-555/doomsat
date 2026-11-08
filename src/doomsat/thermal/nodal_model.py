"""8-node lumped parameter thermal network for DOOMSat 1U CubeSat.

Models thermal node temperatures:
- 6 external face panels (+X, -X, +Y, -Y, +Z, -Z)
- Internal electronics stack (RPi Zero 2 W, ESP32, Comms, ADCS)
- Internal battery cell (Samsung 18650 Li-ion)

Incorporates:
- External environmental heat fluxes: Direct solar flux, Earth albedo, Earth planetary IR
- Stefan-Boltzmann radiative cooling to deep space (3 K)
- Conductive heat transfer between panels, standoffs, and battery mounting
- Internal dissipation from electronics and battery Joule heating
- 4th-order Runge-Kutta (RK4) numerical integration.
"""

from typing import Dict, List, Optional, Tuple
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import (
    STEFAN_BOLTZMANN,
    SOLAR_CONSTANT,
    EARTH_RADIUS_EQUATORIAL,
)
from doomsat.core.schemas import ThermalState


class ThermalNetwork8Node:
    """8-node lumped parameter thermal model of the 1U CubeSat."""

    NODE_NAMES = [
        "face_pos_x",  # 0
        "face_neg_x",  # 1
        "face_pos_y",  # 2
        "face_neg_y",  # 3
        "face_pos_z",  # 4
        "face_neg_z",  # 5
        "electronics", # 6
        "battery",     # 7
    ]

    def __init__(
        self,
        initial_temperatures_k: Optional[Dict[str, float]] = None,
        solar_absorptivity_alpha: float = 0.90,
        ir_emissivity_epsilon: float = 0.82,
        face_area_m2: float = 0.010,           # 100 mm x 100 mm = 0.010 m^2
        earth_ir_flux_w_m2: float = 237.0,     # Global mean Earth IR
        earth_albedo_coeff: float = 0.30,      # Mean Earth optical albedo
        space_temp_k: float = 3.0,             # Deep space sink temperature
    ) -> None:
        self.alpha = solar_absorptivity_alpha
        self.epsilon = ir_emissivity_epsilon
        self.area_face = face_area_m2
        self.earth_ir_flux = earth_ir_flux_w_m2
        self.albedo_coeff = earth_albedo_coeff
        self.t_space = space_temp_k

        # Outward normal vectors for the 6 external faces in Spacecraft Body frame
        self.face_normals: NDArray[np.float64] = np.array([
            [1.0, 0.0, 0.0],   # +X
            [-1.0, 0.0, 0.0],  # -X
            [0.0, 1.0, 0.0],   # +Y
            [0.0, -1.0, 0.0],  # -Y
            [0.0, 0.0, 1.0],   # +Z
            [0.0, 0.0, -1.0],  # -Z
        ], dtype=np.float64)

        # Thermal capacitances C_i = m_i * c_{p,i} [J/K]
        # Aluminum 6061-T6 (c_p = 896 J/(kg*K), 45g per face panel)
        c_panel = 0.045 * 896.0  # 40.32 J/K
        # Electronics stack (c_p = 850 J/(kg*K), 200g avionics stack)
        c_elec = 0.200 * 850.0   # 170.0 J/K
        # Battery cell 18650 (c_p = 1000 J/(kg*K), 45g)
        c_batt = 0.045 * 1000.0  # 45.0 J/K

        self.capacitances: NDArray[np.float64] = np.array([
            c_panel, c_panel, c_panel, c_panel, c_panel, c_panel,
            c_elec, c_batt
        ], dtype=np.float64)

        # Conductance matrix K_ij [W/K] (symmetric 8x8)
        self.k_matrix = np.zeros((8, 8), dtype=np.float64)
        self._build_conductance_matrix()

        # State vector: temperatures of the 8 nodes [K]
        self.temperatures: NDArray[np.float64] = np.full(8, 293.15, dtype=np.float64)
        if initial_temperatures_k:
            for idx, name in enumerate(self.NODE_NAMES):
                if name in initial_temperatures_k:
                    self.temperatures[idx] = initial_temperatures_k[name]

    def _build_conductance_matrix(self) -> None:
        """Construct conductive coupling matrix between nodes."""
        # Panel-to-panel conductive coupling along aluminum corner rails
        k_pp = 0.15  # W/K
        # Adjacent pairs among the 6 faces (each face touches 4 neighbors)
        # Face indices: 0=+X, 1=-X, 2=+Y, 3=-Y, 4=+Z, 5=-Z
        adjacent_pairs = [
            (0, 2), (0, 3), (0, 4), (0, 5),
            (1, 2), (1, 3), (1, 4), (1, 5),
            (2, 4), (2, 5), (3, 4), (3, 5),
        ]
        for i, j in adjacent_pairs:
            self.k_matrix[i, j] = k_pp
            self.k_matrix[j, i] = k_pp

        # Electronics stack (Node 6) connected to panels via aluminum standoffs
        k_ep = 0.05  # W/K to each panel (total 0.30 W/K)
        for p in range(6):
            self.k_matrix[6, p] = k_ep
            self.k_matrix[p, 6] = k_ep

        # Battery (Node 7) clamped to chassis frame (+Y and -Z faces)
        k_bp = 0.02  # W/K
        for p in [2, 5]:
            self.k_matrix[7, p] = k_bp
            self.k_matrix[p, 7] = k_bp

        # Battery to electronics wiring / adjacent standoff
        self.k_matrix[7, 6] = 0.10
        self.k_matrix[6, 7] = 0.10

    def compute_earth_view_factor(self, r_sat_m: float, cos_nadir: float) -> float:
        """Compute Earth view factor for a flat planar surface on the spacecraft.

        Standard spacecraft thermal formulation (Gilmore Spacecraft Thermal Control Handbook):
        At orbital altitude, Earth angular radius is rho_E = arcsin(R_E / r_sat).
        - Nadir-facing surface (cos_nadir = +1): F = sin^2(rho_E)
        - Side surface perpendicular to nadir (cos_nadir = 0): F ~ 0.18
        - Zenith-facing surface (cos_nadir = -1): F = 0.0

        Args:
            r_sat_m: Orbital radial distance from Earth center [m].
            cos_nadir: Cosine of angle between face outward normal and satellite-to-Earth vector.

        Returns:
            Earth radiation view factor F_{i -> Earth} in [0.0, 1.0].
        """
        r_earth = EARTH_RADIUS_EQUATORIAL
        if r_sat_m <= r_earth:
            sin_rho = 1.0
        else:
            sin_rho = min(1.0, r_earth / r_sat_m)

        f_nadir = sin_rho ** 2
        f_side = 0.18 * sin_rho

        if cos_nadir >= 0.0:
            f_earth = cos_nadir * f_nadir + (1.0 - cos_nadir) * f_side
        elif cos_nadir >= -0.35:
            f_earth = f_side * (1.0 + cos_nadir / 0.35)
        else:
            f_earth = 0.0

        return float(max(0.0, min(1.0, f_earth)))

    def compute_heat_inputs(
        self,
        sun_body_unit: Optional[NDArray[np.float64]],
        nadir_body_unit: Optional[NDArray[np.float64]],
        r_sat_m: float,
        eclipse_factor: float,
        p_load_w: float,
        battery_joule_heat_w: float,
        solar_flux_w_m2: float = SOLAR_CONSTANT,
    ) -> NDArray[np.float64]:
        """Compute external and internal environmental heat inputs Q_in [W] for all 8 nodes.

        Args:
            sun_body_unit: Unit vector to Sun in Body frame.
            nadir_body_unit: Unit vector to Earth center (Nadir) in Body frame.
            r_sat_m: Satellite orbit radius [m].
            eclipse_factor: Eclipse factor nu in [0, 1].
            p_load_w: Internal electronics heat dissipation [W].
            battery_joule_heat_w: Battery cell internal I^2 * R heating [W].
            solar_flux_w_m2: Direct solar flux [W/m^2].

        Returns:
            8-element array of heat input rates [W].
        """
        q_in = np.zeros(8, dtype=np.float64)

        # 1. External Faces (Nodes 0 to 5)
        for i in range(6):
            normal = self.face_normals[i]

            # Direct solar radiation
            if sun_body_unit is not None and eclipse_factor > 0.0:
                cos_sun = float(np.dot(normal, sun_body_unit))
                if cos_sun > 0.0:
                    q_solar = self.alpha * self.area_face * solar_flux_w_m2 * cos_sun * eclipse_factor
                else:
                    q_solar = 0.0
            else:
                q_solar = 0.0

            # Earth planetary IR and Albedo
            if nadir_body_unit is not None:
                cos_nadir = float(np.dot(normal, nadir_body_unit))
                f_earth = self.compute_earth_view_factor(r_sat_m, cos_nadir)

                # Planetary IR (emitted continuously by Earth)
                q_earth_ir = self.epsilon * self.area_face * self.earth_ir_flux * f_earth

                # Albedo (Earth reflected sunlight)
                if sun_body_unit is not None and eclipse_factor > 0.0:
                    # Sun illumination of Earth disc visible to satellite
                    cos_sun_earth = max(0.0, float(np.dot(nadir_body_unit, sun_body_unit)))
                    q_albedo = (
                        self.albedo_coeff
                        * solar_flux_w_m2
                        * self.alpha
                        * self.area_face
                        * f_earth
                        * cos_sun_earth
                        * eclipse_factor
                    )
                else:
                    q_albedo = 0.0
            else:
                q_earth_ir = 0.0
                q_albedo = 0.0

            q_in[i] = q_solar + q_earth_ir + q_albedo

        # 2. Internal Avionics Stack (Node 6)
        q_in[6] = max(0.0, p_load_w)

        # 3. Battery Cell (Node 7)
        q_in[7] = max(0.0, battery_joule_heat_w)

        return q_in

    def compute_derivatives(
        self,
        temperatures: NDArray[np.float64],
        q_in: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Compute instantaneous temperature rate of change dT/dt for all 8 nodes.

        Equation:
            C_i * dT_i/dt = Q_in,i - Q_rad,out,i - sum_{j!=i} K_ij * (T_i - T_j)

        Args:
            temperatures: 8-element array of node temperatures [K].
            q_in: 8-element array of heat inputs [W].

        Returns:
            8-element array of dT/dt [K/s].
        """
        dT_dt = np.zeros(8, dtype=np.float64)

        for i in range(8):
            # 1. Radiative cooling to deep space (Nodes 0 to 5)
            if i < 6:
                # Stefan-Boltzmann radiation from outer face to deep space
                t_i = temperatures[i]
                q_rad_space = (
                    STEFAN_BOLTZMANN
                    * self.epsilon
                    * self.area_face
                    * (t_i ** 4 - self.t_space ** 4)
                )
            else:
                q_rad_space = 0.0

            # 2. Conductive heat exchange with other nodes
            q_cond_out = 0.0
            for j in range(8):
                if i != j:
                    k_ij = self.k_matrix[i, j]
                    if k_ij > 0.0:
                        q_cond_out += k_ij * (temperatures[i] - temperatures[j])

            # 3. Energy balance
            net_heat_rate = q_in[i] - q_rad_space - q_cond_out
            dT_dt[i] = net_heat_rate / self.capacitances[i]

        return dT_dt

    def step(
        self,
        sun_body_unit: Optional[NDArray[np.float64]],
        nadir_body_unit: Optional[NDArray[np.float64]],
        r_sat_m: float,
        eclipse_factor: float,
        p_load_w: float,
        battery_joule_heat_w: float,
        dt_s: float,
        solar_flux_w_m2: float = SOLAR_CONSTANT,
    ) -> ThermalState:
        """Integrate thermal network over time step dt_s using 4th-order Runge-Kutta.

        Args:
            sun_body_unit: Unit vector to Sun in Body frame.
            nadir_body_unit: Unit vector to Nadir in Body frame.
            r_sat_m: Orbital radial distance [m].
            eclipse_factor: Eclipse factor nu in [0, 1].
            p_load_w: Total electronics load dissipation [W].
            battery_joule_heat_w: Battery internal Joule heat [W].
            dt_s: Integration time step [s].
            solar_flux_w_m2: Direct solar flux [W/m^2].

        Returns:
            Validated ThermalState schema instance.
        """
        # Calculate heat inputs (assumed quasi-constant over dt_s)
        q_in = self.compute_heat_inputs(
            sun_body_unit=sun_body_unit,
            nadir_body_unit=nadir_body_unit,
            r_sat_m=r_sat_m,
            eclipse_factor=eclipse_factor,
            p_load_w=p_load_w,
            battery_joule_heat_w=battery_joule_heat_w,
            solar_flux_w_m2=solar_flux_w_m2,
        )

        # Sub-stepping for numerical stability if dt_s > 2.0s
        substeps = max(1, int(np.ceil(dt_s / 1.0)))
        dt_sub = dt_s / substeps

        t_curr = self.temperatures.copy()

        for _ in range(substeps):
            k1 = self.compute_derivatives(t_curr, q_in)
            k2 = self.compute_derivatives(t_curr + 0.5 * dt_sub * k1, q_in)
            k3 = self.compute_derivatives(t_curr + 0.5 * dt_sub * k2, q_in)
            k4 = self.compute_derivatives(t_curr + dt_sub * k3, q_in)

            t_curr = t_curr + (dt_sub / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            # Physical sanity clamp: temperature cannot drop below deep space (3K) or exceed 600K
            t_curr = np.clip(t_curr, self.t_space, 600.0)

        self.temperatures = t_curr

        # Construct validated ThermalState schema
        return ThermalState(
            temp_face_pos_x_k=round(float(self.temperatures[0]), 3),
            temp_face_neg_x_k=round(float(self.temperatures[1]), 3),
            temp_face_pos_y_k=round(float(self.temperatures[2]), 3),
            temp_face_neg_y_k=round(float(self.temperatures[3]), 3),
            temp_face_pos_z_k=round(float(self.temperatures[4]), 3),
            temp_face_neg_z_k=round(float(self.temperatures[5]), 3),
            temp_electronics_k=round(float(self.temperatures[6]), 3),
            temp_battery_k=round(float(self.temperatures[7]), 3),
        )
