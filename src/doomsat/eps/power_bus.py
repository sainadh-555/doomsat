"""Power distribution bus and load management for DOOMSat 1U CubeSat.

Models regulated 3.3V and 5.0V power buses, flight-mode electrical load profiles,
DC-DC converter and battery charge/discharge regulator efficiencies, and
Low-Voltage Disconnect (LVD) with hysteresis protection.
"""

from typing import Dict, Optional, Tuple
from doomsat.core.schemas import PowerState
from doomsat.eps.battery import LiIonBattery


class PowerDistributionBus:
    """Manages regulated DC buses, subsystem load draws, and battery power balance."""

    # Nominal flight mode electrical loads [W]
    DEFAULT_LOAD_PROFILES: Dict[str, float] = {
        "BOOT": 0.40,
        "SAFE_HOLD": 1.15,
        "DETUMBLE": 1.15,
        "NOMINAL": 0.75,
        "IMAGE_CAPTURE": 2.20,
        "TRANSMIT_TELEMETRY": 1.45,
        "TRANSMIT_IMAGE": 1.65,
        "CRITICAL_LOW_POWER": 0.15,
    }

    def __init__(
        self,
        battery: Optional[LiIonBattery] = None,
        efficiency_3v3: float = 0.90,
        efficiency_5v0: float = 0.88,
        efficiency_bcr: float = 0.94,  # Battery Charge Regulator efficiency
        efficiency_bdr: float = 0.95,  # Battery Discharge Regulator efficiency
        lvd_trip_soc_pct: float = 20.0,
        lvd_recovery_soc_pct: float = 25.0,
        lvd_trip_voltage_v: float = 3.20,
        lvd_recovery_voltage_v: float = 3.40,
    ) -> None:
        self.battery = battery if battery is not None else LiIonBattery()
        self.efficiency_3v3 = efficiency_3v3
        self.efficiency_5v0 = efficiency_5v0
        self.efficiency_bcr = efficiency_bcr
        self.efficiency_bdr = efficiency_bdr

        self.lvd_trip_soc = lvd_trip_soc_pct
        self.lvd_recovery_soc = lvd_recovery_soc_pct
        self.lvd_trip_voltage = lvd_trip_voltage_v
        self.lvd_recovery_voltage = lvd_recovery_voltage_v

        self.is_lvd_tripped: bool = False
        self.load_profiles: Dict[str, float] = dict(self.DEFAULT_LOAD_PROFILES)

    def get_load_power(self, flight_mode: str) -> float:
        """Get instantaneous subsystem power demand for the given flight mode.

        Args:
            flight_mode: OBC flight software FSM state name.

        Returns:
            Subsystem electrical load drawn [W].
        """
        if self.is_lvd_tripped:
            return self.load_profiles["CRITICAL_LOW_POWER"]

        return self.load_profiles.get(flight_mode, self.load_profiles["NOMINAL"])

    def update_lvd_state(self, current_soc: float, current_voltage: float) -> bool:
        """Update Low-Voltage Disconnect (LVD) state with hysteresis.

        Args:
            current_soc: Battery State of Charge [%].
            current_voltage: Battery terminal voltage [V].

        Returns:
            True if LVD is active (non-essential loads shed).
        """
        if not self.is_lvd_tripped:
            if current_soc <= self.lvd_trip_soc or current_voltage <= self.lvd_trip_voltage:
                self.is_lvd_tripped = True
        else:
            if current_soc >= self.lvd_recovery_soc and current_voltage >= self.lvd_recovery_voltage:
                self.is_lvd_tripped = False

        return self.is_lvd_tripped

    def step(
        self,
        solar_power_w: float,
        flight_mode: str,
        dt_s: float,
        battery_temp_k: float = 298.15,
    ) -> Tuple[PowerState, float]:
        """Perform one discrete time step of the EPS power distribution bus.

        Args:
            solar_power_w: Solar array generation after MPPT conversion [W].
            flight_mode: Active OBC flight mode name.
            dt_s: Integration time step [s].
            battery_temp_k: Instantaneous battery temperature [K].

        Returns:
            Tuple of (PowerState schema instance, battery_joule_heat_w).
        """
        # 1. Update LVD status with current battery state before determining load
        voc_curr = self.battery.compute_open_circuit_voltage(self.battery.soc_pct)
        self.update_lvd_state(current_soc=self.battery.soc_pct, current_voltage=voc_curr)

        # 2. Determine load power considering LVD state
        load_power_w = self.get_load_power(flight_mode)

        # 3. Compute net electrical power balance
        p_net = solar_power_w - load_power_w

        # 4. Compute target battery current based on power balance
        voc = self.battery.compute_open_circuit_voltage(self.battery.soc_pct)
        # Avoid division by zero
        v_est = max(3.0, voc)

        if p_net > 0.0:
            # Surplus solar power charges battery through BCR
            p_charge = p_net * self.efficiency_bcr
            target_current = p_charge / v_est
        elif p_net < 0.0:
            # Deficit drawn from battery through BDR
            p_discharge = abs(p_net) / self.efficiency_bdr
            target_current = -p_discharge / v_est
        else:
            target_current = 0.0

        # 5. Step battery model
        actual_current, v_term, soc_pct, joule_heat_w = self.battery.step(
            target_current_a=target_current,
            dt_s=dt_s,
            temperature_k=battery_temp_k,
        )

        # 6. Update LVD status with new battery state
        self.update_lvd_state(current_soc=soc_pct, current_voltage=v_term)

        # 7. Construct validated PowerState schema
        bus_3v3 = 3.30 if (not self.is_lvd_tripped or flight_mode == "CRITICAL_LOW_POWER") else 0.0
        bus_5v0 = 5.00 if not self.is_lvd_tripped else 0.0

        power_state = PowerState(
            battery_soc_pct=round(soc_pct, 4),
            battery_voltage_v=round(v_term, 4),
            battery_current_a=round(actual_current, 5),
            solar_power_w=round(solar_power_w, 4),
            bus_3v3_v=bus_3v3,
            bus_5v0_v=bus_5v0,
            system_load_w=round(load_power_w, 4),
        )

        return power_state, joule_heat_w
