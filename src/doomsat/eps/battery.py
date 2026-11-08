"""Lithium-ion battery equivalent circuit model for DOOMSat 1U CubeSat.

Models a Samsung INR18650-25R 2500 mAh Li-ion cylindrical cell:
- Coulomb counting State of Charge (SoC) integration with charge/discharge efficiencies
- Monotonic Open-Circuit Voltage polynomial Voc(SoC) matching datasheet curves
- Temperature-dependent Equivalent Series Resistance (ESR) R_int(T)
- Dynamic terminal voltage V_term = Voc + I_batt * R_int
- Current clamping, voltage limits (3.2V - 4.2V), and Joule heating dissipation.
"""

from typing import Tuple
import math


class LiIonBattery:
    """Equivalent circuit model of Samsung INR18650-25R 1S1P Li-ion cell."""

    def __init__(
        self,
        nominal_capacity_mah: float = 2500.0,
        nominal_voltage_v: float = 3.6,
        charge_voltage_limit_v: float = 4.20,
        discharge_cutoff_v: float = 3.20,
        initial_soc_pct: float = 95.0,
        internal_resistance_ohm: float = 0.040,
        charge_efficiency: float = 0.96,
        discharge_efficiency: float = 0.99,
        max_charge_current_a: float = 1.25,      # 0.5C safe space charge rate
        max_discharge_current_a: float = 5.0,     # 2C max discharge rate
    ) -> None:
        self.capacity_mah = nominal_capacity_mah
        self.capacity_coulombs = nominal_capacity_mah * 3.6  # 1 mAh = 3.6 C
        self.nominal_voltage_v = nominal_voltage_v
        self.charge_voltage_limit_v = charge_voltage_limit_v
        self.discharge_cutoff_v = discharge_cutoff_v
        self.soc_pct = float(max(0.0, min(100.0, initial_soc_pct)))
        self.r_int_ref_ohm = internal_resistance_ohm
        self.charge_efficiency = charge_efficiency
        self.discharge_efficiency = discharge_efficiency
        self.max_charge_current_a = max_charge_current_a
        self.max_discharge_current_a = max_discharge_current_a

    def compute_internal_resistance(self, temperature_k: float = 298.15) -> float:
        """Compute temperature-adjusted internal series resistance.

        Equation:
            R_int(T) = R_0 * [1 + alpha_R * max(0, 298.15 - T)]

        Args:
            temperature_k: Battery cell temperature [K].

        Returns:
            Internal resistance [Ohm].
        """
        # Resistance increases below 25°C (298.15 K) due to reduced electrolyte mobility
        cold_delta = max(0.0, 298.15 - temperature_k)
        r_temp = self.r_int_ref_ohm * (1.0 + 0.015 * cold_delta)
        return float(r_temp)

    def compute_open_circuit_voltage(self, soc_pct: float) -> float:
        """Compute open-circuit voltage Voc as a function of State of Charge.

        Empirical cubic polynomial fitted to Samsung INR18650-25R discharge curves:
            s = SoC / 100 in [0, 1]
            Voc(s) = 3.20 + 0.90 * s - 0.20 * s^2 + 0.30 * s^3

        Boundary points:
            SoC =   0% -> Voc = 3.200 V
            SoC =  50% -> Voc = 3.638 V
            SoC = 100% -> Voc = 4.200 V

        Args:
            soc_pct: Battery State of Charge in percentage [0.0, 100.0].

        Returns:
            Open-circuit voltage [V].
        """
        s = max(0.0, min(1.0, soc_pct / 100.0))
        voc = 3.20 + 0.90 * s - 0.20 * (s ** 2) + 0.30 * (s ** 3)
        return float(voc)

    def compute_terminal_voltage(
        self,
        current_a: float,
        temperature_k: float = 298.15,
    ) -> float:
        """Compute instantaneous terminal voltage under load.

        Equation:
            V_term = Voc(SoC) + I_batt * R_int(T)
            where:
                I_batt > 0: charging (V_term > Voc)
                I_batt < 0: discharging (V_term < Voc)

        Args:
            current_a: Battery current [A] (positive = charging, negative = discharging).
            temperature_k: Cell temperature [K].

        Returns:
            Terminal voltage [V].
        """
        voc = self.compute_open_circuit_voltage(self.soc_pct)
        r_int = self.compute_internal_resistance(temperature_k)
        v_term = voc + current_a * r_int
        return float(v_term)

    def step(
        self,
        target_current_a: float,
        dt_s: float,
        temperature_k: float = 298.15,
    ) -> Tuple[float, float, float, float]:
        """Integrate battery state over time step dt_s with current and voltage limits.

        Args:
            target_current_a: Desired battery current [A] (+ for charge, - for discharge).
            dt_s: Time step in seconds.
            temperature_k: Cell temperature [K].

        Returns:
            Tuple of:
                actual_current_a: Effective clamped battery current [A].
                terminal_voltage_v: Instantaneous terminal voltage [V].
                soc_pct: Updated battery State of Charge [%].
                joule_heat_w: Internal I^2 * R thermal power dissipated [W].
        """
        r_int = self.compute_internal_resistance(temperature_k)
        voc = self.compute_open_circuit_voltage(self.soc_pct)

        # 1. Clamp target current to safe physical operating bounds
        if target_current_a > 0.0:
            # Charging: clamp to max charge current
            current = min(target_current_a, self.max_charge_current_a)
            # If already at 100% SoC or Voc >= limit, cutoff charging
            if self.soc_pct >= 100.0 or voc >= self.charge_voltage_limit_v:
                current = 0.0
            # If terminal voltage exceeds 4.2V, taper current (CV mode approximation)
            v_term_est = voc + current * r_int
            if v_term_est > self.charge_voltage_limit_v:
                current = max(0.0, (self.charge_voltage_limit_v - voc) / r_int)
        elif target_current_a < 0.0:
            # Discharging: clamp to max discharge current
            current = max(target_current_a, -self.max_discharge_current_a)
            # If at 0% SoC or Voc <= cutoff, cutoff discharging
            if self.soc_pct <= 0.0 or voc <= self.discharge_cutoff_v:
                current = 0.0
            # If terminal voltage drops below 3.2V, taper current
            v_term_est = voc + current * r_int
            if v_term_est < self.discharge_cutoff_v:
                current = min(0.0, (self.discharge_cutoff_v - voc) / r_int)
        else:
            current = 0.0

        # 2. Coulomb counting integration
        if current > 0.0:
            delta_q = current * dt_s * self.charge_efficiency
        elif current < 0.0:
            delta_q = (current * dt_s) / self.discharge_efficiency
        else:
            delta_q = 0.0

        delta_soc = (delta_q / self.capacity_coulombs) * 100.0
        self.soc_pct = max(0.0, min(100.0, self.soc_pct + delta_soc))

        # 3. Terminal voltage and Joule heating
        v_term = self.compute_open_circuit_voltage(self.soc_pct) + current * r_int
        v_term = max(self.discharge_cutoff_v, min(self.charge_voltage_limit_v, v_term))
        joule_heat_w = (current ** 2) * r_int

        return current, v_term, self.soc_pct, joule_heat_w

