"""Tests for Electrical Power Subsystem (EPS): solar array, Li-ion battery, and power bus."""

import math
import numpy as np
import pytest

from doomsat.eps.solar import SolarPanelFace, SolarArray
from doomsat.eps.battery import LiIonBattery
from doomsat.eps.power_bus import PowerDistributionBus
from doomsat.core.schemas import PowerState


def test_solar_panel_illumination_and_temperature_derating():
    """Verify single panel and array generation across angles, temperature, and eclipse."""
    panel = SolarPanelFace(
        name="+X",
        normal_vector_body=np.array([1.0, 0.0, 0.0]),
        active_area_m2=0.007,
        nominal_efficiency=0.285,
    )

    # 1. Normal incidence at 25°C (298.15K)
    sun_normal = np.array([1.0, 0.0, 0.0])
    p_inc, cos_theta = panel.compute_incident_power(sun_normal, solar_flux_w_m2=1361.0, eclipse_factor=1.0)
    assert math.isclose(cos_theta, 1.0, abs_tol=1e-6)
    expected_p_inc = 0.007 * 1361.0
    assert math.isclose(p_inc, expected_p_inc, rel_tol=1e-5)

    p_elec = panel.compute_electrical_output(sun_normal, temperature_k=298.15)
    expected_p_elec = expected_p_inc * 0.285
    assert math.isclose(p_elec, expected_p_elec, rel_tol=1e-5)

    # 2. 60 degree incidence (cos 60° = 0.5)
    sun_60deg = np.array([0.5, math.sqrt(0.75), 0.0])
    p_elec_60 = panel.compute_electrical_output(sun_60deg, temperature_k=298.15)
    assert math.isclose(p_elec_60, expected_p_elec * 0.5, rel_tol=1e-4)

    # 3. Panel facing away from Sun (shadow side)
    sun_back = np.array([-1.0, 0.0, 0.0])
    p_elec_back = panel.compute_electrical_output(sun_back, temperature_k=298.15)
    assert p_elec_back == 0.0

    # 4. Temperature efficiency derating (+50°C above ref: 348.15 K)
    # beta_T = -0.0020/K -> delta = -10% relative
    p_elec_hot = panel.compute_electrical_output(sun_normal, temperature_k=348.15)
    assert p_elec_hot < p_elec
    assert math.isclose(p_elec_hot, p_elec * 0.90, rel_tol=1e-4)

    # 5. Eclipse attenuation
    p_elec_eclipse = panel.compute_electrical_output(sun_normal, eclipse_factor=0.0)
    assert p_elec_eclipse == 0.0

    # 6. Complete 5-panel array test
    array = SolarArray(panel_area_m2=0.007, cell_efficiency=0.285, mppt_efficiency=0.92)
    mppt_power, face_powers = array.compute_generation(sun_normal)
    assert face_powers["+X"] > 0.0
    assert face_powers["-X"] == 0.0
    assert face_powers["+Y"] == 0.0
    assert face_powers["-Y"] == 0.0
    assert face_powers["+Z"] == 0.0
    # MPPT efficiency check
    assert math.isclose(mppt_power, face_powers["+X"] * 0.92, rel_tol=1e-5)

    # -Z face has no solar cells (camera lens aperture)
    sun_neg_z = np.array([0.0, 0.0, -1.0])
    mppt_neg_z, _ = array.compute_generation(sun_neg_z)
    assert mppt_neg_z == 0.0


def test_battery_coulomb_counting_and_soc_tracking():
    """Verify battery State of Charge Coulomb counting over charge and discharge cycles."""
    batt = LiIonBattery(
        nominal_capacity_mah=2500.0,
        initial_soc_pct=50.0,
        charge_efficiency=0.96,
        discharge_efficiency=0.99,
    )

    # Capacity in Coulombs = 2500 * 3.6 = 9000 Coulombs
    assert math.isclose(batt.capacity_coulombs, 9000.0, abs_tol=1e-5)

    # Discharge at 1.0 A for 900 seconds (15 min)
    # Charge drawn = 900 C / 0.99 = 909.09 C
    # Delta SoC = 909.09 / 9000 * 100% = 10.101%
    curr, v_term, soc, joule = batt.step(target_current_a=-1.0, dt_s=900.0)
    assert curr == -1.0
    expected_soc = 50.0 - (900.0 / 0.99 / 9000.0) * 100.0
    assert math.isclose(soc, expected_soc, abs_tol=0.05)
    assert joule > 0.0

    # Charge at 1.0 A for 900 seconds
    # Charge added = 900 C * 0.96 = 864 C
    # Delta SoC = 864 / 9000 * 100% = 9.60%
    curr_c, _, soc_after_charge, _ = batt.step(target_current_a=1.0, dt_s=900.0)
    assert curr_c == 1.0
    expected_charge_soc = expected_soc + (900.0 * 0.96 / 9000.0) * 100.0
    assert math.isclose(soc_after_charge, expected_charge_soc, abs_tol=0.05)


def test_battery_ocv_and_terminal_voltage_curve():
    """Verify monotonic open-circuit voltage curve and internal resistance IR drops."""
    batt = LiIonBattery(internal_resistance_ohm=0.040)

    # Boundary open-circuit voltages
    voc_0 = batt.compute_open_circuit_voltage(0.0)
    voc_50 = batt.compute_open_circuit_voltage(50.0)
    voc_100 = batt.compute_open_circuit_voltage(100.0)

    assert math.isclose(voc_0, 3.20, abs_tol=1e-3)
    assert math.isclose(voc_100, 4.20, abs_tol=1e-3)
    assert 3.55 < voc_50 < 3.75

    # Monotonicity check across SoC spectrum
    soc_points = np.linspace(0.0, 100.0, 50)
    vocs = [batt.compute_open_circuit_voltage(s) for s in soc_points]
    for i in range(len(vocs) - 1):
        assert vocs[i + 1] > vocs[i]

    # Terminal voltage under load (IR drop)
    batt.soc_pct = 50.0
    v_oc = batt.compute_open_circuit_voltage(50.0)
    r_int = batt.compute_internal_resistance(298.15)
    assert math.isclose(r_int, 0.040, abs_tol=1e-5)

    # Discharging 2.0 A: V_term = Voc - 2 * 0.040
    v_dis = batt.compute_terminal_voltage(current_a=-2.0, temperature_k=298.15)
    assert math.isclose(v_dis, v_oc - 2.0 * 0.040, rel_tol=1e-5)

    # Charging 1.0 A: V_term = Voc + 1 * 0.040
    v_chg = batt.compute_terminal_voltage(current_a=1.0, temperature_k=298.15)
    assert math.isclose(v_chg, v_oc + 1.0 * 0.040, rel_tol=1e-5)


def test_battery_charge_discharge_limits():
    """Verify current clamping at max ratings and cutoff at 100% and 0% SoC."""
    batt = LiIonBattery(
        max_charge_current_a=1.25,
        max_discharge_current_a=5.0,
        initial_soc_pct=100.0,
    )

    # At 100% SoC, further charging current must be cut off
    curr, _, soc, _ = batt.step(target_current_a=2.0, dt_s=10.0)
    assert curr == 0.0
    assert soc == 100.0

    # Over-rate charge requested when at 50% SoC must clamp to 1.25 A
    batt.soc_pct = 50.0
    curr_clamp, _, _, _ = batt.step(target_current_a=3.0, dt_s=10.0)
    assert math.isclose(curr_clamp, 1.25, abs_tol=1e-5)

    # Over-rate discharge requested must clamp to -5.0 A
    curr_dis_clamp, _, _, _ = batt.step(target_current_a=-10.0, dt_s=10.0)
    assert math.isclose(curr_dis_clamp, -5.0, abs_tol=1e-5)

    # At 0% SoC, discharge current must be cut off
    batt.soc_pct = 0.0
    curr_empty, _, soc_empty, _ = batt.step(target_current_a=-1.0, dt_s=10.0)
    assert curr_empty == 0.0
    assert soc_empty == 0.0


def test_power_distribution_bus_efficiency_and_loads():
    """Verify power bus load mapping, net balance, and PowerState schema generation."""
    bus = PowerDistributionBus()

    # Load power profile checks
    assert math.isclose(bus.get_load_power("BOOT"), 0.40)
    assert math.isclose(bus.get_load_power("NOMINAL"), 0.75)
    assert math.isclose(bus.get_load_power("IMAGE_CAPTURE"), 2.20)
    assert math.isclose(bus.get_load_power("TRANSMIT_TELEMETRY"), 1.45)
    assert math.isclose(bus.get_load_power("CRITICAL_LOW_POWER"), 0.15)

    # Surplus solar power: generation 2.0 W, NOMINAL load 0.75 W -> Net +1.25 W (charging)
    state_surplus, joule_heat = bus.step(solar_power_w=2.0, flight_mode="NOMINAL", dt_s=1.0)
    assert isinstance(state_surplus, PowerState)
    assert state_surplus.battery_current_a > 0.0
    assert math.isclose(state_surplus.solar_power_w, 2.0)
    assert math.isclose(state_surplus.system_load_w, 0.75)
    assert state_surplus.bus_3v3_v == 3.3
    assert state_surplus.bus_5v0_v == 5.0

    # Deficit power (eclipse): generation 0.0 W, NOMINAL load 0.75 W -> Net -0.75 W (discharging)
    state_deficit, _ = bus.step(solar_power_w=0.0, flight_mode="NOMINAL", dt_s=1.0)
    assert state_deficit.battery_current_a < 0.0
    assert math.isclose(state_deficit.solar_power_w, 0.0)
    assert math.isclose(state_deficit.system_load_w, 0.75)


def test_low_voltage_disconnect_hysteresis():
    """Verify Low-Voltage Disconnect triggers at <=20% SoC and recovers at >=25% SoC."""
    bus = PowerDistributionBus(lvd_trip_soc_pct=20.0, lvd_recovery_soc_pct=25.0)

    # Initial healthy state
    assert not bus.is_lvd_tripped

    # Drop to 19.5% SoC -> LVD trips
    bus.battery.soc_pct = 19.5
    state, _ = bus.step(solar_power_w=0.0, flight_mode="NOMINAL", dt_s=1.0)
    assert bus.is_lvd_tripped
    # When tripped, load power drops to CRITICAL_LOW_POWER (0.15 W)
    assert math.isclose(state.system_load_w, 0.15)
    assert state.bus_5v0_v == 0.0  # 5V rail turned off

    # Recover slightly to 23.0% SoC -> LVD must remain tripped (hysteresis)
    bus.battery.soc_pct = 23.0
    state2, _ = bus.step(solar_power_w=1.0, flight_mode="NOMINAL", dt_s=1.0)
    assert bus.is_lvd_tripped
    assert math.isclose(state2.system_load_w, 0.15)

    # Rise to 26.0% SoC (and voltage > 3.45V) -> LVD recovers
    bus.battery.soc_pct = 26.0
    state3, _ = bus.step(solar_power_w=1.5, flight_mode="NOMINAL", dt_s=1.0)
    assert not bus.is_lvd_tripped
    assert math.isclose(state3.system_load_w, 0.75)
    assert state3.bus_5v0_v == 5.0


def test_eps_orbit_energy_balance():
    """Simulate a full 90-minute orbit and verify positive net energy margin in NOMINAL mode."""
    # 60 min sunlight (avg 1.8 W MPPT solar), 30 min eclipse (0 W)
    bus = PowerDistributionBus()
    bus.battery.soc_pct = 85.0

    dt = 10.0  # 10s simulation steps
    sunlight_steps = int(3600 / dt)
    eclipse_steps = int(1800 / dt)

    # Sunlight phase
    for _ in range(sunlight_steps):
        bus.step(solar_power_w=1.8, flight_mode="NOMINAL", dt_s=dt)

    soc_sunlight = bus.battery.soc_pct
    assert soc_sunlight > 85.0

    # Eclipse phase
    for _ in range(eclipse_steps):
        bus.step(solar_power_w=0.0, flight_mode="NOMINAL", dt_s=dt)

    soc_final = bus.battery.soc_pct
    # Net energy generation over orbit is positive, so end SoC >= initial SoC
    assert soc_final >= 85.0
    assert not bus.is_lvd_tripped

