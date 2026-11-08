"""Tests for Thermal subsystem: 8-node lumped parameter network, conduction, radiation, and orbital cycle."""

import math
import numpy as np
import pytest

from doomsat.thermal.nodal_model import ThermalNetwork8Node
from doomsat.core.constants import STEFAN_BOLTZMANN
from doomsat.core.schemas import ThermalState


def test_nodal_thermal_capacities_and_conductances():
    """Verify 8-node thermal network parameters, symmetric conductance, and positive capacitances."""
    model = ThermalNetwork8Node()

    assert len(model.NODE_NAMES) == 8
    assert len(model.capacitances) == 8

    # Verify panel capacitance (45g Al6061-T6)
    assert math.isclose(model.capacitances[0], 40.32, abs_tol=0.1)
    # Verify electronics stack capacitance (200g)
    assert math.isclose(model.capacitances[6], 170.0, abs_tol=0.1)
    # Verify battery cell capacitance (45g)
    assert math.isclose(model.capacitances[7], 45.0, abs_tol=0.1)

    # Verify conductance matrix symmetry: K_ij == K_ji
    assert model.k_matrix.shape == (8, 8)
    for i in range(8):
        for j in range(8):
            assert math.isclose(model.k_matrix[i, j], model.k_matrix[j, i], abs_tol=1e-9)

    # Verify key conductances exist
    assert model.k_matrix[0, 2] > 0.0  # +X to +Y corner rail
    assert model.k_matrix[6, 0] > 0.0  # Electronics to +X panel standoff
    assert model.k_matrix[7, 2] > 0.0  # Battery to +Y panel clamp


def test_solar_albedo_earth_ir_flux_calculations():
    """Verify environmental heat inputs under direct sunlight, nadir alignment, and eclipse."""
    model = ThermalNetwork8Node(solar_absorptivity_alpha=0.90, ir_emissivity_epsilon=0.82)

    # 1. Pure solar verification without Nadir
    sun_body = np.array([1.0, 0.0, 0.0])
    r_sat = 6878137.0  # 500 km altitude orbit

    q_in_pure_solar = model.compute_heat_inputs(
        sun_body_unit=sun_body,
        nadir_body_unit=None,
        r_sat_m=r_sat,
        eclipse_factor=1.0,
        p_load_w=0.75,
        battery_joule_heat_w=0.05,
        solar_flux_w_m2=1361.0,
    )

    # Node 0 (+X) receives direct solar: alpha * A * S0 = 0.90 * 0.010 * 1361 = 12.249 W
    assert math.isclose(q_in_pure_solar[0], 0.90 * 0.010 * 1361.0, rel_tol=1e-4)

    # Node 1 (-X) receives no direct solar
    assert q_in_pure_solar[1] < 1e-4

    # 2. Combined solar + Earth IR verification with Nadir along -Z
    nadir_body = np.array([0.0, 0.0, -1.0])
    q_in = model.compute_heat_inputs(
        sun_body_unit=sun_body,
        nadir_body_unit=nadir_body,
        r_sat_m=r_sat,
        eclipse_factor=1.0,
        p_load_w=0.75,
        battery_joule_heat_w=0.05,
        solar_flux_w_m2=1361.0,
    )

    # Node 5 (-Z) faces Nadir: receives planetary Earth IR
    assert q_in[5] > 0.5  # Significant Earth-IR flux

    # Electronics and battery internal dissipation
    assert math.isclose(q_in[6], 0.75, abs_tol=1e-4)
    assert math.isclose(q_in[7], 0.05, abs_tol=1e-4)

    # In eclipse (nu = 0): direct solar drops to 0, Earth IR persists
    q_in_eclipse = model.compute_heat_inputs(
        sun_body_unit=sun_body,
        nadir_body_unit=nadir_body,
        r_sat_m=r_sat,
        eclipse_factor=0.0,
        p_load_w=0.75,
        battery_joule_heat_w=0.0,
    )
    # Solar flux is removed, only Earth IR remains
    assert math.isclose(q_in_eclipse[0], q_in[0] - 0.90 * 0.010 * 1361.0, rel_tol=1e-4)
    assert q_in_eclipse[5] > 0.0   # Earth IR still present in eclipse


def test_deep_space_radiation_equilibrium():
    """Verify analytical Stefan-Boltzmann radiative equilibrium condition."""
    model = ThermalNetwork8Node(
        solar_absorptivity_alpha=0.90,
        ir_emissivity_epsilon=0.82,
        face_area_m2=0.010,
    )

    # Suppose a face receives constant Q_in = 5.0 W with negligible conduction
    # Radiative equilibrium: Q_in = sigma * epsilon * A * (T_eq^4 - T_space^4)
    q_test = 5.0
    sigma_eps_a = STEFAN_BOLTZMANN * 0.82 * 0.010
    t_eq = (q_test / sigma_eps_a) ** 0.25  # ~ 322.0 K

    # Set all nodes to t_eq and inject q_test on node 0 with no conduction
    t_uniform = np.full(8, t_eq)
    q_in = np.zeros(8)
    q_in[0] = q_test

    dT_dt = model.compute_derivatives(t_uniform, q_in)
    # At radiative equilibrium with uniform temperature, net dT/dt on face 0 must be near zero
    assert abs(dT_dt[0]) < 0.05


def test_thermal_rk4_integration_stability():
    """Verify RK4 integration stability and schema generation over 300 seconds."""
    model = ThermalNetwork8Node()

    sun_body = np.array([0.7071, 0.7071, 0.0])
    nadir_body = np.array([0.0, 0.0, -1.0])

    for step_idx in range(150):
        state = model.step(
            sun_body_unit=sun_body,
            nadir_body_unit=nadir_body,
            r_sat_m=6878137.0,
            eclipse_factor=1.0,
            p_load_w=0.75,
            battery_joule_heat_w=0.02,
            dt_s=2.0,
        )

    assert isinstance(state, ThermalState)
    # Verify all temperatures remain within physically valid bounds
    temps = [
        state.temp_face_pos_x_k, state.temp_face_neg_x_k,
        state.temp_face_pos_y_k, state.temp_face_neg_y_k,
        state.temp_face_pos_z_k, state.temp_face_neg_z_k,
        state.temp_electronics_k, state.temp_battery_k,
    ]
    for t in temps:
        assert 200.0 < t < 400.0


def test_internal_dissipation_joule_heating():
    """Verify that continuous avionics dissipation keeps internal stack warmer than outer panels in eclipse."""
    model = ThermalNetwork8Node(
        initial_temperatures_k={name: 290.0 for name in ThermalNetwork8Node.NODE_NAMES}
    )

    # In eclipse (no solar) with 2.5 W internal electronics load for 600 s (10 min)
    for _ in range(60):
        state = model.step(
            sun_body_unit=None,
            nadir_body_unit=np.array([0.0, 0.0, -1.0]),
            r_sat_m=6878137.0,
            eclipse_factor=0.0,
            p_load_w=2.5,
            battery_joule_heat_w=0.10,
            dt_s=10.0,
        )

    # Internal electronics stack is receiving 2.5W and conducts heat outwards to panels
    assert state.temp_electronics_k > state.temp_face_pos_x_k
    assert state.temp_electronics_k > state.temp_face_neg_x_k


def test_full_orbit_thermal_cycle_bounds():
    """Simulate a complete 95-minute orbit (sunlight + eclipse) and verify survival envelopes."""
    model = ThermalNetwork8Node(
        initial_temperatures_k={
            "face_pos_x": 290.0,
            "face_neg_x": 290.0,
            "face_pos_y": 290.0,
            "face_neg_y": 290.0,
            "face_pos_z": 290.0,
            "face_neg_z": 290.0,
            "electronics": 295.0,
            "battery": 293.0,
        }
    )

    # 60 min sunlight (3600 s), 35 min eclipse (2100 s) = 5700 s
    dt = 10.0
    sun_steps = int(3600 / dt)
    eclipse_steps = int(2100 / dt)

    nadir_vec = np.array([0.0, 0.0, -1.0])
    r_sat = 6878137.0
    omega_0 = 2.0 * math.pi / 5700.0

    min_battery_temp = 999.0
    max_battery_temp = -999.0
    min_elec_temp = 999.0
    max_elec_temp = -999.0

    # 1. Sunlight phase (orbital pitch rotation relative to Sun)
    for i in range(sun_steps):
        t_sec = i * dt
        sun_vec = np.array([math.cos(omega_0 * t_sec), 0.0, math.sin(omega_0 * t_sec)])
        state = model.step(
            sun_body_unit=sun_vec,
            nadir_body_unit=nadir_vec,
            r_sat_m=r_sat,
            eclipse_factor=1.0,
            p_load_w=0.75,
            battery_joule_heat_w=0.03,
            dt_s=dt,
        )
        min_battery_temp = min(min_battery_temp, state.temp_battery_k)
        max_battery_temp = max(max_battery_temp, state.temp_battery_k)
        min_elec_temp = min(min_elec_temp, state.temp_electronics_k)
        max_elec_temp = max(max_elec_temp, state.temp_electronics_k)

    # In sunlight (charging mode), battery must stay within [0°C, 45°C] = [273.15K, 318.15K]
    assert min_battery_temp >= 273.15, f"Sunlight battery too cold: {min_battery_temp} K"
    assert max_battery_temp <= 318.15, f"Sunlight battery too hot: {max_battery_temp} K"

    # 2. Eclipse phase
    for i in range(eclipse_steps):
        t_sec = 3600.0 + i * dt
        sun_vec = np.array([math.cos(omega_0 * t_sec), 0.0, math.sin(omega_0 * t_sec)])
        state = model.step(
            sun_body_unit=sun_vec,
            nadir_body_unit=nadir_vec,
            r_sat_m=r_sat,
            eclipse_factor=0.0,
            p_load_w=0.75,
            battery_joule_heat_w=0.02,
            dt_s=dt,
        )
        min_battery_temp = min(min_battery_temp, state.temp_battery_k)
        max_battery_temp = max(max_battery_temp, state.temp_battery_k)
        min_elec_temp = min(min_elec_temp, state.temp_electronics_k)
        max_elec_temp = max(max_elec_temp, state.temp_electronics_k)

    # In eclipse (discharging mode), battery must remain above -20°C (253.15 K)
    assert min_battery_temp >= 253.15, f"Battery fell too cold in eclipse: {min_battery_temp} K"

    # Verify electronics stays within [-20°C, +70°C] = [253.15K, 343.15K] throughout entire orbit
    assert min_elec_temp >= 253.15, f"Electronics fell too cold: {min_elec_temp} K"
    assert max_elec_temp <= 343.15, f"Electronics grew too hot: {max_elec_temp} K"
