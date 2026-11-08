"""Unit tests for DOOMSat orbital mechanics, J2 perturbation, SGP4, solar ephemeris, and ground passes."""

import math
import numpy as np
import pytest

from doomsat.core.constants import EARTH_MU, EARTH_RADIUS_EQUATORIAL
from doomsat.orbit import (
    keplerian_to_cartesian,
    cartesian_to_keplerian,
    compute_j2_acceleration,
    NumericalOrbitPropagator,
    SGP4Propagator,
    compute_sun_vector_eci,
    compute_eclipse_factor,
    compute_solar_flux,
    compute_geomagnetic_field_eci,
    GroundStation,
)


def test_keplerian_cartesian_roundtrip():
    """Verify Keplerian elements to Cartesian ECI and back roundtrip conversion."""
    a = 6878137.0  # 500 km altitude
    e = 0.0012
    inc = math.radians(97.4)
    raan = math.radians(45.0)
    argp = math.radians(30.0)
    nu = math.radians(60.0)

    r_eci, v_eci = keplerian_to_cartesian(a, e, inc, raan, argp, nu)
    assert np.linalg.norm(r_eci) > 6800000.0
    assert 7500.0 < np.linalg.norm(v_eci) < 7700.0

    # Convert back
    a_rec, e_rec, inc_rec, raan_rec, argp_rec, nu_rec = cartesian_to_keplerian(r_eci, v_eci)

    assert math.isclose(a, a_rec, rel_tol=1e-8)
    assert math.isclose(e, e_rec, rel_tol=1e-6)
    assert math.isclose(inc, inc_rec, abs_tol=1e-8)
    assert math.isclose(raan, raan_rec, abs_tol=1e-8)
    assert math.isclose(argp, argp_rec, abs_tol=1e-7)
    assert math.isclose(nu, nu_rec, abs_tol=1e-7)


def test_j2_acceleration_calculation():
    """Verify J2 perturbation acceleration magnitude and oblateness geometry."""
    # Equatorial position: z = 0, r = 6878137 m
    r_eq = np.array([6878137.0, 0.0, 0.0], dtype=np.float64)
    a_eq = compute_j2_acceleration(r_eq)

    # Point-mass gravity at r=6878km: g ~= 8.428 m/s^2
    # J2 perturbation is ~ 1e-3 of g ~= 0.012 m/s^2
    # Check total acceleration magnitude
    g_mag = np.linalg.norm(a_eq)
    assert 8.40 < g_mag < 8.46
    # Acceleration vector must point toward Earth center (negative x)
    assert a_eq[0] < 0.0
    assert math.isclose(a_eq[1], 0.0, abs_tol=1e-12)
    assert math.isclose(a_eq[2], 0.0, abs_tol=1e-12)


def test_numerical_rk4_orbital_energy_conservation():
    """Verify that RK4 numerical orbit propagation conserves specific orbital energy for 2-body orbit."""
    # Circular equatorial orbit at 500 km
    r0 = np.array([6878137.0, 0.0, 0.0], dtype=np.float64)
    v_circ = math.sqrt(EARTH_MU / 6878137.0)
    v0 = np.array([0.0, v_circ, 0.0], dtype=np.float64)

    prop = NumericalOrbitPropagator(r0, v0, epoch_time_s=0.0)

    # Orbital period: T = 2 * pi * sqrt(a^3 / mu) ~= 5677 s (~94.6 min)
    period = 2.0 * math.pi * math.sqrt((6878137.0**3) / EARTH_MU)

    # Propagate for 500 seconds in steps of 1.0 s
    dt = 1.0
    steps = 500

    e_initial = (np.linalg.norm(v0)**2 / 2.0) - (EARTH_MU / np.linalg.norm(r0))

    for _ in range(steps):
        prop.step(dt)

    e_final = (np.linalg.norm(prop.v_eci)**2 / 2.0) - (EARTH_MU / np.linalg.norm(prop.r_eci))

    # Energy variation should be less than 1e-6 relative error
    assert math.isclose(e_initial, e_final, rel_tol=1e-6)
    # Radius should remain near 500 km
    assert math.isclose(np.linalg.norm(prop.r_eci), 6878137.0, rel_tol=1e-3)


def test_sgp4_tle_propagation():
    """Verify analytical SGP4 orbit propagation using a standard 1U CubeSat TLE."""
    # Standard ISS / 51.6 deg TLE test set
    line1 = "1 25544U 98067A   23315.54166667  .00016717  00000-0  10270-3 0  9002"
    line2 = "2 25544  51.6416 200.1234 0004567  85.1234 275.0567 15.50000000420001"

    sgp4_prop = SGP4Propagator(line1, line2)
    # Propagate to arbitrary Julian Day
    jd = 2460260.0
    r_m, v_mps = sgp4_prop.propagate_jd(jd, 0.5)

    assert 6700000.0 < np.linalg.norm(r_m) < 6900000.0
    assert 7500.0 < np.linalg.norm(v_mps) < 7800.0


def test_solar_ephemeris_and_eclipse():
    """Verify solar vector unit magnitude, seasonal tilt, and umbra eclipse detection."""
    # Epoch solar vector
    s_vec = compute_sun_vector_eci(time_from_epoch_s=0.0)
    assert math.isclose(np.linalg.norm(s_vec), 1.0, rel_tol=1e-12)

    # Spacecraft on day side (same direction as Sun) -> nu = 1.0
    r_day = s_vec * 6878137.0
    nu_day, in_shadow_day = compute_eclipse_factor(r_day, s_vec)
    assert nu_day == 1.0
    assert not in_shadow_day

    # Spacecraft directly behind Earth (opposite direction) -> total umbra (nu = 0.0)
    r_night = -s_vec * 6878137.0
    nu_night, in_shadow_night = compute_eclipse_factor(r_night, s_vec)
    assert nu_night == 0.0
    assert in_shadow_night

    # Solar flux in sunlight should be ~1361 W/m^2; in eclipse should be 0 W/m^2
    flux_day, _, _ = compute_solar_flux(r_day, s_vec)
    flux_night, _, _ = compute_solar_flux(r_night, s_vec)
    assert math.isclose(flux_day, 1361.0, rel_tol=1e-5)
    assert flux_night == 0.0


def test_geomagnetic_field_magnitude():
    """Verify geomagnetic field strength at 500 km LEO is in the realistic range (20 - 50 uT)."""
    # Equatorial position at 500 km altitude
    r_eci = np.array([6878137.0, 0.0, 0.0], dtype=np.float64)
    b_vec = compute_geomagnetic_field_eci(r_eci, gmst_rad=0.0)

    b_norm_tesla = np.linalg.norm(b_vec)
    # B should be between 20 uT (20e-6 T) and 60 uT (60e-6 T)
    assert 20.0e-6 < b_norm_tesla < 60.0e-6


def test_ground_station_look_angles():
    """Verify ground station look angles calculation and visibility pass detection."""
    gs = GroundStation(name="Hyderabad-GS", latitude_deg=17.3850, longitude_deg=78.4867)

    # Spacecraft directly overhead at zenith
    # ECEF position of GS + 500 km directly upward along Up vector
    up_direction = gs.dcm_ecef_to_neu[2, :]  # Up vector in ECEF
    r_sat_ecef = gs.r_gs_ecef + up_direction * 500_000.0

    # With gmst = 0, ECI == ECEF
    look = gs.compute_look_angles(
        r_sat_eci=r_sat_ecef,
        v_sat_eci=np.array([0.0, 7600.0, 0.0]),
        gmst_rad=0.0,
    )

    # Zenith elevation should be 90 degrees
    assert math.isclose(look.elevation_deg, 90.0, abs_tol=0.1)
    assert math.isclose(look.slant_range_m, 500_000.0, rel_tol=1e-4)
    assert look.is_visible
