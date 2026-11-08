"""Unit tests for coordinate transformations, frames, and attitude kinematics."""

import math
import numpy as np
import pytest
from doomsat.core import coordinates, constants


def test_eci_ecef_roundtrip():
    """Verify that ECI -> ECEF -> ECI transformation is lossless."""
    r_eci = np.array([7000000.0, -1500000.0, 3200000.0], dtype=np.float64)
    gmst = 1.2345678  # rad

    r_ecef = coordinates.eci_to_ecef(r_eci, gmst)
    r_eci_back = coordinates.ecef_to_eci(r_ecef, gmst)

    # Norm must be preserved exactly
    assert math.isclose(np.linalg.norm(r_eci), np.linalg.norm(r_ecef), rel_tol=1e-12)
    # Reconstructed vector must match within millimeter precision
    assert np.allclose(r_eci, r_eci_back, atol=1e-6)
    # Z-axis should remain identical in ECI and ECEF
    assert math.isclose(r_eci[2], r_ecef[2], rel_tol=1e-12)


def test_geodetic_canonical_points():
    """Verify WGS-84 geodetic to ECEF conversions on known landmark coordinates."""
    # 1. Equator & Prime Meridian: lat=0, lon=0, alt=0
    r_eq = coordinates.geodetic_to_ecef(0.0, 0.0, 0.0)
    assert math.isclose(r_eq[0], constants.EARTH_RADIUS_EQUATORIAL, abs_tol=1e-4)
    assert math.isclose(r_eq[1], 0.0, abs_tol=1e-4)
    assert math.isclose(r_eq[2], 0.0, abs_tol=1e-4)

    # 2. North Pole: lat = pi/2, lon = 0, alt = 0
    r_pole = coordinates.geodetic_to_ecef(math.pi / 2.0, 0.0, 0.0)
    assert math.isclose(r_pole[0], 0.0, abs_tol=1e-4)
    assert math.isclose(r_pole[1], 0.0, abs_tol=1e-4)
    assert math.isclose(r_pole[2], constants.EARTH_RADIUS_POLAR, abs_tol=1e-4)


def test_geodetic_ecef_roundtrip_bowring():
    """Verify sub-millimeter round-trip accuracy of Bowring's method across diverse latitudes."""
    test_locations = [
        (math.radians(17.3850), math.radians(78.4867), 505.0),    # Hyderabad GS
        (math.radians(0.0), math.radians(0.0), 500000.0),          # Equatorial LEO 500km
        (math.radians(65.0), math.radians(-120.0), 1000.0),        # High Latitude
        (math.radians(-45.0), math.radians(170.0), 300000.0),      # Southern Hemisphere LEO
    ]

    for lat_in, lon_in, alt_in in test_locations:
        r_ecef = coordinates.geodetic_to_ecef(lat_in, lon_in, alt_in)
        lat_out, lon_out, alt_out = coordinates.ecef_to_geodetic(r_ecef)

        assert math.isclose(lat_in, lat_out, abs_tol=1e-9), f"Latitude mismatch: {lat_in} vs {lat_out}"
        assert math.isclose(lon_in, lon_out, abs_tol=1e-9), f"Longitude mismatch: {lon_in} vs {lon_out}"
        assert math.isclose(alt_in, alt_out, abs_tol=1e-3), f"Altitude mismatch: {alt_in} vs {alt_out}"


def test_quaternion_normalization_and_identity():
    """Verify quaternion normalization and identity operations."""
    q_unnorm = np.array([2.0, 0.0, 0.0, 0.0])
    q_norm = coordinates.quaternion_normalize(q_unnorm)
    assert math.isclose(np.linalg.norm(q_norm), 1.0, rel_tol=1e-12)
    assert np.allclose(q_norm, [1.0, 0.0, 0.0, 0.0])

    # Zero quaternion fallback
    q_zero = np.array([0.0, 0.0, 0.0, 0.0])
    q_safe = coordinates.quaternion_normalize(q_zero)
    assert math.isclose(np.linalg.norm(q_safe), 1.0, rel_tol=1e-12)


def test_quaternion_hamilton_product():
    """Verify quaternion multiplication properties: q * q* = [1, 0, 0, 0]."""
    q1 = coordinates.quaternion_normalize(np.array([1.0, 0.5, -0.2, 0.3]))
    q1_conj = coordinates.quaternion_conjugate(q1)

    q_identity = coordinates.quaternion_multiply(q1, q1_conj)
    assert np.allclose(q_identity, [1.0, 0.0, 0.0, 0.0], atol=1e-10)


def test_quaternion_dcm_roundtrip():
    """Verify quaternion to DCM and Shepperd's DCM to quaternion roundtrip."""
    test_angles = [
        (0.1, 0.2, 0.3),
        (-0.5, 0.4, 1.2),
        (0.0, 0.0, 0.0),
        (math.pi / 4, -math.pi / 6, math.pi / 3),
    ]

    for roll, pitch, yaw in test_angles:
        q_orig = coordinates.euler_to_quaternion(roll, pitch, yaw)
        dcm = coordinates.quaternion_to_dcm(q_orig)

        # Check DCM orthogonality: R * R^T = I
        identity = np.dot(dcm, dcm.T)
        assert np.allclose(identity, np.eye(3), atol=1e-10)

        # Check determinant is +1 (proper rotation)
        assert math.isclose(np.linalg.det(dcm), 1.0, rel_tol=1e-10)

        # Convert DCM back to quaternion
        q_recovered = coordinates.dcm_to_quaternion(dcm)

        # Quaternions q and -q represent identical rotations
        dot_product = abs(np.dot(q_orig, q_recovered))
        assert math.isclose(dot_product, 1.0, abs_tol=1e-8)


def test_euler_quaternion_roundtrip():
    """Verify 3-2-1 Euler angle to quaternion and back."""
    test_cases = [
        (math.radians(15.0), math.radians(-30.0), math.radians(45.0)),
        (math.radians(-10.0), math.radians(20.0), math.radians(-50.0)),
        (0.0, 0.0, 0.0),
    ]

    for r_in, p_in, y_in in test_cases:
        q = coordinates.euler_to_quaternion(r_in, p_in, y_in)
        r_out, p_out, y_out = coordinates.quaternion_to_euler(q)

        assert math.isclose(r_in, r_out, abs_tol=1e-8)
        assert math.isclose(p_in, p_out, abs_tol=1e-8)
        assert math.isclose(y_in, y_out, abs_tol=1e-8)


def test_quaternion_rotate_vector_norm():
    """Verify that vector rotation preserves Euclidean norm."""
    v = np.array([12.5, -45.2, 88.1])
    q = coordinates.euler_to_quaternion(0.5, -0.3, 1.1)

    v_rot = coordinates.quaternion_rotate_vector(q, v)
    assert math.isclose(np.linalg.norm(v), np.linalg.norm(v_rot), rel_tol=1e-12)


def test_construct_lvlh_dcm():
    """Verify that the LVLH frame is properly formed, orthogonal, and right-handed."""
    r_eci = np.array([6878137.0, 0.0, 0.0])  # Along X_eci
    v_eci = np.array([0.0, 7612.0, 0.0])     # Along Y_eci

    dcm = coordinates.construct_lvlh_dcm(r_eci, v_eci)

    # Orthogonality
    assert np.allclose(np.dot(dcm, dcm.T), np.eye(3), atol=1e-10)
    assert math.isclose(np.linalg.det(dcm), 1.0, rel_tol=1e-10)

    # Z_lvlh must point toward Earth center (-r_eci)
    z_lvlh = dcm[2, :]
    expected_z = -r_eci / np.linalg.norm(r_eci)
    assert np.allclose(z_lvlh, expected_z, atol=1e-10)

    # Y_lvlh must be negative orbital angular momentum
    h = np.cross(r_eci, v_eci)
    expected_y = -h / np.linalg.norm(h)
    assert np.allclose(dcm[1, :], expected_y, atol=1e-10)
