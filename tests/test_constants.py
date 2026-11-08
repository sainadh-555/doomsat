"""Unit tests for physical, geodetic, and astrodynamical constants."""

import math
import pytest
from doomsat.core import constants


def test_fundamental_physical_constants():
    """Verify fundamental constants match CODATA 2018 definitions."""
    assert constants.SPEED_OF_LIGHT == 299_792_458.0
    assert constants.BOLTZMANN_CONSTANT == 1.380_649e-23
    assert constants.STANDARD_GRAVITY == 9.80665
    assert math.isclose(constants.STEFAN_BOLTZMANN, 5.670374e-8, rel_tol=1e-5)
    assert constants.GRAVITATIONAL_CONSTANT > 6.67e-11


def test_wgs84_earth_geodetic_consistency():
    """Verify WGS-84 Earth ellipsoid mathematical relations."""
    a = constants.EARTH_RADIUS_EQUATORIAL
    f = constants.EARTH_FLATTENING
    b = constants.EARTH_RADIUS_POLAR
    e2 = constants.EARTH_ECCENTRICITY_SQ

    # b = a * (1 - f)
    assert math.isclose(b, a * (1.0 - f), rel_tol=1e-12)
    # e^2 = 2f - f^2
    assert math.isclose(e2, 2.0 * f - f**2, rel_tol=1e-12)
    # Equatorial radius should exceed polar radius by ~21.385 km
    assert 21300.0 < (a - b) < 21400.0


def test_astrodynamical_constants():
    """Verify Earth gravitational parameter, J2, and solar constant."""
    assert math.isclose(constants.EARTH_MU, 3.986004418e14, rel_tol=1e-8)
    assert math.isclose(constants.EARTH_J2, 1.08262668e-3, rel_tol=1e-7)
    assert constants.SOLAR_CONSTANT == 1361.0
    assert math.isclose(constants.EARTH_OMEGA_ROTATION, 7.292115e-5, rel_tol=1e-6)


def test_cubesat_1u_constraints():
    """Verify standard 1U CubeSat physical specifications and limits."""
    assert constants.CUBESAT_1U_MAX_MASS_KG == 1.330
    assert constants.CUBESAT_1U_X_LEN_M == 0.100
    assert constants.CUBESAT_1U_Y_LEN_M == 0.100
    assert constants.CUBESAT_1U_Z_LEN_M == 0.1135
    assert math.isclose(constants.CUBESAT_1U_FACE_AREA_M2, 0.010, rel_tol=1e-9)


def test_rf_frequency_constants():
    """Verify standard baseline frequencies for space and HIL benchtop."""
    assert constants.RF_BAND_ITU_AMATEUR_UHF_HZ == 436_500_000.0
    assert constants.RF_BAND_WPC_INDIA_DELICENSED_HZ == 865_500_000.0
    assert constants.BOLTZMANN_DBM_PER_HZ == -174.0
