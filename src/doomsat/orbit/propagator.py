"""Numerical and analytical orbital propagators for DOOMSat.

Includes:
- Numerical Cowell's method (4th-order Runge-Kutta) integrating Earth point-mass gravity
  and the second zonal harmonic (J2) oblateness perturbation.
- SGP4 analytical propagator adapter for Two-Line Element (TLE) ephemeris.
- Orbital element conversions (Keplerian <-> Cartesian).
"""

import math
from typing import Tuple, Optional
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import (
    EARTH_MU,
    EARTH_RADIUS_EQUATORIAL,
    EARTH_J2,
)


def keplerian_to_cartesian(
    a_m: float,
    e: float,
    inc_rad: float,
    raan_rad: float,
    argp_rad: float,
    nu_rad: float,
    mu: float = EARTH_MU,
) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Convert classical Keplerian orbital elements to Cartesian ECI position and velocity.

    Args:
        a_m: Semi-major axis [m].
        e: Eccentricity (0 <= e < 1).
        inc_rad: Inclination [rad].
        raan_rad: Right Ascension of Ascending Node (RAAN) [rad].
        argp_rad: Argument of perigee [rad].
        nu_rad: True anomaly [rad].
        mu: Gravitational parameter [m^3/s^2].

    Returns:
        Tuple of (r_eci [m], v_eci [m/s]).
    """
    if a_m <= 0.0:
        raise ValueError(f"Semi-major axis must be positive: {a_m} m")
    if not (0.0 <= e < 1.0):
        raise ValueError(f"Eccentricity must be in [0, 1): {e}")

    # Semi-latus rectum
    p = a_m * (1.0 - e**2)

    # Position in orbital perifocal plane (PQW frame)
    r_orbit = p / (1.0 + e * math.cos(nu_rad))
    r_pqw = np.array([
        r_orbit * math.cos(nu_rad),
        r_orbit * math.sin(nu_rad),
        0.0,
    ], dtype=np.float64)

    # Velocity in orbital perifocal plane
    v_coef = math.sqrt(mu / p)
    v_pqw = np.array([
        -v_coef * math.sin(nu_rad),
        v_coef * (e + math.cos(nu_rad)),
        0.0,
    ], dtype=np.float64)

    # Rotation matrix from perifocal (PQW) to ECI: R_z(-RAAN) * R_x(-inc) * R_z(-argp)
    cos_o, sin_o = math.cos(raan_rad), math.sin(raan_rad)
    cos_i, sin_i = math.cos(inc_rad), math.sin(inc_rad)
    cos_w, sin_w = math.cos(argp_rad), math.sin(argp_rad)

    # Perifocal to ECI direction cosine matrix
    r11 = cos_o * cos_w - sin_o * sin_w * cos_i
    r12 = -cos_o * sin_w - sin_o * cos_w * cos_i
    r13 = sin_o * sin_i

    r21 = sin_o * cos_w + cos_o * sin_w * cos_i
    r22 = -sin_o * sin_w + cos_o * cos_w * cos_i
    r23 = -cos_o * sin_i

    r31 = sin_w * sin_i
    r32 = cos_w * sin_i
    r33 = cos_i

    dcm_pqw_to_eci = np.array([
        [r11, r12, r13],
        [r21, r22, r23],
        [r31, r32, r33]
    ], dtype=np.float64)

    r_eci = np.dot(dcm_pqw_to_eci, r_pqw)
    v_eci = np.dot(dcm_pqw_to_eci, v_pqw)

    return r_eci, v_eci


def cartesian_to_keplerian(
    r_eci: NDArray[np.float64],
    v_eci: NDArray[np.float64],
    mu: float = EARTH_MU,
) -> Tuple[float, float, float, float, float, float]:
    """Convert Cartesian ECI state vectors to classical Keplerian orbital elements.

    Returns:
        Tuple of (a_m, e, inc_rad, raan_rad, argp_rad, nu_rad).
    """
    r_norm = float(np.linalg.norm(r_eci))
    v_norm = float(np.linalg.norm(v_eci))

    # Angular momentum vector
    h_vec = np.cross(r_eci, v_eci)
    h_norm = float(np.linalg.norm(h_vec))

    # Node line vector
    n_vec = np.cross([0.0, 0.0, 1.0], h_vec)
    n_norm = float(np.linalg.norm(n_vec))

    # Eccentricity vector
    e_vec = (1.0 / mu) * ((v_norm**2 - mu / r_norm) * r_eci - np.dot(r_eci, v_eci) * v_eci)
    e = float(np.linalg.norm(e_vec))

    # Specific orbital energy and semi-major axis
    energy = (v_norm**2 / 2.0) - (mu / r_norm)
    a_m = -mu / (2.0 * energy) if abs(energy) > 1e-12 else float("inf")

    # Inclination
    inc_rad = math.acos(max(-1.0, min(1.0, h_vec[2] / h_norm)))

    # RAAN
    if n_norm > 1e-10:
        raan_rad = math.acos(max(-1.0, min(1.0, n_vec[0] / n_norm)))
        if n_vec[1] < 0.0:
            raan_rad = 2.0 * math.pi - raan_rad
    else:
        raan_rad = 0.0

    # Argument of perigee
    if n_norm > 1e-10 and e > 1e-6:
        argp_rad = math.acos(max(-1.0, min(1.0, np.dot(n_vec, e_vec) / (n_norm * e))))
        if e_vec[2] < 0.0:
            argp_rad = 2.0 * math.pi - argp_rad
    else:
        argp_rad = 0.0

    # True anomaly
    if e > 1e-6:
        nu_rad = math.acos(max(-1.0, min(1.0, np.dot(e_vec, r_eci) / (e * r_norm))))
        if np.dot(r_eci, v_eci) < 0.0:
            nu_rad = 2.0 * math.pi - nu_rad
    else:
        nu_rad = 0.0

    return a_m, e, inc_rad, raan_rad, argp_rad, nu_rad


def compute_j2_acceleration(r_eci: NDArray[np.float64]) -> NDArray[np.float64]:
    """Calculate gravitational acceleration vector including point-mass gravity and J2 oblateness.

    Equations:
        a_two_body = - (mu / r^3) * r
        a_j2 = - (3 * J2 * mu * R_E^2) / (2 * r^5) * [ x*(1 - 5*z^2/r^2), y*(1 - 5*z^2/r^2), z*(3 - 5*z^2/r^2) ]

    Args:
        r_eci: Spacecraft position vector in ECI [m].

    Returns:
        Acceleration vector [ax, ay, az] in ECI [m/s^2].
    """
    x, y, z = r_eci[0], r_eci[1], r_eci[2]
    r2 = x**2 + y**2 + z**2
    r = math.sqrt(r2)

    if r < 1000.0:
        raise ValueError(f"Position vector norm {r} m is inside the core of the Earth.")

    mu = EARTH_MU
    re = EARTH_RADIUS_EQUATORIAL
    j2 = EARTH_J2

    # Two-body gravitational acceleration
    a_two_body = - (mu / (r**3)) * r_eci

    # J2 oblateness perturbation
    z2_r2 = (z**2) / r2
    factor = - (1.5 * j2 * mu * (re**2)) / (r**5)

    a_j2_x = factor * x * (1.0 - 5.0 * z2_r2)
    a_j2_y = factor * y * (1.0 - 5.0 * z2_r2)
    a_j2_z = factor * z * (3.0 - 5.0 * z2_r2)

    a_j2 = np.array([a_j2_x, a_j2_y, a_j2_z], dtype=np.float64)

    return a_two_body + a_j2


class NumericalOrbitPropagator:
    """Numerical Cowell's method orbit propagator using explicit 4th-order Runge-Kutta (RK4)."""

    def __init__(
        self,
        initial_r_eci: NDArray[np.float64],
        initial_v_eci: NDArray[np.float64],
        epoch_time_s: float = 0.0,
    ):
        self.r_eci: NDArray[np.float64] = np.array(initial_r_eci, dtype=np.float64)
        self.v_eci: NDArray[np.float64] = np.array(initial_v_eci, dtype=np.float64)
        self.time_s: float = float(epoch_time_s)

    def step(self, dt_s: float) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Advance orbital state by time step dt_s using RK4 integration.

        State vector: y = [r, v]^T
        Derivative: y_dot = [v, a(r)]^T
        """
        r0 = self.r_eci
        v0 = self.v_eci

        # k1
        k1_v = v0
        k1_a = compute_j2_acceleration(r0)

        # k2
        r_mid1 = r0 + 0.5 * dt_s * k1_v
        v_mid1 = v0 + 0.5 * dt_s * k1_a
        k2_v = v_mid1
        k2_a = compute_j2_acceleration(r_mid1)

        # k3
        r_mid2 = r0 + 0.5 * dt_s * k2_v
        v_mid2 = v0 + 0.5 * dt_s * k2_a
        k3_v = v_mid2
        k3_a = compute_j2_acceleration(r_mid2)

        # k4
        r_end = r0 + dt_s * k3_v
        v_end = v0 + dt_s * k3_a
        k4_v = v_end
        k4_a = compute_j2_acceleration(r_end)

        # Update
        self.r_eci = r0 + (dt_s / 6.0) * (k1_v + 2.0 * k2_v + 2.0 * k3_v + k4_v)
        self.v_eci = v0 + (dt_s / 6.0) * (k1_a + 2.0 * k2_a + 2.0 * k3_a + k4_a)
        self.time_s += dt_s

        return self.r_eci, self.v_eci


class SGP4Propagator:
    """Analytical SGP4 orbit propagator adapter using standard Two-Line Element (TLE) sets."""

    def __init__(self, tle_line1: str, tle_line2: str):
        from sgp4.api import Satrec, WGS72
        self.satrec = Satrec.twoline2rv(tle_line1, tle_line2, WGS72)

    def propagate_jd(self, jd: float, fr: float = 0.0) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Propagate orbit to Julian Date and return position [m] and velocity [m/s] in TEME/ECI."""
        error_code, r_km, v_km_s = self.satrec.sgp4(jd, fr)
        if error_code != 0:
            raise RuntimeError(f"SGP4 propagation error code: {error_code}")

        r_m = np.array(r_km, dtype=np.float64) * 1000.0
        v_mps = np.array(v_km_s, dtype=np.float64) * 1000.0
        return r_m, v_mps

