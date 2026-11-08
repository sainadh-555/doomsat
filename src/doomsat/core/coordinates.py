"""Coordinate frames, transformations, and attitude kinematic algebra for DOOMSat.

All functions use strict SI units:
- Distances in meters (m)
- Angles in radians (rad)
- Velocities in meters/second (m/s)

Quaternion Convention:
- Scalar-first quaternion: q = [q_w, q_x, q_y, q_z]^T
- Where q_w = cos(theta / 2) and [q_x, q_y, q_z]^T = e * sin(theta / 2)
- Unit quaternion condition: ||q|| = 1.0
"""

import math
from typing import Tuple
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import (
    EARTH_RADIUS_EQUATORIAL,
    EARTH_FLATTENING,
    EARTH_ECCENTRICITY_SQ,
    EARTH_RADIUS_POLAR,
)


# ==============================================================================
# Celestial & Terrestrial Frame Transformations (ECI <--> ECEF <--> Geodetic)
# ==============================================================================

def eci_to_ecef(r_eci: NDArray[np.float64], gmst_rad: float) -> NDArray[np.float64]:
    """Transform Cartesian coordinates from Earth-Centered Inertial (ECI) to Earth-Centered Earth-Fixed (ECEF).

    Equation:
        r_ECEF = R_z(theta_GMST) * r_ECI

    Args:
        r_eci: 3D vector [x, y, z] in ECI frame [m].
        gmst_rad: Greenwich Mean Sidereal Time [rad].

    Returns:
        3D vector [x, y, z] in ECEF frame [m].
    """
    cos_g = math.cos(gmst_rad)
    sin_g = math.sin(gmst_rad)
    r_x = cos_g * r_eci[0] + sin_g * r_eci[1]
    r_y = -sin_g * r_eci[0] + cos_g * r_eci[1]
    r_z = r_eci[2]
    return np.array([r_x, r_y, r_z], dtype=np.float64)


def ecef_to_eci(r_ecef: NDArray[np.float64], gmst_rad: float) -> NDArray[np.float64]:
    """Transform Cartesian coordinates from ECEF to ECI.

    Equation:
        r_ECI = R_z(-theta_GMST) * r_ECEF

    Args:
        r_ecef: 3D vector [x, y, z] in ECEF frame [m].
        gmst_rad: Greenwich Mean Sidereal Time [rad].

    Returns:
        3D vector [x, y, z] in ECI frame [m].
    """
    cos_g = math.cos(gmst_rad)
    sin_g = math.sin(gmst_rad)
    r_x = cos_g * r_ecef[0] - sin_g * r_ecef[1]
    r_y = sin_g * r_ecef[0] + cos_g * r_ecef[1]
    r_z = r_ecef[2]
    return np.array([r_x, r_y, r_z], dtype=np.float64)


def geodetic_to_ecef(lat_rad: float, lon_rad: float, alt_m: float) -> NDArray[np.float64]:
    """Convert WGS-84 geodetic coordinates to Cartesian ECEF.

    Equations:
        N = a / sqrt(1 - e^2 * sin^2(lat))
        X = (N + h) * cos(lat) * cos(lon)
        Y = (N + h) * cos(lat) * sin(lon)
        Z = (N * (1 - e^2) + h) * sin(lat)

    Args:
        lat_rad: Geodetic latitude [rad] (-pi/2 to +pi/2).
        lon_rad: Geodetic longitude [rad] (-pi to +pi).
        alt_m: Height above WGS-84 reference ellipsoid [m].

    Returns:
        3D vector [x, y, z] in ECEF frame [m].
    """
    a = EARTH_RADIUS_EQUATORIAL
    e2 = EARTH_ECCENTRICITY_SQ

    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)

    # Prime vertical radius of curvature
    n = a / math.sqrt(1.0 - e2 * sin_lat**2)

    x = (n + alt_m) * cos_lat * cos_lon
    y = (n + alt_m) * cos_lat * sin_lon
    z = (n * (1.0 - e2) + alt_m) * sin_lat

    return np.array([x, y, z], dtype=np.float64)


def ecef_to_geodetic(r_ecef: NDArray[np.float64]) -> Tuple[float, float, float]:
    """Convert Cartesian ECEF to WGS-84 geodetic coordinates using Bowring's method.

    Bowring's algorithm provides sub-millimeter precision for altitudes up to LEO.

    Args:
        r_ecef: 3D vector [x, y, z] in ECEF frame [m].

    Returns:
        Tuple of (latitude [rad], longitude [rad], altitude [m]).
    """
    x, y, z = float(r_ecef[0]), float(r_ecef[1]), float(r_ecef[2])
    a = EARTH_RADIUS_EQUATORIAL
    b = EARTH_RADIUS_POLAR
    e2 = EARTH_ECCENTRICITY_SQ
    ep2 = (a**2 - b**2) / (b**2)

    p = math.sqrt(x**2 + y**2)
    if p < 1e-12:  # Pole singularity
        lat = math.copysign(math.pi / 2.0, z)
        lon = 0.0
        alt = abs(z) - b
        return lat, lon, alt

    lon = math.atan2(y, x)

    # Parametric latitude (theta)
    theta = math.atan2(z * a, p * b)

    # Bowring's formula for geodetic latitude
    sin_th = math.sin(theta)
    cos_th = math.cos(theta)
    lat = math.atan2(z + ep2 * b * (sin_th**3), p - e2 * a * (cos_th**3))

    sin_lat = math.sin(lat)
    n = a / math.sqrt(1.0 - e2 * (sin_lat**2))
    alt = p / math.cos(lat) - n

    return lat, lon, alt


# ==============================================================================
# Attitude Representation: Quaternions, DCM, & Euler Angles
# ==============================================================================

def quaternion_normalize(q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Normalize a quaternion to maintain unit magnitude.

    Args:
        q: 4-element array [q_w, q_x, q_y, q_z].

    Returns:
        Normalized unit quaternion.
    """
    norm = np.linalg.norm(q)
    if norm < 1e-14:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)
    return q / norm


def quaternion_multiply(q1: NDArray[np.float64], q2: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute quaternion Hamilton product: q = q1 * q2.

    Represents consecutive rotations where q2 is applied first, then q1.

    Args:
        q1: 4-element quaternion [w1, x1, y1, z1].
        q2: 4-element quaternion [w2, x2, y2, z2].

    Returns:
        Combined quaternion [w, x, y, z].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z], dtype=np.float64)


def quaternion_conjugate(q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Compute quaternion conjugate: q* = [q_w, -q_x, -q_y, -q_z].

    For a unit quaternion, the conjugate equals the inverse rotation.
    """
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=np.float64)


def quaternion_to_dcm(q: NDArray[np.float64]) -> NDArray[np.float64]:
    """Convert unit quaternion [q_w, q_x, q_y, q_z] to 3x3 Direction Cosine Matrix (DCM).

    Transforms vectors from the reference frame to the rotated frame:
        v_rotated = A * v_reference

    Args:
        q: Unit quaternion [q_w, q_x, q_y, q_z].

    Returns:
        3x3 orthogonal rotation matrix (DCM).
    """
    q_norm = quaternion_normalize(q)
    w, x, y, z = q_norm

    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z

    dcm = np.array([
        [1.0 - 2.0 * (yy + zz),       2.0 * (xy + wz),       2.0 * (xz - wy)],
        [      2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz),       2.0 * (yz + wx)],
        [      2.0 * (xz + wy),       2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)]
    ], dtype=np.float64)

    return dcm


def dcm_to_quaternion(dcm: NDArray[np.float64]) -> NDArray[np.float64]:
    """Convert 3x3 DCM to unit quaternion using Shepperd's algorithm.

    Shepperd's method avoids numerical singularities by selecting the largest diagonal element.

    Args:
        dcm: 3x3 orthogonal rotation matrix.

    Returns:
        Unit quaternion [q_w, q_x, q_y, q_z].
    """
    trace = dcm[0, 0] + dcm[1, 1] + dcm[2, 2]

    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (dcm[1, 2] - dcm[2, 1]) * s
        y = (dcm[2, 0] - dcm[0, 2]) * s
        z = (dcm[0, 1] - dcm[1, 0]) * s
    elif (dcm[0, 0] > dcm[1, 1]) and (dcm[0, 0] > dcm[2, 2]):
        s = 2.0 * math.sqrt(1.0 + dcm[0, 0] - dcm[1, 1] - dcm[2, 2])
        w = (dcm[1, 2] - dcm[2, 1]) / s
        x = 0.25 * s
        y = (dcm[0, 1] + dcm[1, 0]) / s
        z = (dcm[0, 2] + dcm[2, 0]) / s
    elif dcm[1, 1] > dcm[2, 2]:
        s = 2.0 * math.sqrt(1.0 + dcm[1, 1] - dcm[0, 0] - dcm[2, 2])
        w = (dcm[2, 0] - dcm[0, 2]) / s
        x = (dcm[0, 1] + dcm[1, 0]) / s
        y = 0.25 * s
        z = (dcm[1, 2] + dcm[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + dcm[2, 2] - dcm[0, 0] - dcm[1, 1])
        w = (dcm[0, 1] - dcm[1, 0]) / s
        x = (dcm[0, 2] + dcm[2, 0]) / s
        y = (dcm[1, 2] + dcm[2, 1]) / s
        z = 0.25 * s

    q = np.array([w, x, y, z], dtype=np.float64)
    return quaternion_normalize(q)


def euler_to_quaternion(roll_rad: float, pitch_rad: float, yaw_rad: float) -> NDArray[np.float64]:
    """Convert 3-2-1 Euler angles (yaw-psi, pitch-theta, roll-phi) to unit quaternion.

    Sequence:
        Rotation 1: Yaw (psi) about Z-axis
        Rotation 2: Pitch (theta) about intermediate Y-axis
        Rotation 3: Roll (phi) about body X-axis

    Args:
        roll_rad: Roll angle (phi) [rad].
        pitch_rad: Pitch angle (theta) [rad].
        yaw_rad: Yaw angle (psi) [rad].

    Returns:
        Unit quaternion [q_w, q_x, q_y, q_z].
    """
    cr = math.cos(roll_rad * 0.5)
    sr = math.sin(roll_rad * 0.5)
    cp = math.cos(pitch_rad * 0.5)
    sp = math.sin(pitch_rad * 0.5)
    cy = math.cos(yaw_rad * 0.5)
    sy = math.sin(yaw_rad * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return quaternion_normalize(np.array([w, x, y, z], dtype=np.float64))


def quaternion_to_euler(q: NDArray[np.float64]) -> Tuple[float, float, float]:
    """Convert unit quaternion to 3-2-1 Euler angles (roll, pitch, yaw).

    Args:
        q: Unit quaternion [q_w, q_x, q_y, q_z].

    Returns:
        Tuple of (roll [rad], pitch [rad], yaw [rad]).
    """
    q_norm = quaternion_normalize(q)
    w, x, y, z = q_norm

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation) with gimbal lock protection
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_rotate_vector(q: NDArray[np.float64], v: NDArray[np.float64]) -> NDArray[np.float64]:
    """Rotate a 3D vector using unit quaternion: v_rot = A(q) * v.

    Args:
        q: Unit quaternion [q_w, q_x, q_y, q_z].
        v: 3D vector [x, y, z].

    Returns:
        Rotated 3D vector.
    """
    dcm = quaternion_to_dcm(q)
    return np.dot(dcm, v)


# ==============================================================================
# Orbital Local-Vertical Local-Horizontal (LVLH) Reference Frame
# ==============================================================================

def construct_lvlh_dcm(r_eci: NDArray[np.float64], v_eci: NDArray[np.float64]) -> NDArray[np.float64]:
    """Construct transformation matrix from ECI frame to Orbit LVLH frame.

    Definition:
        Z_lvlh = -r / ||r||           (Nadir: pointing toward Earth center)
        Y_lvlh = -(r x v) / ||r x v|| (Opposite to orbital angular momentum vector)
        X_lvlh = Y_lvlh x Z_lvlh      (Along flight direction for circular orbit)

    Args:
        r_eci: Spacecraft position vector in ECI [m].
        v_eci: Spacecraft velocity vector in ECI [m/s].

    Returns:
        3x3 Direction Cosine Matrix R_eci_to_lvlh where v_lvlh = R * v_eci.
    """
    r_norm = np.linalg.norm(r_eci)
    if r_norm < 1e-3:
        raise ValueError("Position vector norm is too small to define LVLH frame.")

    # Nadir vector
    z_lvlh = -r_eci / r_norm

    # Orbital angular momentum vector
    h_vec = np.cross(r_eci, v_eci)
    h_norm = np.linalg.norm(h_vec)
    if h_norm < 1e-6:
        raise ValueError("Angular momentum vector is degenerate (collinear position and velocity).")

    y_lvlh = -h_vec / h_norm
    x_lvlh = np.cross(y_lvlh, z_lvlh)
    x_lvlh = x_lvlh / np.linalg.norm(x_lvlh)

    # Rotation matrix where rows are the LVLH basis vectors projected onto ECI
    dcm_eci_to_lvlh = np.vstack([x_lvlh, y_lvlh, z_lvlh])
    return dcm_eci_to_lvlh
