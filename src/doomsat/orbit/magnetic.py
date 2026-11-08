"""Earth geomagnetic field model (tilted eccentric dipole) for DOOMSat ADCS simulation.

Produces the geomagnetic field vector B_ECI(r, t) in Tesla (T).
All units are strictly SI:
- Distance: meters (m)
- Magnetic flux density: Tesla (T)
- Angles: radians (rad)
"""

import math
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import (
    EARTH_RADIUS_EQUATORIAL,
    EARTH_OMEGA_ROTATION,
)

# Reference Equatorial Surface Magnetic Field Strength [Tesla]
# B_0 ~= 3.12e-5 T (0.312 Gauss)
GEOMAGNETIC_B0_TESLA: float = 3.12e-5

# Magnetic Dipole Axis Tilt relative to Geographic North Pole
# Geomagnetic North pole currently located at ~80.5 deg N, 72.5 deg W
# Tilt theta_m ~= 11.5 degrees = 0.2007 rad
GEOMAGNETIC_TILT_RAD: float = math.radians(11.5)

# Longitude of geomagnetic pole (west)
GEOMAGNETIC_POLE_LON_RAD: float = math.radians(-72.5)


def get_magnetic_dipole_axis_eci(gmst_rad: float) -> NDArray[np.float64]:
    """Calculate the Earth's magnetic dipole unit vector m_hat in ECI at given GMST.

    Args:
        gmst_rad: Greenwich Mean Sidereal Time [rad].

    Returns:
        3D unit vector [mx, my, mz] in ECI pointing along the magnetic dipole axis.
    """
    # Dipole axis in ECEF frame
    tilt = GEOMAGNETIC_TILT_RAD
    lon = GEOMAGNETIC_POLE_LON_RAD

    m_ecef = np.array([
        math.sin(tilt) * math.cos(lon),
        math.sin(tilt) * math.sin(lon),
        math.cos(tilt),
    ], dtype=np.float64)

    # Rotate ECEF to ECI by GMST: R_z(-gmst)
    cos_g = math.cos(gmst_rad)
    sin_g = math.sin(gmst_rad)

    m_eci_x = cos_g * m_ecef[0] - sin_g * m_ecef[1]
    m_eci_y = sin_g * m_ecef[0] + cos_g * m_ecef[1]
    m_eci_z = m_ecef[2]

    m_eci = np.array([m_eci_x, m_eci_y, m_eci_z], dtype=np.float64)
    return m_eci / np.linalg.norm(m_eci)


def compute_geomagnetic_field_eci(
    r_eci: NDArray[np.float64],
    gmst_rad: float,
    b0_tesla: float = GEOMAGNETIC_B0_TESLA,
    re_m: float = EARTH_RADIUS_EQUATORIAL,
) -> NDArray[np.float64]:
    """Calculate the geomagnetic field vector B_ECI at spacecraft position [Tesla].

    Tilted Earth dipole equation:
        B(r) = (B_0 * R_E^3 / r^3) * [ 3 * (m_hat . r_hat) * r_hat - m_hat ]

    Args:
        r_eci: Spacecraft position vector in ECI [m].
        gmst_rad: Greenwich Mean Sidereal Time [rad].
        b0_tesla: Surface equatorial field strength (default 3.12e-5 T).
        re_m: Earth equatorial radius (default 6,378,137 m).

    Returns:
        3D magnetic field vector [Bx, By, Bz] in Tesla (T).
    """
    r_norm = float(np.linalg.norm(r_eci))
    if r_norm < 1000.0:
        raise ValueError(f"Position vector norm {r_norm} m is too small.")

    r_hat = r_eci / r_norm
    m_hat = get_magnetic_dipole_axis_eci(gmst_rad)

    m_dot_r = float(np.dot(m_hat, r_hat))

    # Dipole field scaling factor: B_0 * (R_E / r)^3
    scale = b0_tesla * ((re_m / r_norm) ** 3)

    b_vec = scale * (3.0 * m_dot_r * r_hat - m_hat)
    return b_vec
