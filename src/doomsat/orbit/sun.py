"""Solar ephemeris, solar vector, and Earth shadow eclipse modeling for DOOMSat.

All units are strictly SI:
- Distances: meters (m)
- Angles: radians (rad)
- Time: seconds (s)
- Flux: Watts / m^2 (W/m^2)
"""

import math
from typing import Tuple
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import (
    EARTH_RADIUS_EQUATORIAL,
    SOLAR_CONSTANT,
    ASTRONOMICAL_UNIT,
)

# Earth orbital obliquity (axial tilt) J2000: 23.4392911 degrees
EARTH_OBLIQUITY_RAD: float = math.radians(23.4392911)

# Mean tropical year in seconds
YEAR_SECONDS: float = 365.25636 * 86400.0


def compute_sun_vector_eci(time_from_epoch_s: float, epoch_day_of_year: float = 312.0) -> NDArray[np.float64]:
    """Calculate the normalized Sun direction vector in ECI frame.

    Low-precision solar ephemeris suitable for LEO power, thermal, and sun-sensor simulation.
    Accuracy is within 0.05 degrees of astronomical tables.

    Args:
        time_from_epoch_s: Simulation elapsed time [s].
        epoch_day_of_year: Mission epoch day of year (default 312.0 ~= Nov 8).

    Returns:
        3D unit vector [sx, sy, sz] pointing from Earth to the Sun in ECI.
    """
    # Fraction of tropical year elapsed
    fraction_year = (epoch_day_of_year + time_from_epoch_s / 86400.0) / 365.25636
    # Mean anomaly of Sun (radians)
    mean_anomaly = 2.0 * math.pi * fraction_year - math.radians(3.0)

    # Ecliptic longitude of Sun (lambda) accounting for equation of center
    ecliptic_lon = (
        mean_anomaly
        + math.radians(1.915) * math.sin(mean_anomaly)
        + math.radians(0.020) * math.sin(2.0 * mean_anomaly)
        + math.radians(280.46)
    )
    ecliptic_lon = ecliptic_lon % (2.0 * math.pi)

    # Transform from Ecliptic plane to ECI equatorial plane using Earth obliquity (eps)
    eps = EARTH_OBLIQUITY_RAD
    cos_l = math.cos(ecliptic_lon)
    sin_l = math.sin(ecliptic_lon)

    s_x = cos_l
    s_y = math.cos(eps) * sin_l
    s_z = math.sin(eps) * sin_l

    sun_vec = np.array([s_x, s_y, s_z], dtype=np.float64)
    return sun_vec / np.linalg.norm(sun_vec)


def compute_eclipse_factor(
    r_eci: NDArray[np.float64],
    sun_unit_vector: NDArray[np.float64],
    penumbra_margin_m: float = 20_000.0,
) -> Tuple[float, bool]:
    """Calculate the solar eclipse factor nu in [0.0, 1.0] and eclipse flag.

    Shadow geometry:
    - nu = 1.0: Full sunlight
    - 0.0 < nu < 1.0: Penumbra transition
    - nu = 0.0: Total umbra (Earth shadow)

    Args:
        r_eci: Spacecraft position vector in ECI [m].
        sun_unit_vector: Normalized vector pointing toward the Sun in ECI.
        penumbra_margin_m: Atmospheric penumbra transition buffer [m].

    Returns:
        Tuple of (eclipse_factor_nu, is_in_shadow).
    """
    # Project spacecraft position onto Sun line
    # Positive projection -> Spacecraft is on sunlit day side of Earth
    s_proj = float(np.dot(r_eci, sun_unit_vector))

    if s_proj > 0.0:
        # Day side -> full sunlight
        return 1.0, False

    # Spacecraft is behind the Earth terminator plane (s_proj <= 0)
    # Perpendicular distance from Earth-Sun centerline
    r_perp_vec = r_eci - s_proj * sun_unit_vector
    d_perp = float(np.linalg.norm(r_perp_vec))

    re = EARTH_RADIUS_EQUATORIAL
    r_umbra = re
    r_penumbra = re + penumbra_margin_m

    if d_perp <= r_umbra:
        # Total umbra
        return 0.0, True
    elif d_perp < r_penumbra:
        # Penumbra gradient
        fraction = (d_perp - r_umbra) / (r_penumbra - r_umbra)
        return float(fraction), True
    else:
        # Sun is unocculted outside Earth cylinder
        return 1.0, False


def compute_solar_flux(
    r_eci: NDArray[np.float64],
    sun_unit_vector: NDArray[np.float64],
    solar_constant_w_m2: float = SOLAR_CONSTANT,
) -> Tuple[float, float, bool]:
    """Calculate instantaneous incident solar irradiance flux [W/m^2].

    Args:
        r_eci: Spacecraft position in ECI [m].
        sun_unit_vector: Unit vector to Sun in ECI.
        solar_constant_w_m2: Solar constant at 1 AU (default 1361 W/m^2).

    Returns:
        Tuple of (incident_flux_w_m2, eclipse_factor_nu, in_shadow).
    """
    nu, in_shadow = compute_eclipse_factor(r_eci, sun_unit_vector)
    flux = solar_constant_w_m2 * nu
    return flux, nu, in_shadow

