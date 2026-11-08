"""Orbital mechanics, SGP4/J2 propagators, solar ephemeris, and ground station visibility package."""

from doomsat.orbit.propagator import (
    keplerian_to_cartesian,
    cartesian_to_keplerian,
    compute_j2_acceleration,
    NumericalOrbitPropagator,
    SGP4Propagator,
)
from doomsat.orbit.sun import (
    compute_sun_vector_eci,
    compute_eclipse_factor,
    compute_solar_flux,
    EARTH_OBLIQUITY_RAD,
)
from doomsat.orbit.magnetic import (
    compute_geomagnetic_field_eci,
    get_magnetic_dipole_axis_eci,
    GEOMAGNETIC_B0_TESLA,
)
from doomsat.orbit.ground_pass import (
    GroundStation,
    LookAngles,
    GroundPassEvent,
)

__all__ = [
    "keplerian_to_cartesian",
    "cartesian_to_keplerian",
    "compute_j2_acceleration",
    "NumericalOrbitPropagator",
    "SGP4Propagator",
    "compute_sun_vector_eci",
    "compute_eclipse_factor",
    "compute_solar_flux",
    "EARTH_OBLIQUITY_RAD",
    "compute_geomagnetic_field_eci",
    "get_magnetic_dipole_axis_eci",
    "GEOMAGNETIC_B0_TESLA",
    "GroundStation",
    "LookAngles",
    "GroundPassEvent",
]
