"""Physical, astrodynamical, and CubeSat structural constants for DOOMSat.

All values are strictly in SI units:
- Distance: meters (m)
- Time: seconds (s)
- Mass: kilograms (kg)
- Temperature: Kelvin (K)
- Power: Watts (W)
- Energy: Joules (J)
- Voltage: Volts (V)
- Current: Amperes (A)
- Magnetic Field: Tesla (T)
- Angles: radians (rad), unless explicitly denoted otherwise.

Sources:
- CODATA 2018 Recommended Values of the Fundamental Physical Constants
- WGS-84 (World Geodetic System 1984, NIMA TR8350.2)
- IAU 2015 Resolution B3 (Nominal Astrodynamical Parameters)
"""

import math

# ==============================================================================
# Fundamental Physical Constants (CODATA 2018)
# ==============================================================================

SPEED_OF_LIGHT: float = 299_792_458.0
"""Speed of light in vacuum (c) [m/s]. Exact by definition."""

BOLTZMANN_CONSTANT: float = 1.380_649e-23
"""Boltzmann constant (k_B) [J/K]. Exact by definition."""

STEFAN_BOLTZMANN: float = 5.670_374_419e-8
"""Stefan-Boltzmann constant (sigma) [W / (m^2 * K^4)]."""

GRAVITATIONAL_CONSTANT: float = 6.674_30e-11
"""Universal gravitational constant (G) [m^3 / (kg * s^2)]. CODATA 2018."""

STANDARD_GRAVITY: float = 9.80665
"""Standard acceleration due to gravity on Earth surface (g_0) [m/s^2]."""

VACUUM_PERMITTIVITY: float = 8.854_187_8128e-12
"""Vacuum electric permittivity (epsilon_0) [F/m]."""

VACUUM_PERMEABILITY: float = 1.256_637_062_12e-6
"""Vacuum magnetic permeability (mu_0) [H/m or N/A^2]."""

# ==============================================================================
# Earth Astrodynamical & Geodetic Constants (WGS-84 / IAU 2015)
# ==============================================================================

EARTH_MU: float = 3.986_004_418e14
"""Earth standard gravitational parameter (G * M_earth) [m^3 / s^2]. WGS-84."""

EARTH_MASS: float = 5.972_19e24
"""Mass of Earth [kg]."""

EARTH_RADIUS_EQUATORIAL: float = 6_378_137.0
"""Earth semi-major axis (equatorial radius, a) [m]. WGS-84."""

EARTH_FLATTENING: float = 1.0 / 298.257_223_563
"""Earth ellipsoidal flattening factor (f). WGS-84."""

EARTH_RADIUS_POLAR: float = EARTH_RADIUS_EQUATORIAL * (1.0 - EARTH_FLATTENING)
"""Earth semi-minor axis (polar radius, b) [m]. WGS-84: 6356752.3142 m."""

EARTH_ECCENTRICITY_SQ: float = 2.0 * EARTH_FLATTENING - EARTH_FLATTENING**2
"""Square of Earth's first eccentricity (e^2 = (a^2 - b^2) / a^2)."""

EARTH_J2: float = 1.082_626_68e-3
"""Earth second zonal harmonic coefficient (oblateness perturbation, J2). WGS-84."""

EARTH_OMEGA_ROTATION: float = 7.292_115_0e-5
"""Earth mean angular rotation rate (omega_earth) [rad/s]. WGS-84."""

SOLAR_CONSTANT: float = 1361.0
"""Solar irradiance flux at 1 Astronomical Unit (AM0 spectrum, S_0) [W/m^2]."""

ASTRONOMICAL_UNIT: float = 149_597_870_700.0
"""1 Astronomical Unit (AU) [m]. IAU 2012 definition."""

# ==============================================================================
# 1U CubeSat Physical Limits & Mechanical Constraints
# ==============================================================================

CUBESAT_1U_MAX_MASS_KG: float = 1.330
"""Standard 1U CubeSat maximum allowable launch mass [kg]. CubeSat Spec Rev 14.1."""

CUBESAT_1U_X_LEN_M: float = 0.100
"""Nominal 1U structure width along X-axis [m]."""

CUBESAT_1U_Y_LEN_M: float = 0.100
"""Nominal 1U structure depth along Y-axis [m]."""

CUBESAT_1U_Z_LEN_M: float = 0.1135
"""Nominal 1U structure height along Z-axis (including dispenser rails) [m]."""

CUBESAT_1U_FACE_AREA_M2: float = 0.010
"""Nominal external area of one 1U face (100mm x 100mm) [m^2]."""

# ==============================================================================
# RF & Telecommunication Constants
# ==============================================================================

RF_BAND_ITU_AMATEUR_UHF_HZ: float = 436_500_000.0
"""Default ITU Amateur Satellite space downlink/uplink frequency [Hz] (436.5 MHz)."""

RF_BAND_WPC_INDIA_DELICENSED_HZ: float = 865_500_000.0
"""Default Indian WPC delicensed ISM test frequency [Hz] (865.5 MHz, G.S.R. 564(E))."""

BOLTZMANN_DBM_PER_HZ: float = -174.0
"""Thermal noise spectral power density at 290 Kelvin [dBm / Hz]."""
