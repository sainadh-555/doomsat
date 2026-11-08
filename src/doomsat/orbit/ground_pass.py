"""Ground station topocentric horizon coordinates, pass prediction, and visibility events for DOOMSat.

Transforms spacecraft state to Topocentric North-East-Up (NEU) frame
relative to the Ground Station (Hyderabad, India).
"""

import math
from typing import Tuple, List, Optional
import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, Field

from doomsat.core.coordinates import geodetic_to_ecef, eci_to_ecef
from doomsat.core.constants import EARTH_OMEGA_ROTATION


class LookAngles(BaseModel):
    """Instantaneous look angles from Ground Station to Spacecraft."""
    azimuth_deg: float = Field(..., ge=0.0, le=360.0, description="Azimuth angle clockwise from North [deg]")
    elevation_deg: float = Field(..., ge=-90.0, le=90.0, description="Elevation angle above horizon [deg]")
    slant_range_m: float = Field(..., ge=0.0, description="Line-of-sight slant range distance [m]")
    radial_velocity_mps: float = Field(..., description="Range rate (positive = moving away, negative = approaching) [m/s]")
    is_visible: bool = Field(..., description="True if elevation exceeds min_elevation threshold")


class GroundPassEvent(BaseModel):
    """Summary of a ground station visibility pass."""
    pass_id: int
    aos_time_s: float = Field(..., description="Acquisition of Signal epoch [s]")
    tca_time_s: float = Field(..., description="Time of Closest Approach epoch [s]")
    los_time_s: float = Field(..., description="Loss of Signal epoch [s]")
    duration_s: float = Field(..., description="Pass duration [s]")
    max_elevation_deg: float = Field(..., description="Maximum pass elevation [deg]")
    min_slant_range_m: float = Field(..., description="Slant range at TCA [m]")


class GroundStation:
    """Represents a terrestrial ground station antenna facility."""

    def __init__(
        self,
        name: str = "Hyderabad-GS",
        latitude_deg: float = 17.3850,
        longitude_deg: float = 78.4867,
        altitude_m: float = 505.0,
        min_elevation_deg: float = 10.0,
    ):
        self.name = name
        self.latitude_deg = latitude_deg
        self.longitude_deg = longitude_deg
        self.latitude_rad = math.radians(latitude_deg)
        self.longitude_rad = math.radians(longitude_deg)
        self.altitude_m = altitude_m
        self.min_elevation_deg = min_elevation_deg

        # Ground station position in ECEF [m]
        self.r_gs_ecef: NDArray[np.float64] = geodetic_to_ecef(
            self.latitude_rad,
            self.longitude_rad,
            self.altitude_m,
        )

        # Transformation matrix from ECEF to topocentric North-East-Up (NEU)
        # Up = [cos(lat)cos(lon), cos(lat)sin(lon), sin(lat)]
        # East = [-sin(lon), cos(lon), 0]
        # North = [-sin(lat)cos(lon), -sin(lat)sin(lon), cos(lat)]
        sin_lat, cos_lat = math.sin(self.latitude_rad), math.cos(self.latitude_rad)
        sin_lon, cos_lon = math.sin(self.longitude_rad), math.cos(self.longitude_rad)

        north_vec = np.array([-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat], dtype=np.float64)
        east_vec = np.array([-sin_lon, cos_lon, 0.0], dtype=np.float64)
        up_vec = np.array([cos_lat * cos_lon, cos_lat * sin_lon, sin_lat], dtype=np.float64)

        self.dcm_ecef_to_neu = np.vstack([north_vec, east_vec, up_vec])

    def compute_look_angles(
        self,
        r_sat_eci: NDArray[np.float64],
        v_sat_eci: NDArray[np.float64],
        gmst_rad: float,
    ) -> LookAngles:
        """Calculate topocentric look angles (Azimuth, Elevation, Slant Range) and range rate.

        Args:
            r_sat_eci: Spacecraft position vector in ECI [m].
            v_sat_eci: Spacecraft velocity vector in ECI [m/s].
            gmst_rad: Greenwich Mean Sidereal Time [rad].

        Returns:
            LookAngles instance.
        """
        # Transform spacecraft from ECI to ECEF
        r_sat_ecef = eci_to_ecef(r_sat_eci, gmst_rad)

        # Spacecraft velocity in ECEF: v_ecef = R_z(gmst) * (v_eci - omega_earth x r_eci)
        omega_earth_vec = np.array([0.0, 0.0, EARTH_OMEGA_ROTATION], dtype=np.float64)
        v_inertial_rel = v_sat_eci - np.cross(omega_earth_vec, r_sat_eci)
        v_sat_ecef = eci_to_ecef(v_inertial_rel, gmst_rad)

        # Slant vector in ECEF from Ground Station to Spacecraft
        rho_ecef = r_sat_ecef - self.r_gs_ecef
        slant_range = float(np.linalg.norm(rho_ecef))

        # Rotate slant vector into NEU horizon frame: [North, East, Up]^T
        rho_neu = np.dot(self.dcm_ecef_to_neu, rho_ecef)
        n, e, u = rho_neu[0], rho_neu[1], rho_neu[2]

        # Elevation angle (arcsin(Up / slant_range))
        elev_rad = math.asin(max(-1.0, min(1.0, u / slant_range)))
        elev_deg = math.degrees(elev_rad)

        # Azimuth angle (clockwise from North: 0 deg = North, 90 deg = East)
        az_rad = math.atan2(e, n)
        if az_rad < 0.0:
            az_rad += 2.0 * math.pi
        az_deg = math.degrees(az_rad)

        # Slant range rate (radial velocity: positive = moving away)
        # v_radial = (rho_ecef . v_sat_ecef) / slant_range
        v_radial = float(np.dot(rho_ecef, v_sat_ecef)) / slant_range

        is_visible = elev_deg >= self.min_elevation_deg

        return LookAngles(
            azimuth_deg=az_deg,
            elevation_deg=elev_deg,
            slant_range_m=slant_range,
            radial_velocity_mps=v_radial,
            is_visible=is_visible,
        )

