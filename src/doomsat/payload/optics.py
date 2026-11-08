"""Optical geometry, Ground Sampling Distance (GSD), and field of view calculations for DOOMSat.

All calculations follow first-principles aerial and spaceborne photogrammetry.
All units are strictly SI:
- Dimensions, focal length, pixel pitch: meters (m)
- Altitude, slant range, swath width: meters (m)
- Angles: radians (rad), unless explicitly denoted with _deg.

Sensor Baseline: OmniVision OV5647 5MP CMOS
- Resolution: 2592 x 1944 active pixels
- Pixel pitch: 1.4 um = 1.4e-6 m
- Sensor width: 3.674 mm = 0.003674 m
- Sensor height: 2.760 mm = 0.002760 m
- Sensor diagonal: 4.595 mm = 0.004595 m
- Focal length: 3.6 mm = 0.0036 m (M12 mount)
"""

import math
from typing import Tuple, Dict, Any


# Manufacturer Datasheet Parameters (OmniVision OV5647)
PIXEL_WIDTH: int = 2592
PIXEL_HEIGHT: int = 1944
PIXEL_PITCH_M: float = 1.4e-6
SENSOR_WIDTH_M: float = 0.003674
SENSOR_HEIGHT_M: float = 0.002760
SENSOR_DIAGONAL_M: float = 0.004595
DEFAULT_FOCAL_LENGTH_M: float = 0.0036


def compute_field_of_view(
    sensor_width_m: float = SENSOR_WIDTH_M,
    sensor_height_m: float = SENSOR_HEIGHT_M,
    sensor_diagonal_m: float = SENSOR_DIAGONAL_M,
    focal_length_m: float = DEFAULT_FOCAL_LENGTH_M,
) -> Tuple[float, float, float]:
    """Calculate the angular Field of View (Horizontal, Vertical, Diagonal).

    Equations:
        HFOV = 2 * arctan(W_sensor / (2 * f))
        VFOV = 2 * arctan(H_sensor / (2 * f))
        DFOV = 2 * arctan(D_sensor / (2 * f))

    Args:
        sensor_width_m: Sensor active width [m].
        sensor_height_m: Sensor active height [m].
        sensor_diagonal_m: Sensor diagonal dimension [m].
        focal_length_m: Lens focal length [m].

    Returns:
        Tuple of (hfov_rad, vfov_rad, dfov_rad).
    """
    if focal_length_m <= 0.0:
        raise ValueError(f"Focal length must be positive: {focal_length_m} m")

    hfov = 2.0 * math.atan(sensor_width_m / (2.0 * focal_length_m))
    vfov = 2.0 * math.atan(sensor_height_m / (2.0 * focal_length_m))
    dfov = 2.0 * math.atan(sensor_diagonal_m / (2.0 * focal_length_m))

    return hfov, vfov, dfov


def compute_ground_sampling_distance(
    altitude_m: float,
    pixel_pitch_m: float = PIXEL_PITCH_M,
    focal_length_m: float = DEFAULT_FOCAL_LENGTH_M,
    off_nadir_angle_rad: float = 0.0,
) -> float:
    """Calculate the Ground Sampling Distance (GSD) projected on Earth surface.

    Nadir Equation:
        GSD_nadir = (pixel_pitch * altitude) / focal_length

    Off-nadir Equation (accounting for slant range d = h / cos(eta)):
        GSD_off_nadir = GSD_nadir / (cos^2(eta))

    Args:
        altitude_m: Spacecraft altitude above Earth surface [m].
        pixel_pitch_m: Physical pixel dimension [m].
        focal_length_m: Lens focal length [m].
        off_nadir_angle_rad: Off-nadir pointing angle (eta) [rad].

    Returns:
        Projected Ground Sampling Distance [m / pixel].
    """
    if altitude_m <= 0.0:
        raise ValueError(f"Altitude must be positive: {altitude_m} m")
    if focal_length_m <= 0.0:
        raise ValueError(f"Focal length must be positive: {focal_length_m} m")

    cos_eta = math.cos(off_nadir_angle_rad)
    if cos_eta <= 1e-6:
        raise ValueError(f"Off-nadir angle {math.degrees(off_nadir_angle_rad):.1f} deg is beyond horizon.")

    gsd_nadir = (pixel_pitch_m * altitude_m) / focal_length_m
    gsd = gsd_nadir / (cos_eta**2)

    return gsd


def compute_swath_dimensions(
    altitude_m: float,
    pixel_width: int = PIXEL_WIDTH,
    pixel_height: int = PIXEL_HEIGHT,
    pixel_pitch_m: float = PIXEL_PITCH_M,
    focal_length_m: float = DEFAULT_FOCAL_LENGTH_M,
    off_nadir_angle_rad: float = 0.0,
) -> Tuple[float, float, float]:
    """Calculate the swath width, height, and surface area footprint.

    Equations:
        Swath_X = GSD * N_pixels_x
        Swath_Y = GSD * N_pixels_y
        Footprint_Area = Swath_X * Swath_Y

    Args:
        altitude_m: Spacecraft altitude [m].
        pixel_width: Horizontal pixel count.
        pixel_height: Vertical pixel count.
        pixel_pitch_m: Physical pixel pitch [m].
        focal_length_m: Focal length [m].
        off_nadir_angle_rad: Off-nadir angle [rad].

    Returns:
        Tuple of (swath_width_m, swath_height_m, footprint_area_m2).
    """
    gsd = compute_ground_sampling_distance(
        altitude_m=altitude_m,
        pixel_pitch_m=pixel_pitch_m,
        focal_length_m=focal_length_m,
        off_nadir_angle_rad=off_nadir_angle_rad,
    )

    swath_x = gsd * float(pixel_width)
    swath_y = gsd * float(pixel_height)
    area = swath_x * swath_y

    return swath_x, swath_y, area


def get_optical_profile(altitude_m: float = 500_000.0) -> Dict[str, Any]:
    """Return a comprehensive dictionary of optical parameters for DOOMSat at given altitude."""
    hfov, vfov, dfov = compute_field_of_view()
    gsd = compute_ground_sampling_distance(altitude_m)
    swath_w, swath_h, footprint_area = compute_swath_dimensions(altitude_m)

    return {
        "sensor_name": "OmniVision OV5647 5MP",
        "altitude_km": altitude_m / 1000.0,
        "focal_length_mm": DEFAULT_FOCAL_LENGTH_M * 1000.0,
        "pixel_pitch_um": PIXEL_PITCH_M * 1e6,
        "resolution": (PIXEL_WIDTH, PIXEL_HEIGHT),
        "total_pixels_mp": (PIXEL_WIDTH * PIXEL_HEIGHT) / 1e6,
        "hfov_deg": math.degrees(hfov),
        "vfov_deg": math.degrees(vfov),
        "dfov_deg": math.degrees(dfov),
        "gsd_m_per_pixel": gsd,
        "swath_width_km": swath_w / 1000.0,
        "swath_height_km": swath_h / 1000.0,
        "footprint_area_sq_km": footprint_area / 1e6,
    }
