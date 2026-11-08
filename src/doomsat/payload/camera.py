"""OmniVision OV5647 5MP Camera Payload simulator and synthetic Earth scene generator.

Simulates sensor capture physics:
- Radiometric surface illumination modulated by Solar Zenith Angle (SZA)
- Day/Night and orbital eclipse transitions (zero illumination + thermal dark current)
- Procedural multispectral Earth surface modeling (landmass, ocean, cloud cover, atmospheric haze)
- Lens vignetting according to the optical cosine-fourth law (I = I_0 * cos^4(theta))
- Adjustable resolution (full 2592x1944 or fast downscaled preview 640x480)
"""

import math
from typing import Tuple, Optional
import numpy as np
from numpy.typing import NDArray

from doomsat.payload.optics import (
    PIXEL_WIDTH,
    PIXEL_HEIGHT,
    DEFAULT_FOCAL_LENGTH_M,
    SENSOR_DIAGONAL_M,
)


class OV5647Camera:
    """OmniVision OV5647 5MP CMOS camera sensor simulator."""

    def __init__(
        self,
        full_width: int = PIXEL_WIDTH,
        full_height: int = PIXEL_HEIGHT,
        focal_length_m: float = DEFAULT_FOCAL_LENGTH_M,
        iso_gain: float = 100.0,
        dark_noise_std: float = 2.5,
    ):
        self.full_width = full_width
        self.full_height = full_height
        self.focal_length_m = focal_length_m
        self.iso_gain = iso_gain
        self.dark_noise_std = dark_noise_std
        self.capture_count: int = 0

    def capture_scene(
        self,
        latitude_rad: float,
        longitude_rad: float,
        solar_zenith_angle_rad: float,
        in_eclipse: bool = False,
        render_width: int = 640,
        render_height: int = 480,
        cloud_cover_pct: float = 35.0,
        random_seed: Optional[int] = None,
    ) -> NDArray[np.uint8]:
        """Synthesize an Earth observation image at the sub-satellite point.

        Args:
            latitude_rad: Sub-satellite geodetic latitude [rad].
            longitude_rad: Sub-satellite geodetic longitude [rad].
            solar_zenith_angle_rad: Solar zenith angle at sub-satellite point [rad].
            in_eclipse: True if spacecraft or Earth surface is in shadow.
            render_width: Output horizontal pixel count (default 640 for fast simulation).
            render_height: Output vertical pixel count (default 480 for fast simulation).
            cloud_cover_pct: Cloud fraction [0% - 100%].
            random_seed: Optional RNG seed for deterministic testing.

        Returns:
            RGB uint8 numpy array with shape (render_height, render_width, 3).
        """
        rng = np.random.default_rng(random_seed)
        self.capture_count += 1

        # 1. Radiometric Illumination (Lambertian cosine law)
        # SZA < pi/2 is daytime, SZA >= pi/2 is twilight/night
        if in_eclipse or solar_zenith_angle_rad >= math.pi / 2.0:
            illumination = 0.0
        else:
            illumination = max(0.0, math.cos(solar_zenith_angle_rad))

        # If completely in dark eclipse, return sensor thermal dark current frame
        if illumination <= 1e-4:
            dark_frame = rng.normal(
                loc=8.0,
                scale=self.dark_noise_std,
                size=(render_height, render_width, 3),
            )
            return np.clip(dark_frame, 0, 255).astype(np.uint8)

        # 2. Procedural Terrain Generation
        # Coordinate grid normalized to [-1, 1]
        y_coords = np.linspace(-1.0, 1.0, render_height)
        x_coords = np.linspace(-1.0, 1.0, render_width)
        xx, yy = np.meshgrid(x_coords, y_coords)

        # Harmonic surface texture representing coastlines, continents, and oceans
        # Lat/lon offsets shift the procedural geography
        lat_deg = math.degrees(latitude_rad)
        lon_deg = math.degrees(longitude_rad)
        k_x = 2.0 * math.pi * 1.5
        k_y = 2.0 * math.pi * 1.5

        terrain_noise = (
            0.5 * np.sin(k_x * xx + math.radians(lon_deg))
            + 0.3 * np.cos(k_y * yy + math.radians(lat_deg))
            + 0.2 * np.sin(2.0 * k_x * (xx + yy))
        )

        # Ocean mask: terrain_noise < 0.1 -> Deep blue / turquoise ocean
        # Land mask: terrain_noise >= 0.1 -> Green / ochre terrain
        is_ocean = terrain_noise < 0.1

        # Base RGB colors
        rgb = np.zeros((render_height, render_width, 3), dtype=np.float64)

        # Ocean: deep ocean [15, 60, 140] to shallow turquoise [25, 110, 175]
        ocean_factor = (terrain_noise + 1.0) / 1.1
        rgb[is_ocean, 0] = 15.0 + 10.0 * ocean_factor[is_ocean]
        rgb[is_ocean, 1] = 60.0 + 50.0 * ocean_factor[is_ocean]
        rgb[is_ocean, 2] = 135.0 + 40.0 * ocean_factor[is_ocean]

        # Land: vegetation [35, 95, 30] to arid desert [170, 140, 90]
        land_factor = (terrain_noise[~is_ocean] - 0.1) / 0.9
        rgb[~is_ocean, 0] = 40.0 + 120.0 * land_factor
        rgb[~is_ocean, 1] = 90.0 + 50.0 * land_factor
        rgb[~is_ocean, 2] = 30.0 + 60.0 * land_factor

        # 3. Cloud Layer Simulation
        cloud_freq_x = 4.0 * math.pi
        cloud_freq_y = 4.0 * math.pi
        cloud_noise = (
            0.6 * np.sin(cloud_freq_x * xx + 1.2) * np.cos(cloud_freq_y * yy - 0.7)
            + 0.4 * np.sin(2.5 * cloud_freq_x * (xx - yy))
        )
        # Normalize to [0, 1]
        cloud_noise = (cloud_noise + 1.0) / 2.0

        cloud_threshold = 1.0 - (cloud_cover_pct / 100.0)
        has_clouds = cloud_noise > cloud_threshold
        cloud_alpha = np.clip((cloud_noise - cloud_threshold) / (1.0 - cloud_threshold + 1e-6), 0.0, 1.0)

        # Blend white clouds [240, 240, 250] over ground terrain
        for c in range(3):
            rgb[:, :, c] = (1.0 - cloud_alpha) * rgb[:, :, c] + cloud_alpha * 240.0

        # 4. Atmospheric Rayleigh Scattering (Haze)
        # Adds faint blue veil across the scene
        haze_blue = 25.0 * illumination
        haze_green = 12.0 * illumination
        rgb[:, :, 2] += haze_blue
        rgb[:, :, 1] += haze_green

        # 5. Apply Solar Illumination
        rgb *= illumination

        # 6. Lens Vignetting (Cosine-Fourth Law: I(r) = I_0 * cos^4(alpha))
        # Radial distance from optical center
        r_dist = np.sqrt(xx**2 + yy**2)
        # Angle alpha = arctan(r_sensor / f)
        # Sensor half-diagonal corresponds to r_dist = sqrt(2)
        r_sensor_m = (r_dist / math.sqrt(2.0)) * (SENSOR_DIAGONAL_M / 2.0)
        alpha = np.arctan(r_sensor_m / self.focal_length_m)
        vignette = np.cos(alpha) ** 4

        for c in range(3):
            rgb[:, :, c] *= vignette

        # 7. Sensor Noise
        shot_noise = rng.normal(loc=0.0, scale=self.dark_noise_std, size=rgb.shape)
        rgb += shot_noise

        return np.clip(rgb, 0, 255).astype(np.uint8)
