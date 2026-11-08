"""Camera payload (OmniVision OV5647), optical geometry, compression, and CSTP fragmentation package."""

from doomsat.payload.optics import (
    PIXEL_WIDTH,
    PIXEL_HEIGHT,
    PIXEL_PITCH_M,
    SENSOR_WIDTH_M,
    SENSOR_HEIGHT_M,
    SENSOR_DIAGONAL_M,
    DEFAULT_FOCAL_LENGTH_M,
    compute_field_of_view,
    compute_ground_sampling_distance,
    compute_swath_dimensions,
    get_optical_profile,
)
from doomsat.payload.camera import OV5647Camera
from doomsat.payload.compression import (
    compress_jpeg,
    decompress_jpeg,
    CompressionResult,
)
from doomsat.payload.fragmenter import (
    compute_crc16_ccitt,
    ImageDescriptor,
    ImageChunk,
    fragment_image,
    ImageDefragmenter,
)

__all__ = [
    "PIXEL_WIDTH",
    "PIXEL_HEIGHT",
    "PIXEL_PITCH_M",
    "SENSOR_WIDTH_M",
    "SENSOR_HEIGHT_M",
    "SENSOR_DIAGONAL_M",
    "DEFAULT_FOCAL_LENGTH_M",
    "compute_field_of_view",
    "compute_ground_sampling_distance",
    "compute_swath_dimensions",
    "get_optical_profile",
    "OV5647Camera",
    "compress_jpeg",
    "decompress_jpeg",
    "CompressionResult",
    "compute_crc16_ccitt",
    "ImageDescriptor",
    "ImageChunk",
    "fragment_image",
    "ImageDefragmenter",
]
