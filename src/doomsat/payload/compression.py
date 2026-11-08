"""JPEG image compression and decompression engine for DOOMSat payload data."""

import io
from typing import Tuple
import numpy as np
from numpy.typing import NDArray
from PIL import Image
from pydantic import BaseModel, Field


class CompressionResult(BaseModel):
    """Metadata and binary payload resulting from JPEG compression."""
    quality: int = Field(..., ge=1, le=100)
    raw_size_bytes: int
    compressed_size_bytes: int
    compression_ratio: float
    width: int
    height: int
    data: bytes

    model_config = {
        "arbitrary_types_allowed": True
    }


def compress_jpeg(
    image_rgb: NDArray[np.uint8],
    quality: int = 75,
    optimize: bool = True,
) -> CompressionResult:
    """Compress an RGB numpy array image into a standard JPEG byte stream.

    Args:
        image_rgb: Array of shape (H, W, 3) and dtype uint8.
        quality: JPEG compression factor (1 to 100, nominal 75).
        optimize: Enable Huffman table optimization for size reduction.

    Returns:
        CompressionResult instance containing JPEG bytes and metrics.
    """
    if image_rgb.ndim != 3 or image_rgb.shape[2] != 3:
        raise ValueError(f"Expected RGB image array of shape (H, W, 3), got {image_rgb.shape}")
    if not (1 <= quality <= 100):
        raise ValueError(f"Quality must be between 1 and 100, got {quality}")

    h, w, _ = image_rgb.shape
    raw_size = image_rgb.nbytes

    img = Image.fromarray(image_rgb, mode="RGB")
    buffer = io.BytesIO()
    img.save(buffer, format="JPEG", quality=quality, optimize=optimize)
    compressed_data = buffer.getvalue()
    compressed_size = len(compressed_data)

    ratio = raw_size / max(1, compressed_size)

    return CompressionResult(
        quality=quality,
        raw_size_bytes=raw_size,
        compressed_size_bytes=compressed_size,
        compression_ratio=ratio,
        width=w,
        height=h,
        data=compressed_data,
    )


def decompress_jpeg(jpeg_bytes: bytes) -> NDArray[np.uint8]:
    """Decompress a JPEG byte stream back into an RGB uint8 numpy array.

    Args:
        jpeg_bytes: Raw JPEG binary data.

    Returns:
        RGB numpy array of shape (H, W, 3) and dtype uint8.
    """
    buffer = io.BytesIO(jpeg_bytes)
    with Image.open(buffer) as img:
        rgb_img = img.convert("RGB")
        return np.array(rgb_img, dtype=np.uint8)
