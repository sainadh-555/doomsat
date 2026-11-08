"""Unit tests for camera payload, optical geometry, JPEG compression, and CSTP packet fragmentation."""

import math
import numpy as np
import pytest

from doomsat.payload import (
    compute_field_of_view,
    compute_ground_sampling_distance,
    compute_swath_dimensions,
    get_optical_profile,
    OV5647Camera,
    compress_jpeg,
    decompress_jpeg,
    compute_crc16_ccitt,
    ImageDescriptor,
    ImageChunk,
    fragment_image,
    ImageDefragmenter,
)


def test_optical_geometry_calculations():
    """Verify optical geometry calculations (GSD, swath, FOV) against analytical formulas."""
    altitude_m = 500_000.0  # 500 km LEO
    focal_length_m = 0.0036  # 3.6 mm
    pixel_pitch_m = 1.4e-6   # 1.4 um

    # GSD = (p * h) / f = (1.4e-6 * 500e3) / 0.0036 = 194.444... m/pixel
    expected_gsd = (pixel_pitch_m * altitude_m) / focal_length_m
    calculated_gsd = compute_ground_sampling_distance(altitude_m)
    assert math.isclose(calculated_gsd, expected_gsd, rel_tol=1e-10)
    assert 194.0 < calculated_gsd < 195.0

    # Diagonal FOV should be approximately 65 degrees
    hfov, vfov, dfov = compute_field_of_view()
    assert math.isclose(math.degrees(dfov), 65.0, abs_tol=1.0)
    assert math.isclose(math.degrees(hfov), 54.0, abs_tol=1.0)
    assert math.isclose(math.degrees(vfov), 42.0, abs_tol=1.0)

    # Swath dimensions: 2592 * 194.44m ~= 504 km; 1944 * 194.44m ~= 378 km
    swath_w, swath_h, area = compute_swath_dimensions(altitude_m)
    assert 500_000.0 < swath_w < 510_000.0
    assert 370_000.0 < swath_h < 385_000.0
    assert area > 1.8e11


def test_off_nadir_gsd_elongation():
    """Verify that off-nadir pointing increases GSD by 1 / cos^2(eta)."""
    altitude_m = 500_000.0
    off_nadir_angle = math.radians(30.0)  # 30 deg

    gsd_nadir = compute_ground_sampling_distance(altitude_m, off_nadir_angle_rad=0.0)
    gsd_off = compute_ground_sampling_distance(altitude_m, off_nadir_angle_rad=off_nadir_angle)

    # GSD_off = GSD_nadir / cos^2(30 deg) = GSD_nadir / 0.75 = 1.333 * GSD_nadir
    expected_factor = 1.0 / (math.cos(off_nadir_angle)**2)
    assert math.isclose(gsd_off / gsd_nadir, expected_factor, rel_tol=1e-8)


def test_camera_synthetic_scene_daylight_vs_eclipse():
    """Verify synthetic scene generation produces distinct daylight vs dark frames."""
    camera = OV5647Camera()

    # Daytime capture: SZA = 20 deg, lat=17.38 deg, lon=78.48 deg (Hyderabad)
    day_img = camera.capture_scene(
        latitude_rad=math.radians(17.3850),
        longitude_rad=math.radians(78.4867),
        solar_zenith_angle_rad=math.radians(20.0),
        in_eclipse=False,
        render_width=160,
        render_height=120,
        random_seed=42,
    )
    assert day_img.shape == (120, 160, 3)
    assert day_img.dtype == np.uint8
    assert np.mean(day_img) > 50.0  # Significant optical illumination

    # Eclipse capture: in_eclipse = True
    dark_img = camera.capture_scene(
        latitude_rad=math.radians(17.3850),
        longitude_rad=math.radians(78.4867),
        solar_zenith_angle_rad=math.radians(20.0),
        in_eclipse=True,
        render_width=160,
        render_height=120,
        random_seed=42,
    )
    assert dark_img.shape == (120, 160, 3)
    # Average pixel value should be near thermal dark noise (~8 DN)
    assert np.mean(dark_img) < 20.0


def test_jpeg_compression_and_decompression():
    """Verify JPEG compression, quality control, and decompression recovery."""
    camera = OV5647Camera()
    raw_img = camera.capture_scene(
        latitude_rad=0.0,
        longitude_rad=0.0,
        solar_zenith_angle_rad=math.radians(15.0),
        render_width=320,
        render_height=240,
        random_seed=123,
    )

    # Compress at quality 75
    result = compress_jpeg(raw_img, quality=75)
    assert result.compressed_size_bytes < result.raw_size_bytes
    assert result.compression_ratio > 5.0  # Typically 8:1 to 15:1 for JPEG
    assert len(result.data) == result.compressed_size_bytes

    # Decompress back to numpy array
    recovered_img = decompress_jpeg(result.data)
    assert recovered_img.shape == raw_img.shape
    assert recovered_img.dtype == np.uint8


def test_crc16_ccitt_checksum():
    """Verify CRC-16-CCITT implementation against known standard test vector."""
    # Standard ASCII "123456789" with initial 0xFFFF has CRC-16-CCITT = 0x29B1
    test_bytes = b"123456789"
    crc = compute_crc16_ccitt(test_bytes, initial_val=0xFFFF)
    assert crc == 0x29B1


def test_packet_fragmentation_and_lossless_reassembly():
    """Verify full round-trip: fragmentation into MTU-compliant chunks and lossless defragmentation."""
    camera = OV5647Camera()
    raw_img = camera.capture_scene(
        latitude_rad=math.radians(17.385),
        longitude_rad=math.radians(78.486),
        solar_zenith_angle_rad=math.radians(30.0),
        render_width=320,
        render_height=240,
        random_seed=999,
    )
    compressed = compress_jpeg(raw_img, quality=60)
    original_jpeg_bytes = compressed.data

    # Fragment with MTU payload size = 200 bytes
    descriptor, chunks = fragment_image(
        image_id=101,
        image_bytes=original_jpeg_bytes,
        width=320,
        height=240,
        gsd_m=194.4,
        capture_time_s=1234.5,
        latitude_rad=0.303,
        longitude_rad=1.369,
        chunk_payload_size=200,
    )

    # Verify all chunks satisfy MTU limit <= 220 bytes
    assert len(chunks) == descriptor.total_chunks
    for chunk in chunks:
        assert len(chunk.payload) <= 200
        # Check serialization and deserialization
        packed_bytes = chunk.pack()
        unpacked_chunk = ImageChunk.unpack(packed_bytes)
        assert unpacked_chunk.image_id == 101
        assert unpacked_chunk.chunk_index == chunk.chunk_index
        assert unpacked_chunk.payload == chunk.payload

    # Reassemble using ImageDefragmenter
    defragmenter = ImageDefragmenter(descriptor)
    assert not defragmenter.is_complete
    assert defragmenter.progress_pct == 0.0

    # Ingest chunks
    for chunk in chunks:
        defragmenter.ingest_chunk(chunk)

    assert defragmenter.is_complete
    assert math.isclose(defragmenter.progress_pct, 100.0)
    assert len(defragmenter.get_missing_chunk_indices()) == 0

    # Reconstruct byte stream
    reconstructed_bytes = defragmenter.reconstruct()
    assert reconstructed_bytes == original_jpeg_bytes

    # Verify reconstructed image can be decoded and viewed
    final_img = decompress_jpeg(reconstructed_bytes)
    assert final_img.shape == (240, 320, 3)


def test_missing_chunk_arq_detection():
    """Verify that defragmenter detects missing chunks for selective retransmission."""
    fake_data = b"X" * 1000
    descriptor, chunks = fragment_image(
        image_id=202,
        image_bytes=fake_data,
        width=100,
        height=100,
        gsd_m=200.0,
        capture_time_s=0.0,
        latitude_rad=0.0,
        longitude_rad=0.0,
        chunk_payload_size=200,  # 5 chunks
    )
    assert len(chunks) == 5

    defragmenter = ImageDefragmenter(descriptor)
    # Ingest chunks 0, 1, 3, 4 (chunk 2 is missing)
    defragmenter.ingest_chunk(chunks[0])
    defragmenter.ingest_chunk(chunks[1])
    defragmenter.ingest_chunk(chunks[3])
    defragmenter.ingest_chunk(chunks[4])

    assert not defragmenter.is_complete
    assert defragmenter.get_missing_chunk_indices() == [2]

    # Reconstructing should raise an error
    with pytest.raises(ValueError) as exc:
        defragmenter.reconstruct()
    assert "Cannot reconstruct image" in str(exc.value)

    # Supplying missing chunk 2 completes the stream
    defragmenter.ingest_chunk(chunks[2])
    assert defragmenter.is_complete
    assert defragmenter.reconstruct() == fake_data
