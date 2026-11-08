"""Packet Fragmentation & Reassembly (PFR) for DOOMSat image transmission over CSTP.

Partitions compressed JPEG files into MTU-compliant chunks (<= 220 bytes)
and reassembles them with CRC-16-CCITT verification.
"""

from typing import List, Dict, Optional, Tuple, Set
import struct
from pydantic import BaseModel, Field


def compute_crc16_ccitt(data: bytes, initial_val: int = 0xFFFF) -> int:
    """Calculate CRC-16-CCITT (polynomial 0x1021) checksum.

    Args:
        data: Input bytes.
        initial_val: Initial register value (0xFFFF standard).

    Returns:
        16-bit integer checksum.
    """
    crc = initial_val
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class ImageDescriptor(BaseModel):
    """Metadata descriptor packet (CSTP Message Type 0x20) sent prior to image downlink."""
    image_id: int = Field(..., ge=0)
    width: int = Field(..., gt=0)
    height: int = Field(..., gt=0)
    compressed_size_bytes: int = Field(..., gt=0)
    total_chunks: int = Field(..., gt=0)
    chunk_payload_size: int = Field(..., gt=0, le=220)
    gsd_m: float = Field(..., gt=0.0)
    capture_time_s: float
    latitude_rad: float
    longitude_rad: float

    def pack(self) -> bytes:
        """Binary serialize descriptor: 28 bytes."""
        # Format: >H H H I H H f f f f
        # image_id (u16), width (u16), height (u16), size (u32),
        # total_chunks (u16), chunk_size (u16), gsd (f32),
        # time (f32), lat (f32), lon (f32)
        return struct.pack(
            ">HHHIHHffff",
            self.image_id,
            self.width,
            self.height,
            self.compressed_size_bytes,
            self.total_chunks,
            self.chunk_payload_size,
            self.gsd_m,
            self.capture_time_s,
            self.latitude_rad,
            self.longitude_rad,
        )

    @classmethod
    def unpack(cls, data: bytes) -> "ImageDescriptor":
        """Deserialize binary descriptor."""
        unpacked = struct.unpack(">HHHIHHffff", data[:28])
        return cls(
            image_id=unpacked[0],
            width=unpacked[1],
            height=unpacked[2],
            compressed_size_bytes=unpacked[3],
            total_chunks=unpacked[4],
            chunk_payload_size=unpacked[5],
            gsd_m=unpacked[6],
            capture_time_s=unpacked[7],
            latitude_rad=unpacked[8],
            longitude_rad=unpacked[9],
        )


class ImageChunk(BaseModel):
    """Individual image chunk packet (CSTP Message Type 0x21)."""
    image_id: int
    chunk_index: int
    byte_offset: int
    payload: bytes
    crc16: int

    model_config = {
        "arbitrary_types_allowed": True
    }

    def pack(self) -> bytes:
        """Binary serialize chunk: Header (8 bytes) + Payload + CRC (2 bytes).

        Header:
            image_id (u16)
            chunk_index (u16)
            byte_offset (u32)
            payload (N bytes)
            crc16 (u16)
        """
        header = struct.pack(">HHI", self.image_id, self.chunk_index, self.byte_offset)
        raw_body = header + self.payload
        crc = compute_crc16_ccitt(raw_body)
        return raw_body + struct.pack(">H", crc)

    @classmethod
    def unpack(cls, data: bytes) -> "ImageChunk":
        """Deserialize binary chunk and verify CRC."""
        if len(data) < 10:
            raise ValueError(f"Data length {len(data)} too short for ImageChunk")

        header = data[:8]
        image_id, chunk_index, byte_offset = struct.unpack(">HHI", header)
        payload = data[8:-2]
        received_crc = struct.unpack(">H", data[-2:])[0]

        computed_crc = compute_crc16_ccitt(data[:-2])
        if received_crc != computed_crc:
            raise ValueError(
                f"CRC mismatch for chunk {chunk_index}: received 0x{received_crc:04X}, computed 0x{computed_crc:04X}"
            )

        return cls(
            image_id=image_id,
            chunk_index=chunk_index,
            byte_offset=byte_offset,
            payload=payload,
            crc16=received_crc,
        )


def fragment_image(
    image_id: int,
    image_bytes: bytes,
    width: int,
    height: int,
    gsd_m: float,
    capture_time_s: float,
    latitude_rad: float,
    longitude_rad: float,
    chunk_payload_size: int = 210,
) -> Tuple[ImageDescriptor, List[ImageChunk]]:
    """Partition a compressed image into an ImageDescriptor and MTU-compliant ImageChunks.

    Args:
        image_id: Unique integer identifier.
        image_bytes: Raw JPEG compressed bytes.
        width: Image width in pixels.
        height: Image height in pixels.
        gsd_m: Ground Sampling Distance [m].
        capture_time_s: Epoch time of capture [s].
        latitude_rad: Geodetic latitude [rad].
        longitude_rad: Geodetic longitude [rad].
        chunk_payload_size: Maximum payload size per chunk (default 210 <= 220 MTU).

    Returns:
        Tuple of (ImageDescriptor, list of ImageChunk).
    """
    total_size = len(image_bytes)
    total_chunks = (total_size + chunk_payload_size - 1) // chunk_payload_size

    descriptor = ImageDescriptor(
        image_id=image_id,
        width=width,
        height=height,
        compressed_size_bytes=total_size,
        total_chunks=total_chunks,
        chunk_payload_size=chunk_payload_size,
        gsd_m=gsd_m,
        capture_time_s=capture_time_s,
        latitude_rad=latitude_rad,
        longitude_rad=longitude_rad,
    )

    chunks: List[ImageChunk] = []
    for idx in range(total_chunks):
        offset = idx * chunk_payload_size
        slice_end = min(total_size, offset + chunk_payload_size)
        payload_slice = image_bytes[offset:slice_end]

        # Calculate CRC over header + payload slice
        header = struct.pack(">HHI", image_id, idx, offset)
        crc = compute_crc16_ccitt(header + payload_slice)

        chunk = ImageChunk(
            image_id=image_id,
            chunk_index=idx,
            byte_offset=offset,
            payload=payload_slice,
            crc16=crc,
        )
        chunks.append(chunk)

    return descriptor, chunks


class ImageDefragmenter:
    """Reassembles received image chunks into a complete JPEG file and tracks missing chunks."""

    def __init__(self, descriptor: ImageDescriptor):
        self.descriptor = descriptor
        self.received_chunks: Dict[int, ImageChunk] = {}
        self.expected_total: int = descriptor.total_chunks

    def ingest_chunk(self, chunk: ImageChunk) -> bool:
        """Store an ingested chunk if valid.

        Returns:
            True if chunk is accepted and belongs to this image.
        """
        if chunk.image_id != self.descriptor.image_id:
            return False
        if not (0 <= chunk.chunk_index < self.expected_total):
            return False

        self.received_chunks[chunk.chunk_index] = chunk
        return True

    @property
    def progress_pct(self) -> float:
        """Reception progress percentage [0.0% - 100.0%]."""
        return (len(self.received_chunks) / float(self.expected_total)) * 100.0

    @property
    def is_complete(self) -> bool:
        """True if all expected chunks have been received without gaps."""
        return len(self.received_chunks) == self.expected_total

    def get_missing_chunk_indices(self) -> List[int]:
        """Return a sorted list of missing chunk indices for selective NACK ARQ."""
        missing = [idx for idx in range(self.expected_total) if idx not in self.received_chunks]
        return missing

    def reconstruct(self) -> bytes:
        """Reassemble the received chunks into the original JPEG byte stream.

        Raises:
            ValueError if chunks are missing.
        """
        if not self.is_complete:
            missing = self.get_missing_chunk_indices()
            raise ValueError(f"Cannot reconstruct image: {len(missing)} chunks missing ({missing[:5]}...)")

        # Sort chunks by index
        sorted_chunks = [self.received_chunks[idx] for idx in range(self.expected_total)]
        full_data = b"".join(c.payload for c in sorted_chunks)

        if len(full_data) != self.descriptor.compressed_size_bytes:
            raise ValueError(
                f"Reconstructed size mismatch: expected {self.descriptor.compressed_size_bytes}, got {len(full_data)}"
            )

        return full_data
