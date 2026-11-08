"""Ground station image reconstruction and selective ARQ manager for DOOMSat."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import io
import base64
from PIL import Image

from doomsat.payload.fragmenter import (
    ImageDescriptor,
    ImageChunk,
    ImageDefragmenter,
)
from doomsat.comms.cstp import CSTPPacket, MessageType


@dataclass
class ReconstructedImageRecord:
    """Record of a successfully reconstructed satellite payload image."""
    image_id: int
    width: int
    height: int
    size_bytes: int
    total_chunks: int
    capture_time_s: float
    gsd_m: float
    latitude_rad: float
    longitude_rad: float
    raw_jpeg_bytes: bytes
    base64_thumbnail: str
    retransmission_requests: int = 0


class GroundStationImageManager:
    """Manages multi-session image downlinks, chunk reassembly, and selective ARQ NACK generation."""

    def __init__(self, thumbnail_max_dim: int = 128) -> None:
        self.thumbnail_max_dim = thumbnail_max_dim
        # Active defragmentation sessions indexed by image_id
        self._active_sessions: Dict[int, ImageDefragmenter] = {}
        # Completed images
        self._completed_images: Dict[int, ReconstructedImageRecord] = {}
        # Retransmission counters per image
        self._nack_counts: Dict[int, int] = {}

    def register_descriptor(self, descriptor: ImageDescriptor) -> None:
        """Register or update an image metadata descriptor."""
        if descriptor.image_id not in self._active_sessions:
            self._active_sessions[descriptor.image_id] = ImageDefragmenter(descriptor)
            self._nack_counts[descriptor.image_id] = 0

    def ingest_chunk(self, chunk: ImageChunk) -> Tuple[bool, bool]:
        """Ingest an incoming image chunk.

        Returns:
            Tuple of (accepted: bool, is_complete: bool).
        """
        session = self._active_sessions.get(chunk.image_id)
        if session is None:
            # If chunk arrived before descriptor, synthesize temporary descriptor
            # assuming typical chunk payload size
            est_chunks = chunk.chunk_index + 1
            temp_desc = ImageDescriptor(
                image_id=chunk.image_id,
                width=640,
                height=480,
                compressed_size_bytes=len(chunk.payload) * est_chunks,
                total_chunks=max(1, est_chunks),
                chunk_payload_size=len(chunk.payload),
                gsd_m=194.4,
                capture_time_s=0.0,
                latitude_rad=0.0,
                longitude_rad=0.0,
            )
            session = ImageDefragmenter(temp_desc)
            self._active_sessions[chunk.image_id] = session
            self._nack_counts[chunk.image_id] = 0

        # Ingest into defragmenter
        if chunk.chunk_index >= session.expected_total:
            # Dynamically expand expected chunks if descriptor was an estimate
            session.expected_total = chunk.chunk_index + 1
            session.descriptor.total_chunks = session.expected_total

        accepted = session.ingest_chunk(chunk)
        if not accepted:
            return False, False

        # Check if completed
        if session.is_complete:
            self._finalize_image(session)
            return True, True

        return True, False

    def _finalize_image(self, session: ImageDefragmenter) -> None:
        """Reconstruct full image bytes, generate thumbnail, and store record."""
        try:
            jpeg_bytes = session.reconstruct()
        except Exception:
            # If exact size check failed due to dynamic estimate, reconstruct best-effort
            sorted_chunks = [session.received_chunks[i] for i in range(session.expected_total) if i in session.received_chunks]
            jpeg_bytes = b"".join(c.payload for c in sorted_chunks)

        # Generate base64 thumbnail
        thumb_b64 = ""
        width = session.descriptor.width
        height = session.descriptor.height
        try:
            img = Image.open(io.BytesIO(jpeg_bytes))
            width, height = img.size
            img.thumbnail((self.thumbnail_max_dim, self.thumbnail_max_dim))
            buf = io.BytesIO()
            img.save(buf, format="JPEG", quality=75)
            thumb_b64 = "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode("ascii")
        except Exception:
            thumb_b64 = ""

        record = ReconstructedImageRecord(
            image_id=session.descriptor.image_id,
            width=width,
            height=height,
            size_bytes=len(jpeg_bytes),
            total_chunks=session.expected_total,
            capture_time_s=session.descriptor.capture_time_s,
            gsd_m=session.descriptor.gsd_m,
            latitude_rad=session.descriptor.latitude_rad,
            longitude_rad=session.descriptor.longitude_rad,
            raw_jpeg_bytes=jpeg_bytes,
            base64_thumbnail=thumb_b64,
            retransmission_requests=self._nack_counts.get(session.descriptor.image_id, 0),
        )
        self._completed_images[session.descriptor.image_id] = record

    def get_progress(self, image_id: int) -> float:
        """Get reception progress percentage [0.0 - 100.0] for image_id."""
        if image_id in self._completed_images:
            return 100.0
        session = self._active_sessions.get(image_id)
        if session is not None:
            return session.progress_pct
        return 0.0

    def generate_selective_nack_packet(self, image_id: int, max_indices_per_packet: int = 100) -> Optional[CSTPPacket]:
        """Generate a CSTP NACK packet containing missing chunk indices for ARQ uplink.

        Args:
            image_id: Target image identifier.
            max_indices_per_packet: Max chunk IDs to include in one frame.

        Returns:
            CSTPPacket with MessageType.NACK containing binary encoded missing indices, or None if complete.
        """
        session = self._active_sessions.get(image_id)
        if session is None or session.is_complete:
            return None

        missing = session.get_missing_chunk_indices()
        if not missing:
            return None

        self._nack_counts[image_id] = self._nack_counts.get(image_id, 0) + 1

        # Format binary payload:
        # image_id (u16) + count (u16) + chunk_idx_1 (u16) + ...
        limited_missing = missing[:max_indices_per_packet]
        import struct
        payload = struct.pack(">HH", image_id, len(limited_missing))
        for idx in limited_missing:
            payload += struct.pack(">H", idx)

        packet = CSTPPacket(
            src_id=1,  # Ground Station
            dst_id=5,  # Payload
            msg_type=MessageType.CHUNK_ACK_NACK,
            sequence_num=self._nack_counts[image_id],
            flags=0x00,
            payload=payload,
        )
        return packet

    def get_image(self, image_id: int) -> Optional[ReconstructedImageRecord]:
        """Retrieve completed image record."""
        return self._completed_images.get(image_id)

    def list_completed_images(self) -> List[ReconstructedImageRecord]:
        """List all completed images."""
        return list(self._completed_images.values())

    def clear(self) -> None:
        """Clear all active sessions and completed images."""
        self._active_sessions.clear()
        self._completed_images.clear()
        self._nack_counts.clear()
