"""Ground Station CSTP packet receiver, demodulation, and telemetry handler for DOOMSat."""

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
import json
import time

from doomsat.comms.cstp import (
    CSTPPacket,
    MessageType,
    NodeID,
    slip_decode,
    slip_encode,
)
from doomsat.comms.drivers import RadioDriver
from doomsat.core.schemas import TelemetryBeacon
from doomsat.payload.fragmenter import ImageDescriptor, ImageChunk
from doomsat.ground_station.telemetry_store import TelemetryStore
from doomsat.ground_station.image_reconstruction import GroundStationImageManager


@dataclass
class GroundStationMetrics:
    """Aggregated RF link and ground station reception statistics."""
    total_packets_received: int = 0
    valid_packets: int = 0
    crc_errors: int = 0
    beacons_received: int = 0
    image_chunks_received: int = 0
    commands_uplinked: int = 0
    last_rssi_dbm: Optional[float] = None
    last_snr_db: Optional[float] = None
    mean_snr_db: float = 0.0
    last_contact_time_s: float = 0.0

    @property
    def packet_delivery_ratio(self) -> float:
        """Packet Delivery Ratio (PDR) percentage."""
        if self.total_packets_received == 0:
            return 100.0
        return (self.valid_packets / float(self.total_packets_received)) * 100.0


class GroundStationReceiver:
    """Ground station software managing CSTP frame ingestion, telemetry, and payload downlinks."""

    def __init__(
        self,
        radio_driver: Optional[RadioDriver] = None,
        telemetry_store: Optional[TelemetryStore] = None,
        image_manager: Optional[GroundStationImageManager] = None,
    ) -> None:
        self.radio_driver = radio_driver
        self.telemetry_store = telemetry_store if telemetry_store is not None else TelemetryStore()
        self.image_manager = image_manager if image_manager is not None else GroundStationImageManager()
        self.metrics = GroundStationMetrics()
        self._command_seq_counter: int = 1
        self._last_command_response: Optional[Dict[str, Any]] = None

    def process_raw_frame(
        self,
        frame_bytes: bytes,
        rssi_dbm: Optional[float] = None,
        snr_db: Optional[float] = None,
    ) -> Tuple[bool, Optional[str]]:
        """Process an incoming SLIP-encoded CSTP radio frame.

        Args:
            frame_bytes: Raw bytes received from the radio driver (SLIP framed).
            rssi_dbm: Ground station received signal strength [dBm].
            snr_db: Ground station received signal-to-noise ratio [dB].

        Returns:
            Tuple of (success: bool, status_message: str).
        """
        self.metrics.total_packets_received += 1
        if rssi_dbm is not None:
            self.metrics.last_rssi_dbm = rssi_dbm
        if snr_db is not None:
            self.metrics.last_snr_db = snr_db
            # Exponential moving average for SNR
            if self.metrics.valid_packets == 0:
                self.metrics.mean_snr_db = snr_db
            else:
                self.metrics.mean_snr_db = 0.1 * snr_db + 0.9 * self.metrics.mean_snr_db

        # 1. SLIP decode
        try:
            cstp_raw = slip_decode(frame_bytes)
        except Exception as e:
            self.metrics.crc_errors += 1
            return False, f"SLIP decode error: {e}"

        # 2. CSTP unpack and CRC-16 check
        try:
            packet = CSTPPacket.unpack(cstp_raw)
        except ValueError as e:
            self.metrics.crc_errors += 1
            return False, f"CSTP packet error: {e}"

        self.metrics.valid_packets += 1
        self.metrics.last_contact_time_s = time.time()

        # 3. Route according to MessageType
        return self._route_packet(packet, rssi_dbm=rssi_dbm, snr_db=snr_db)

    def _route_packet(
        self,
        packet: CSTPPacket,
        rssi_dbm: Optional[float] = None,
        snr_db: Optional[float] = None,
    ) -> Tuple[bool, str]:
        """Dispatch CSTP packet payload to appropriate subsystem handler."""
        msg_type = packet.msg_type

        # A. Telemetry Beacon (0x10)
        if msg_type == MessageType.TELEMETRY_BEACON:
            try:
                if packet.payload.startswith(b"{"):
                    beacon_dict = json.loads(packet.payload.decode("utf-8"))
                    if rssi_dbm is not None:
                        beacon_dict["rssi_dbm"] = rssi_dbm
                    if snr_db is not None:
                        beacon_dict["snr_db"] = snr_db
                    beacon = TelemetryBeacon(**beacon_dict)
                else:
                    beacon = TelemetryBeacon.unpack_binary(packet.payload)
                    beacon.rssi_dbm = rssi_dbm
                    beacon.snr_db = snr_db

                self.telemetry_store.add_beacon(beacon)
                self.metrics.beacons_received += 1
                return True, f"Beacon seq={beacon.sequence_id} stored"
            except Exception as e:
                return False, f"Beacon parsing error: {e}"

        # B. Image Descriptor (0x20)
        elif msg_type == MessageType.IMAGE_DESCRIPTOR:
            try:
                desc = ImageDescriptor.unpack(packet.payload)
                self.image_manager.register_descriptor(desc)
                return True, f"Image descriptor img_id={desc.image_id} registered ({desc.total_chunks} chunks)"
            except Exception as e:
                return False, f"Image descriptor unpack error: {e}"

        # C. Image Chunk (0x21)
        elif msg_type == MessageType.IMAGE_CHUNK:
            try:
                chunk = ImageChunk.unpack(packet.payload)
                accepted, complete = self.image_manager.ingest_chunk(chunk)
                if accepted:
                    self.metrics.image_chunks_received += 1
                    status = "Image complete!" if complete else f"Chunk {chunk.chunk_index} ingested"
                    return True, status
                return False, "Chunk rejected by image manager"
            except Exception as e:
                return False, f"Image chunk unpack error: {e}"

        # D. Command Response (0x31)
        elif msg_type == MessageType.COMMAND_RESPONSE:
            try:
                resp_str = packet.payload.decode("utf-8", errors="replace")
                self._last_command_response = {"raw": resp_str, "timestamp": time.time()}
                return True, f"Command response: {resp_str}"
            except Exception as e:
                return False, f"Command response decode error: {e}"

        # E. Ping (0x01)
        elif msg_type == MessageType.PING:
            return True, "Ping acknowledged"

        return True, f"Unhandled CSTP message type: {msg_type}"

    def poll_radio(self) -> int:
        """Poll physical or simulated radio driver for incoming packets.

        Returns:
            Number of packets processed during this poll cycle.
        """
        if self.radio_driver is None:
            return 0

        packets_processed = 0
        while True:
            raw_frame = self.radio_driver.receive_packet()
            if raw_frame is None:
                break
            self.process_raw_frame(raw_frame)
            packets_processed += 1

        return packets_processed

    def send_command(
        self,
        command_str: str,
        target_node: NodeID = NodeID.OBC,
    ) -> bool:
        """Format and transmit a command request packet to the spacecraft.

        Args:
            command_str: ASCII or JSON command payload string (e.g. "CAPTURE_IMAGE", "DETUMBLE").
            target_node: Destination subsystem node (defaults to OBC).

        Returns:
            True if transmitted successfully by radio driver.
        """
        if self.radio_driver is None:
            return False

        seq = self._command_seq_counter
        self._command_seq_counter += 1

        payload_bytes = command_str.encode("utf-8")
        packet = CSTPPacket(
            src_id=NodeID.GROUND_STATION,
            dst_id=target_node,
            msg_type=MessageType.COMMAND_REQUEST,
            sequence_num=seq,
            flags=0x00,
            payload=payload_bytes,
        )

        framed = slip_encode(packet.pack())
        success = self.radio_driver.send_packet(framed)
        if success:
            self.metrics.commands_uplinked += 1
        return success
