"""Satellite Communications Controller (ESP32) and packet router.

Models the firmware running on the onboard Espressif ESP32-WROOM-32D:
- High-speed UART link with Raspberry Pi Zero 2 W OBC (115200 baud).
- SPI link with Semtech SX1276 LoRa transceiver.
- Downlink transmission queue and Uplink command queue.
- Periodic heartbeat generation kicking the OBC watchdog.
- Selective Repeat ARQ (Automatic Repeat reQuest) retransmission manager.
"""

from typing import List, Dict, Optional, Deque, Tuple
from collections import deque
import logging

from doomsat.comms.cstp import (
    CSTPPacket,
    NodeID,
    MessageType,
    slip_encode,
    slip_decode,
)
from doomsat.comms.sx1276 import LoRaConfig, compute_airtime, get_sx1276_current_consumption_ma

logger = logging.getLogger(__name__)


class ARQRecord:
    """Tracks a transmitted packet awaiting ground acknowledgement."""
    def __init__(self, packet: CSTPPacket, dispatch_time_s: float, timeout_s: float = 15.0):
        self.packet = packet
        self.dispatch_time_s = dispatch_time_s
        self.timeout_s = timeout_s
        self.retry_count: int = 0
        self.acknowledged: bool = False


class ESP32Controller:
    """Simulates the ESP32-WROOM-32D onboard communications controller."""

    def __init__(
        self,
        radio_config: LoRaConfig,
        heartbeat_interval_s: float = 10.0,
        arq_timeout_s: float = 15.0,
        max_retries: int = 3,
    ):
        self.radio_config = radio_config
        self.heartbeat_interval_s = heartbeat_interval_s
        self.arq_timeout_s = arq_timeout_s
        self.max_retries = max_retries

        self.last_heartbeat_time_s: float = 0.0
        self.sequence_counter: int = 0

        # Queues
        self.downlink_queue: Deque[CSTPPacket] = deque(maxlen=200)
        self.uplink_queue: Deque[CSTPPacket] = deque(maxlen=50)

        # ARQ tracking
        self.unacked_packets: Dict[int, ARQRecord] = {}

        # Transceiver activity state
        self.is_transmitting: bool = False
        self.is_receiving: bool = True
        self.transmission_end_time_s: float = 0.0

        # Statistics
        self.packets_transmitted: int = 0
        self.packets_received: int = 0
        self.retransmission_count: int = 0

    def queue_downlink_packet(self, packet: CSTPPacket) -> None:
        """Enqueue an outbound CSTP packet for transmission."""
        self.downlink_queue.append(packet)

    def process_incoming_rf_packet(self, packet_bytes: bytes, current_time_s: float) -> Optional[CSTPPacket]:
        """Ingest a demodulated RF packet from the SX1276 receiver.

        Verifies CSTP framing, inspects message type, handles ARQ ACKs,
        and routes telecommands to the uplink queue.
        """
        try:
            packet = CSTPPacket.unpack(packet_bytes)
            self.packets_received += 1

            # Check if this is an ACK / NACK for an in-flight packet
            if packet.msg_type == MessageType.CHUNK_ACK_NACK:
                self._handle_ack_nack(packet)
            elif packet.msg_type in (MessageType.COMMAND_REQUEST, MessageType.PING):
                self.uplink_queue.append(packet)

            return packet
        except Exception as e:
            logger.warning(f"Failed to unpack incoming RF packet: {e}")
            return None

    def _handle_ack_nack(self, ack_packet: CSTPPacket) -> None:
        """Handle ACK or NACK payload."""
        # Payload carries sequence_num (2 bytes) + status (1 byte: 0x06=ACK, 0x15=NACK)
        if len(ack_packet.payload) >= 3:
            acked_seq = (ack_packet.payload[0] << 8) | ack_packet.payload[1]
            status_byte = ack_packet.payload[2]

            if acked_seq in self.unacked_packets:
                if status_byte == 0x06:  # ACK
                    self.unacked_packets[acked_seq].acknowledged = True
                    del self.unacked_packets[acked_seq]
                elif status_byte == 0x15:  # NACK -> Immediate re-queue
                    record = self.unacked_packets[acked_seq]
                    if record.retry_count < self.max_retries:
                        record.retry_count += 1
                        self.retransmission_count += 1
                        self.downlink_queue.appendleft(record.packet)

    def step(self, current_time_s: float) -> Tuple[Optional[bytes], bool]:
        """Execute one cycle of the ESP32 controller state machine.

        Returns:
            Tuple of (outgoing_packet_bytes_or_None, watchdog_heartbeat_pulse).
        """
        pulse_watchdog = False

        # 1. Heartbeat generator (issues ping to OBC)
        if (current_time_s - self.last_heartbeat_time_s) >= self.heartbeat_interval_s:
            pulse_watchdog = True
            self.last_heartbeat_time_s = current_time_s

        # 2. Update transceiver state (check if current transmission finished)
        if self.is_transmitting and current_time_s >= self.transmission_end_time_s:
            self.is_transmitting = False
            self.is_receiving = True

        # 3. Check for ARQ timeouts
        timed_out_seqs = []
        for seq, record in self.unacked_packets.items():
            if (current_time_s - record.dispatch_time_s) > record.timeout_s:
                timed_out_seqs.append(seq)

        for seq in timed_out_seqs:
            record = self.unacked_packets.pop(seq)
            if record.retry_count < self.max_retries:
                record.retry_count += 1
                self.retransmission_count += 1
                self.downlink_queue.appendleft(record.packet)

        # 4. Transmit next queued downlink packet if radio is available
        outgoing_bytes = None
        if not self.is_transmitting and len(self.downlink_queue) > 0:
            packet = self.downlink_queue.popleft()
            packet_raw = packet.pack()

            # Calculate on-air duration
            airtime = compute_airtime(len(packet_raw), self.radio_config)
            self.is_transmitting = True
            self.is_receiving = False
            self.transmission_end_time_s = current_time_s + airtime.total_airtime_s
            self.packets_transmitted += 1

            # If packet requires reliable delivery (e.g. image chunks), register in ARQ
            if packet.msg_type in (MessageType.IMAGE_CHUNK, MessageType.IMAGE_DESCRIPTOR):
                self.unacked_packets[packet.sequence_num] = ARQRecord(
                    packet=packet,
                    dispatch_time_s=current_time_s,
                    timeout_s=self.arq_timeout_s,
                )

            outgoing_bytes = packet_raw

        return outgoing_bytes, pulse_watchdog

    @property
    def power_consumption_w(self) -> float:
        """Instantaneous power consumption of ESP32 + SX1276 radio [W]."""
        # ESP32 active core: 240 mW (0.24 W) at 3.3V
        esp32_pwr = 0.240

        # SX1276 RF current
        sx1276_current_ma = get_sx1276_current_consumption_ma(
            tx_power_dbm=self.radio_config.tx_power_dbm,
            is_transmitting=self.is_transmitting,
            is_receiving=self.is_receiving,
        )
        sx1276_pwr = 3.3 * (sx1276_current_ma / 1000.0)

        return esp32_pwr + sx1276_pwr
