"""Hardware Abstraction Layer (HAL) and radio drivers for DOOMSat.

Decouples pure software simulation from physical Hardware-in-the-Loop (HIL) drivers.
Both drivers share an identical programmatic interface.
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, Deque
from collections import deque
import random
import logging

from doomsat.comms.sx1276 import LoRaConfig
from doomsat.comms.link_budget import evaluate_link_budget, LinkBudgetResult

logger = logging.getLogger(__name__)


class RadioDriver(ABC):
    """Abstract interface for satellite and ground station radio transceivers."""

    @abstractmethod
    def send_packet(self, data: bytes) -> bool:
        """Transmit a packet over the radio link."""
        pass

    @abstractmethod
    def receive_packet(self) -> Optional[bytes]:
        """Poll for an incoming received packet."""
        pass

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Return diagnostic status of the radio."""
        pass


class SimulatedSX1276Driver(RadioDriver):
    """Pure software digital twin of the Semtech SX1276 LoRa transceiver and RF channel."""

    def __init__(self, radio_config: LoRaConfig):
        self.config = radio_config
        self.tx_buffer: Deque[bytes] = deque(maxlen=100)
        self.rx_buffer: Deque[bytes] = deque(maxlen=100)
        self.last_link_budget: Optional[LinkBudgetResult] = None
        self.packets_sent: int = 0
        self.packets_delivered: int = 0
        self.packets_dropped: int = 0

    def set_channel_conditions(
        self,
        slant_range_m: float,
        elevation_rad: float,
        radial_velocity_mps: float,
        tx_antenna_gain_dbi: float = 2.15,
        rx_antenna_gain_dbi: float = 12.0,
    ) -> LinkBudgetResult:
        """Update current orbital geometry and evaluate the link budget."""
        lb = evaluate_link_budget(
            slant_range_m=slant_range_m,
            elevation_rad=elevation_rad,
            radial_velocity_mps=radial_velocity_mps,
            radio_config=self.config,
            tx_antenna_gain_dbi=tx_antenna_gain_dbi,
            rx_antenna_gain_dbi=rx_antenna_gain_dbi,
        )
        self.last_link_budget = lb
        return lb

    def send_packet(self, data: bytes) -> bool:
        """Simulate transmitting a packet across the physical RF channel.

        Packet delivery depends on:
        1. Link viability (elevation >= 10 deg, positive link margin).
        2. Statistical Packet Error Rate (PER) evaluated from SNR.
        """
        self.packets_sent += 1
        self.tx_buffer.append(data)

        if self.last_link_budget is None:
            # If no link budget set, assume default benchtop link (100% delivered)
            self.packets_delivered += 1
            return True

        lb = self.last_link_budget
        if not lb.is_link_viable:
            self.packets_dropped += 1
            return False

        # Evaluate packet drop probability from PER
        if random.random() < lb.packet_error_rate:
            self.packets_dropped += 1
            return False

        self.packets_delivered += 1
        return True

    def inject_rx_packet(self, data: bytes) -> None:
        """Inject a packet into the receiver buffer (simulating downlink arrival)."""
        self.rx_buffer.append(data)

    def receive_packet(self) -> Optional[bytes]:
        """Retrieve the next received packet from the buffer."""
        if len(self.rx_buffer) > 0:
            return self.rx_buffer.popleft()
        return None

    def get_status(self) -> Dict[str, Any]:
        return {
            "mode": "SIMULATION",
            "frequency_hz": self.config.frequency_hz,
            "bandwidth_hz": self.config.bandwidth_hz,
            "spreading_factor": self.config.spreading_factor,
            "packets_sent": self.packets_sent,
            "packets_delivered": self.packets_delivered,
            "packets_dropped": self.packets_dropped,
            "last_margin_db": self.last_link_budget.link_margin_db if self.last_link_budget else None,
            "last_snr_db": self.last_link_budget.snr_db if self.last_link_budget else None,
        }


class PhysicalESP32SerialDriver(RadioDriver):
    """Production hardware driver communicating with physical ESP32+SX1276 boards via USB/Serial."""

    def __init__(self, port: str = "COM3", baudrate: int = 115200, timeout_s: float = 0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout_s = timeout_s
        self.serial_conn = None
        self.is_connected: bool = False
        self._init_connection()

    def _init_connection(self) -> None:
        try:
            import serial
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout_s,
            )
            self.is_connected = True
            logger.info(f"Connected to physical radio on {self.port}")
        except Exception as e:
            self.is_connected = False
            logger.warning(f"Could not open physical serial port {self.port}: {e}")

    def send_packet(self, data: bytes) -> bool:
        if not self.is_connected or self.serial_conn is None:
            return False
        try:
            self.serial_conn.write(data)
            return True
        except Exception as e:
            logger.error(f"Serial transmission error: {e}")
            return False

    def receive_packet(self) -> Optional[bytes]:
        if not self.is_connected or self.serial_conn is None:
            return None
        try:
            if self.serial_conn.in_waiting > 0:
                return self.serial_conn.read(self.serial_conn.in_waiting)
        except Exception as e:
            logger.error(f"Serial reception error: {e}")
        return None

    def get_status(self) -> Dict[str, Any]:
        return {
            "mode": "PHYSICAL_SERIAL",
            "port": self.port,
            "baudrate": self.baudrate,
            "is_connected": self.is_connected,
        }
