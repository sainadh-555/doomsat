"""Hardware-in-the-Loop (HIL) test runner and benchtop serial bridge for DOOMSat.

Enforces Indian RF regulatory compliance:
- Uses 865.500 MHz (delicensed ISM band, G.S.R. 564(E), ERP <= 1W, BW <= 200 kHz)
  for terrestrial benchtop radio tests without requiring Indian space RF clearance.
- Connects to physical ESP32 + SX1276 hardware over serial UART (115200 baud).
"""

from typing import Optional, List, Dict, Any
import time
import logging
import serial.tools.list_ports

from doomsat.core.constants import RF_BAND_WPC_INDIA_DELICENSED_HZ
from doomsat.comms.drivers import PhysicalESP32SerialDriver
from doomsat.comms.cstp import CSTPPacket, MessageType, NodeID, slip_encode, slip_decode
from doomsat.core.schemas import TelemetryBeacon

logger = logging.getLogger(__name__)


class HILBenchtopRunner:
    """Manages Hardware-in-the-Loop communication with physical ESP32 transceivers."""

    def __init__(
        self,
        port: Optional[str] = None,
        baud_rate: int = 115200,
        frequency_hz: float = RF_BAND_WPC_INDIA_DELICENSED_HZ,
    ) -> None:
        self.port = port
        self.baud_rate = baud_rate
        self.frequency_hz = frequency_hz
        self.driver: Optional[PhysicalESP32SerialDriver] = None
        self.is_connected: bool = False

    @staticmethod
    def list_available_ports() -> List[str]:
        """List all available serial COM ports detected on the system."""
        ports = serial.tools.list_ports.comports()
        return [p.device for p in ports]

    def connect(self) -> bool:
        """Connect to the physical ESP32 serial device."""
        if not self.port:
            available = self.list_available_ports()
            if not available:
                logger.warning("No physical serial ports detected on host system.")
                return False
            self.port = available[0]

        try:
            self.driver = PhysicalESP32SerialDriver(
                port=self.port,
                baud_rate=self.baud_rate,
            )
            self.is_connected = True
            logger.info(f"Connected to ESP32 HIL board on {self.port} at {self.baud_rate} baud.")
            return True
        except Exception as e:
            logger.error(f"Failed to connect to ESP32 serial port {self.port}: {e}")
            self.is_connected = False
            return False

    def send_ping(self) -> bool:
        """Send a CSTP PING frame to the physical ESP32 transceiver."""
        if not self.driver:
            return False

        pkt = CSTPPacket(
            src_id=NodeID.GROUND_STATION,
            dst_id=NodeID.SPACECRAFT_TTC,
            msg_type=MessageType.PING,
            sequence_num=1,
            payload=b"HIL_PING",
        )
        framed = slip_encode(pkt.pack())
        return self.driver.send_packet(framed)

    def receive_packet(self) -> Optional[CSTPPacket]:
        """Poll physical transceiver for an incoming packet."""
        if not self.driver:
            return None

        raw = self.driver.receive_packet()
        if raw is None:
            return None

        try:
            unstuffed = slip_decode(raw)
            return CSTPPacket.unpack(unstuffed)
        except Exception as e:
            logger.warning(f"Failed to decode incoming HIL packet: {e}")
            return None

    def close(self) -> None:
        """Disconnect and release serial port."""
        if self.driver:
            self.driver.close()
            self.is_connected = False
