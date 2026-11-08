"""DOOMSat Communications, TT&C controller (ESP32), Semtech SX1276 LoRa, and CSTP protocol package."""

from doomsat.comms.cstp import (
    NodeID,
    MessageType,
    CSTPPacket,
    slip_encode,
    slip_decode,
    CSTP_MAX_PAYLOAD,
    CSTP_SYNC_WORD,
)
from doomsat.comms.sx1276 import (
    LoRaConfig,
    CodingRate,
    AirtimeResult,
    compute_airtime,
    compute_receiver_sensitivity,
    get_sx1276_current_consumption_ma,
    SNR_DEMODULATION_LIMITS_DB,
    SX1276_NOISE_FIGURE_DB,
)
from doomsat.comms.link_budget import (
    LinkBudgetResult,
    compute_fspl,
    compute_atmospheric_loss,
    compute_doppler_shift,
    compute_ber_per,
    evaluate_link_budget,
)
from doomsat.comms.esp32_controller import (
    ESP32Controller,
    ARQRecord,
)
from doomsat.comms.drivers import (
    RadioDriver,
    SimulatedSX1276Driver,
    PhysicalESP32SerialDriver,
)

__all__ = [
    "NodeID",
    "MessageType",
    "CSTPPacket",
    "slip_encode",
    "slip_decode",
    "CSTP_MAX_PAYLOAD",
    "CSTP_SYNC_WORD",
    "LoRaConfig",
    "CodingRate",
    "AirtimeResult",
    "compute_airtime",
    "compute_receiver_sensitivity",
    "get_sx1276_current_consumption_ma",
    "SNR_DEMODULATION_LIMITS_DB",
    "SX1276_NOISE_FIGURE_DB",
    "LinkBudgetResult",
    "compute_fspl",
    "compute_atmospheric_loss",
    "compute_doppler_shift",
    "compute_ber_per",
    "evaluate_link_budget",
    "ESP32Controller",
    "ARQRecord",
    "RadioDriver",
    "SimulatedSX1276Driver",
    "PhysicalESP32SerialDriver",
]
