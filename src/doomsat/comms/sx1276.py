"""Semtech SX1276 LoRa transceiver physical layer parameters and airtime calculation engine.

Directly implemented from Semtech SX1276/77/78/79 Datasheet (Rev 7, 2020),
including Section 4.1.1.6 ("LoRa Packet Structure and Airtime Calculation").
"""

import math
from enum import Enum
from typing import Dict, Tuple, Optional
from pydantic import BaseModel, Field


class CodingRate(str, Enum):
    CR_4_5 = "4/5"  # Index 1
    CR_4_6 = "4/6"  # Index 2
    CR_4_7 = "4/7"  # Index 3
    CR_4_8 = "4/8"  # Index 4


# Minimum demodulation SNR threshold for each Spreading Factor [dB]
# Source: Semtech SX1276 Datasheet Table 10 & Table 13
SNR_DEMODULATION_LIMITS_DB: Dict[int, float] = {
    6: -5.0,
    7: -7.5,
    8: -10.0,
    9: -12.5,
    10: -15.0,
    11: -17.5,
    12: -20.0,
}

# Standard SX1276 Receiver Noise Figure [dB]
SX1276_NOISE_FIGURE_DB: float = 6.0


class LoRaConfig(BaseModel):
    """Configuration parameters for the Semtech SX1276 radio."""
    frequency_hz: float = Field(436_500_000.0, description="Carrier frequency [Hz]")
    bandwidth_hz: float = Field(125_000.0, description="LoRa channel bandwidth [Hz]")
    spreading_factor: int = Field(9, ge=6, le=12, description="Spreading Factor (SF6 to SF12)")
    coding_rate: CodingRate = Field(CodingRate.CR_4_5, description="Forward error correction coding rate")
    tx_power_dbm: float = Field(17.0, ge=2.0, le=20.0, description="RF transmit output power [dBm]")
    preamble_symbols: int = Field(8, ge=6, le=65535, description="Preamble symbol count")
    enable_crc: bool = Field(True, description="Enable 16-bit payload CRC")
    explicit_header: bool = Field(True, description="Explicit header mode (False = implicit)")
    auto_ldro: bool = Field(True, description="Automatically enable Low Data Rate Optimization when Ts > 16ms")


class AirtimeResult(BaseModel):
    """Breakdown of calculated LoRa packet airtime."""
    symbol_duration_s: float
    preamble_duration_s: float
    payload_symbols: int
    payload_duration_s: float
    total_airtime_s: float
    bitrate_bps: float
    low_data_rate_opt_active: bool


def compute_airtime(
    payload_bytes_len: int,
    config: LoRaConfig,
) -> AirtimeResult:
    """Calculate the exact on-air transmission time for a LoRa packet.

    Implements Semtech SX1276 Datasheet Section 4.1.1.6:
        T_s = 2^SF / BW
        T_preamble = (N_preamble + 4.25) * T_s

        N_payload = 8 + max( ceil( (8*PL - 4*SF + 28 + 16*CRC - 20*IH) / (4*(SF - 2*DE)) ) * (CR + 4), 0 )

        T_payload = N_payload * T_s
        T_packet = T_preamble + T_payload

    Args:
        payload_bytes_len: Size of payload data in bytes (PL).
        config: LoRa radio configuration parameters.

    Returns:
        AirtimeResult instance with timing breakdown.
    """
    sf = config.spreading_factor
    bw = config.bandwidth_hz
    pl = payload_bytes_len
    n_pre = config.preamble_symbols

    # Symbol period [s]
    t_s = (2.0**sf) / bw

    # Low Data Rate Optimization (DE): Mandatory/recommended if symbol duration exceeds 16.38 ms
    if config.auto_ldro:
        ldro_active = t_s > 0.01638
    else:
        ldro_active = False
    de = 1 if ldro_active else 0

    # Header parameters
    ih = 0 if config.explicit_header else 1
    crc = 1 if config.enable_crc else 0

    # Coding Rate index: CR = 1 for 4/5, 2 for 4/6, 3 for 4/7, 4 for 4/8
    cr_map = {
        CodingRate.CR_4_5: 1,
        CodingRate.CR_4_6: 2,
        CodingRate.CR_4_7: 3,
        CodingRate.CR_4_8: 4,
    }
    cr_idx = cr_map[config.coding_rate]

    # Preamble duration
    t_preamble = (n_pre + 4.25) * t_s

    # Payload symbols calculation
    numerator = 8 * pl - 4 * sf + 28 + 16 * crc - 20 * ih
    denominator = 4 * (sf - 2 * de)

    # In SF6, explicit header is not supported and formulas slightly vary, but standard allows ceil
    term = math.ceil(numerator / denominator)
    payload_symbols = 8 + max(term * (cr_idx + 4), 0)

    t_payload = payload_symbols * t_s
    total_airtime = t_preamble + t_payload

    # Raw effective bitrate [bps]
    effective_bitrate = (pl * 8.0) / total_airtime if total_airtime > 0 else 0.0

    return AirtimeResult(
        symbol_duration_s=t_s,
        preamble_duration_s=t_preamble,
        payload_symbols=payload_symbols,
        payload_duration_s=t_payload,
        total_airtime_s=total_airtime,
        bitrate_bps=effective_bitrate,
        low_data_rate_opt_active=ldro_active,
    )


def compute_receiver_sensitivity(
    bandwidth_hz: float,
    spreading_factor: int,
    noise_figure_db: float = SX1276_NOISE_FIGURE_DB,
) -> float:
    """Calculate the theoretical SX1276 receiver sensitivity threshold [dBm].

    Equation:
        S = -174.0 + 10 * log10(BW) + NF + SNR_limit

    Args:
        bandwidth_hz: Channel bandwidth [Hz].
        spreading_factor: Spreading Factor (6 to 12).
        noise_figure_db: Receiver front-end noise figure [dB].

    Returns:
        Sensitivity threshold [dBm] (e.g., ~ -129 dBm for SF9, BW=125kHz).
    """
    snr_limit = SNR_DEMODULATION_LIMITS_DB.get(spreading_factor, -12.5)
    thermal_noise_floor = -174.0 + 10.0 * math.log10(bandwidth_hz)
    sensitivity = thermal_noise_floor + noise_figure_db + snr_limit
    return sensitivity


def get_sx1276_current_consumption_ma(
    tx_power_dbm: float,
    is_transmitting: bool = False,
    is_receiving: bool = False,
) -> float:
    """Return estimated DC supply current [mA] for the SX1276 transceiver.

    Datasheet Ratings:
        - Standby: 1.6 mA
        - Rx continuous: 10.8 mA
        - Tx at +14 dBm (RFO pin): 44.0 mA
        - Tx at +17 dBm (PA_BOOST): 90.0 mA
        - Tx at +20 dBm (PA_BOOST with 20dBm reg): 120.0 mA
    """
    if is_transmitting:
        if tx_power_dbm <= 14.0:
            return 44.0
        elif tx_power_dbm <= 17.0:
            return 90.0
        else:
            return 120.0
    elif is_receiving:
        return 10.8
    else:
        # Standby
        return 1.6
