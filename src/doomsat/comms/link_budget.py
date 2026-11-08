"""RF link budget, propagation loss, Doppler shift, and BER/PER channel modeling for DOOMSat.

All calculations follow rigorous telecommunications and satellite link design equations.
All units are strictly SI:
- Frequency: Hertz (Hz)
- Distance and Slant Range: meters (m)
- Angles: radians (rad)
- Power: dBm and Watts
"""

import math
from typing import Dict, Any, Tuple
from pydantic import BaseModel, Field
from scipy.special import erfc

from doomsat.core.constants import SPEED_OF_LIGHT
from doomsat.comms.sx1276 import (
    LoRaConfig,
    compute_receiver_sensitivity,
    SNR_DEMODULATION_LIMITS_DB,
)


class LinkBudgetResult(BaseModel):
    """Detailed instantaneous satellite-to-ground RF link budget."""
    slant_range_m: float
    elevation_deg: float
    doppler_shift_hz: float
    fspl_db: float
    atmospheric_loss_db: float
    polarization_loss_db: float
    tx_power_dbm: float
    eirp_dbm: float
    rx_power_dbm: float
    noise_floor_dbm: float
    snr_db: float
    rx_sensitivity_dbm: float
    link_margin_db: float
    bit_error_rate: float
    packet_error_rate: float
    is_link_viable: bool


def compute_fspl(slant_range_m: float, frequency_hz: float) -> float:
    """Calculate Free Space Path Loss (FSPL) in dB.

    Equation:
        FSPL = 20 * log10(d) + 20 * log10(f) + 20 * log10(4 * pi / c)

    Args:
        slant_range_m: Line-of-sight distance [m].
        frequency_hz: Carrier frequency [Hz].

    Returns:
        Path loss in decibels [dB].
    """
    if slant_range_m <= 0.0 or frequency_hz <= 0.0:
        raise ValueError("Slant range and frequency must be positive.")

    c = SPEED_OF_LIGHT
    fspl_db = (
        20.0 * math.log10(slant_range_m)
        + 20.0 * math.log10(frequency_hz)
        + 20.0 * math.log10(4.0 * math.pi / c)
    )
    return fspl_db


def compute_atmospheric_loss(elevation_rad: float, frequency_hz: float = 436.5e6) -> float:
    """Estimate atmospheric gas, water vapor, and tropospheric absorption in dB.

    Follows ITU-R P.676 model approximation for UHF/VHF:
        L_atm = L_zenith / sin(max(elevation, 5 deg))

    Args:
        elevation_rad: Ground station look angle elevation [rad].
        frequency_hz: Carrier frequency [Hz].

    Returns:
        Atmospheric loss in decibels [dB].
    """
    # At 436.5 MHz (UHF), dry air + water vapor zenith attenuation is ~0.05 dB
    # At 865 MHz, zenith attenuation is ~0.08 dB
    l_zenith = 0.05 if frequency_hz < 600e6 else 0.08

    # Prevent elevation singularity at horizon (minimum 5 degrees elevation = 0.087 rad)
    effective_elev = max(math.radians(5.0), elevation_rad)
    loss = l_zenith / math.sin(effective_elev)

    return loss


def compute_doppler_shift(
    relative_radial_velocity_mps: float,
    carrier_frequency_hz: float,
) -> float:
    """Calculate Doppler frequency shift on the radio carrier.

    Equation:
        Delta f_D = - (v_radial / c) * f_c

    Convention:
        v_radial > 0 (spacecraft moving away): negative Doppler shift (frequency drops).
        v_radial < 0 (spacecraft approaching): positive Doppler shift (frequency rises).

    Args:
        relative_radial_velocity_mps: Spacecraft velocity projected onto LOS vector [m/s].
        carrier_frequency_hz: Center RF carrier frequency [Hz].

    Returns:
        Doppler frequency offset in Hertz [Hz].
    """
    doppler_hz = - (relative_radial_velocity_mps / SPEED_OF_LIGHT) * carrier_frequency_hz
    return doppler_hz


def compute_ber_per(
    snr_db: float,
    spreading_factor: int,
    payload_bytes_len: int,
) -> Tuple[float, float]:
    """Calculate Bit Error Rate (BER) and Packet Error Rate (PER) for LoRa in AWGN.

    LoRa modulation is equivalent to M-ary orthogonal FSK (M = 2^SF).
    The union bound / Q-function approximation for BER is:
        BER = 0.5 * erfc( sqrt( 2 * 10^(SNR/10) * (2^SF / SF) ) / sqrt(2) )
            = 0.5 * erfc( sqrt( 10^(SNR/10) * (2^SF / SF) ) )

    Packet Error Rate (PER):
        PER = 1 - (1 - BER)^(8 * N_bytes)

    Args:
        snr_db: Signal-to-Noise Ratio at receiver demodulator [dB].
        spreading_factor: Spreading Factor (6 to 12).
        payload_bytes_len: Payload length in bytes.

    Returns:
        Tuple of (BER, PER).
    """
    sf = spreading_factor
    snr_linear = 10.0 ** (snr_db / 10.0)

    # Effective Eb/N0 factor for M-ary FSK with M = 2^SF
    gamma = snr_linear * (2.0**sf / float(sf))

    if gamma > 50.0:  # High SNR -> BER practically zero
        ber = 1e-15
    elif gamma < 0.01:  # Extremely low SNR -> random guessing (BER = 0.5)
        ber = 0.5
    else:
        # erfc(x) / 2
        ber = 0.5 * float(erfc(math.sqrt(gamma)))
        ber = max(1e-15, min(0.5, ber))

    total_bits = max(1, payload_bytes_len * 8)
    per = 1.0 - (1.0 - ber)**total_bits
    per = max(0.0, min(1.0, per))

    return ber, per


def evaluate_link_budget(
    slant_range_m: float,
    elevation_rad: float,
    radial_velocity_mps: float,
    radio_config: LoRaConfig,
    tx_antenna_gain_dbi: float = 2.15,
    rx_antenna_gain_dbi: float = 12.0,
    tx_feeder_loss_db: float = 0.5,
    rx_feeder_loss_db: float = 1.0,
    polarization_loss_db: float = 3.0,
    rx_noise_figure_db: float = 4.0,
    payload_bytes_len: int = 50,
) -> LinkBudgetResult:
    """Evaluate complete space-to-ground RF link budget."""
    fc = radio_config.frequency_hz
    bw = radio_config.bandwidth_hz
    sf = radio_config.spreading_factor
    p_tx = radio_config.tx_power_dbm

    # Path loss and atmospheric attenuation
    fspl = compute_fspl(slant_range_m, fc)
    l_atm = compute_atmospheric_loss(elevation_rad, fc)
    doppler = compute_doppler_shift(radial_velocity_mps, fc)

    # Transmit EIRP: P_tx + G_tx - L_tx
    eirp = p_tx + tx_antenna_gain_dbi - tx_feeder_loss_db

    # Received Power: EIRP - PathLosses + G_rx - L_rx
    p_rx = eirp - fspl - l_atm - polarization_loss_db + rx_antenna_gain_dbi - rx_feeder_loss_db

    # Receiver Noise Floor: -174 dBm/Hz + 10*log10(BW) + NF
    n_0 = -174.0 + 10.0 * math.log10(bw) + rx_noise_figure_db

    # SNR
    snr = p_rx - n_0

    # Sensitivity and Margin
    sensitivity = compute_receiver_sensitivity(bw, sf, rx_noise_figure_db)
    margin = p_rx - sensitivity

    # BER and PER
    ber, per = compute_ber_per(snr, sf, payload_bytes_len)

    is_viable = (margin >= 0.0) and (math.degrees(elevation_rad) >= 10.0)

    return LinkBudgetResult(
        slant_range_m=slant_range_m,
        elevation_deg=math.degrees(elevation_rad),
        doppler_shift_hz=doppler,
        fspl_db=fspl,
        atmospheric_loss_db=l_atm,
        polarization_loss_db=polarization_loss_db,
        tx_power_dbm=p_tx,
        eirp_dbm=eirp,
        rx_power_dbm=p_rx,
        noise_floor_dbm=n_0,
        snr_db=snr,
        rx_sensitivity_dbm=sensitivity,
        link_margin_db=margin,
        bit_error_rate=ber,
        packet_error_rate=per,
        is_link_viable=is_viable,
    )
