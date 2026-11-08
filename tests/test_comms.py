"""Unit tests for CSTP packet protocol, SX1276 LoRa transceiver, link budget, and ESP32 controller."""

import math
import numpy as np
import pytest

from doomsat.comms import (
    NodeID,
    MessageType,
    CSTPPacket,
    slip_encode,
    slip_decode,
    CSTP_MAX_PAYLOAD,
    LoRaConfig,
    CodingRate,
    compute_airtime,
    compute_receiver_sensitivity,
    compute_fspl,
    compute_atmospheric_loss,
    compute_doppler_shift,
    compute_ber_per,
    evaluate_link_budget,
    ESP32Controller,
    SimulatedSX1276Driver,
    PhysicalESP32SerialDriver,
)


def test_slip_framing_and_byte_stuffing():
    """Verify SLIP framing delimiters, byte stuffing, and roundtrip decoding."""
    # Data containing both 0xC0 (delimiter) and 0xDB (escape byte)
    raw_data = bytes([0x10, 0xC0, 0x20, 0xDB, 0x30, 0xC0, 0x40])

    encoded = slip_encode(raw_data)
    # Must start and end with 0xC0
    assert encoded[0] == 0xC0
    assert encoded[-1] == 0xC0
    # Must not contain unescaped 0xC0 internally
    assert 0xC0 not in encoded[1:-1]

    # Decode must recover exact original byte sequence
    decoded = slip_decode(encoded)
    assert decoded == raw_data


def test_slip_decode_malformed_frame():
    """Verify malformed SLIP frame rejection."""
    # Trailing isolated escape character 0xDB without escaped byte
    malformed = bytes([0xC0, 0x01, 0x02, 0xDB, 0xC0])
    with pytest.raises(ValueError):
        slip_decode(malformed)


def test_cstp_packet_pack_unpack_and_crc():
    """Verify CSTP packet serialization, CRC-16 calculation, and deserialization."""
    payload = b"DOOMSat Telemetry Beacon 001"
    packet = CSTPPacket(
        src_id=NodeID.SPACECRAFT_TTC,
        dst_id=NodeID.GROUND_STATION,
        msg_type=MessageType.TELEMETRY_BEACON,
        sequence_num=42,
        payload=payload,
    )

    packed = packet.pack()
    # Header (10 bytes) + payload (28 bytes) + CRC (2 bytes) = 40 bytes
    assert len(packed) == 10 + len(payload) + 2

    # Unpack
    unpacked = CSTPPacket.unpack(packed)
    assert unpacked.src_id == NodeID.SPACECRAFT_TTC
    assert unpacked.dst_id == NodeID.GROUND_STATION
    assert unpacked.msg_type == MessageType.TELEMETRY_BEACON
    assert unpacked.sequence_num == 42
    assert unpacked.payload == payload

    # Test CRC corruption detection: alter one byte
    corrupted = bytearray(packed)
    corrupted[15] ^= 0xFF
    with pytest.raises(ValueError) as exc:
        CSTPPacket.unpack(bytes(corrupted))
    assert "CRC-16 mismatch" in str(exc.value)


def test_cstp_mtu_rejection():
    """Verify that payloads exceeding 220 bytes are rejected."""
    oversized_payload = b"A" * (CSTP_MAX_PAYLOAD + 1)
    with pytest.raises(ValueError):
        CSTPPacket(
            src_id=NodeID.OBC,
            dst_id=NodeID.GROUND_STATION,
            msg_type=MessageType.IMAGE_CHUNK,
            sequence_num=1,
            payload=oversized_payload,
        )


def test_cstp_slip_roundtrip():
    """Verify end-to-end CSTPPacket <-> SLIP frame conversion."""
    packet = CSTPPacket(
        src_id=NodeID.OBC,
        dst_id=NodeID.GROUND_STATION,
        msg_type=MessageType.IMAGE_CHUNK,
        sequence_num=105,
        payload=bytes([0xC0, 0xDB, 0x01, 0x02, 0xC0]),
    )

    slip_frame = packet.to_slip_frame()
    recovered = CSTPPacket.from_slip_frame(slip_frame)

    assert recovered.sequence_num == 105
    assert recovered.payload == packet.payload


def test_sx1276_airtime_calculation():
    """Verify exact Semtech SX1276 LoRa on-air duration formulas."""
    config = LoRaConfig(
        frequency_hz=436.5e6,
        bandwidth_hz=125_000.0,
        spreading_factor=9,
        coding_rate=CodingRate.CR_4_5,
        preamble_symbols=8,
        enable_crc=True,
        explicit_header=True,
    )

    # Symbol duration for SF9, BW=125kHz: 2^9 / 125000 = 512 / 125000 = 4.096 ms
    res = compute_airtime(payload_bytes_len=50, config=config)
    assert math.isclose(res.symbol_duration_s, 0.004096, rel_tol=1e-6)

    # Preamble duration: (8 + 4.25) * 4.096 ms = 50.176 ms
    assert math.isclose(res.preamble_duration_s, 0.050176, rel_tol=1e-5)

    # Total airtime should be positive and reasonable (~300ms - 350ms for 50 bytes SF9)
    assert 0.200 < res.total_airtime_s < 0.400
    assert not res.low_data_rate_opt_active  # Ts = 4.096ms < 16.38ms


def test_low_data_rate_optimization_trigger():
    """Verify LDRO automatically activates when symbol duration exceeds 16.38 ms."""
    config_sf12 = LoRaConfig(
        bandwidth_hz=125_000.0,
        spreading_factor=12,  # Ts = 2^12 / 125000 = 4096 / 125000 = 32.768 ms > 16.38 ms
    )
    res = compute_airtime(payload_bytes_len=20, config=config_sf12)
    assert res.symbol_duration_s > 0.01638
    assert res.low_data_rate_opt_active


def test_receiver_sensitivity_thresholds():
    """Verify SX1276 receiver sensitivity formula against datasheet specs."""
    # SF9, BW=125kHz: -174 + 10*log10(125000) + 6 + (-12.5) = -174 + 50.97 + 6 - 12.5 = -129.5 dBm
    sens_sf9 = compute_receiver_sensitivity(bandwidth_hz=125_000.0, spreading_factor=9)
    assert -131.0 < sens_sf9 < -128.0

    # SF12, BW=125kHz: -174 + 50.97 + 6 - 20.0 = -137.0 dBm
    sens_sf12 = compute_receiver_sensitivity(bandwidth_hz=125_000.0, spreading_factor=12)
    assert -138.5 < sens_sf12 < -135.5


def test_fspl_and_doppler_shift():
    """Verify Free Space Path Loss and Doppler shift equations."""
    # 500 km at 436.5 MHz
    slant_range_m = 500_000.0
    fc = 436_500_000.0

    fspl = compute_fspl(slant_range_m, fc)
    # FSPL ~ 139.2 dB
    assert 138.0 < fspl < 141.0

    # Doppler: approaching at 7000 m/s (v_radial = -7000 m/s) -> positive shift
    doppler = compute_doppler_shift(-7000.0, fc)
    # Delta f = - (-7000 / 299792458) * 436.5e6 = +10192 Hz (~ +10.19 kHz)
    assert 10000.0 < doppler < 10400.0


def test_link_budget_evaluation():
    """Verify complete link budget evaluation at zenith pass."""
    config = LoRaConfig(
        frequency_hz=436.5e6,
        bandwidth_hz=125_000.0,
        spreading_factor=9,
        tx_power_dbm=17.0,
    )

    # Satellite directly overhead (elevation = 90 deg, slant range = 500 km, v_radial = 0)
    lb = evaluate_link_budget(
        slant_range_m=500_000.0,
        elevation_rad=math.radians(90.0),
        radial_velocity_mps=0.0,
        radio_config=config,
        tx_antenna_gain_dbi=2.15,
        rx_antenna_gain_dbi=12.0,
    )

    assert lb.is_link_viable
    assert lb.link_margin_db > 6.0  # Must exceed REQ-LNK-03 6 dB margin
    assert lb.bit_error_rate < 1e-6
    assert lb.packet_error_rate < 1e-4


def test_esp32_controller_queue_and_arq():
    """Verify ESP32 controller packet queuing, on-air timing, and ARQ ACK handling."""
    config = LoRaConfig(spreading_factor=7, bandwidth_hz=250_000.0)  # Faster airtime for test
    controller = ESP32Controller(radio_config=config, arq_timeout_s=5.0)

    # Queue an image chunk packet
    pkt = CSTPPacket(
        src_id=NodeID.SPACECRAFT_TTC,
        dst_id=NodeID.GROUND_STATION,
        msg_type=MessageType.IMAGE_CHUNK,
        sequence_num=77,
        payload=b"ImageChunkTestData",
    )
    controller.queue_downlink_packet(pkt)
    assert len(controller.downlink_queue) == 1

    # Step at t = 0s triggers transmission
    outgoing_bytes, pulse = controller.step(current_time_s=0.0)
    assert outgoing_bytes is not None
    assert controller.is_transmitting
    assert 77 in controller.unacked_packets

    # Ground station transmits ACK packet back
    # ACK payload: seq_num (u16) + status (0x06)
    ack_payload = bytes([0x00, 77, 0x06])
    ack_pkt = CSTPPacket(
        src_id=NodeID.GROUND_STATION,
        dst_id=NodeID.SPACECRAFT_TTC,
        msg_type=MessageType.CHUNK_ACK_NACK,
        sequence_num=1,
        payload=ack_payload,
    )
    controller.process_incoming_rf_packet(ack_pkt.pack(), current_time_s=1.0)

    # Unacked record should be cleared upon ACK
    assert 77 not in controller.unacked_packets


def test_radio_drivers_simulation_and_physical():
    """Verify RadioDriver abstraction for simulated and physical drivers."""
    config = LoRaConfig()
    sim_driver = SimulatedSX1276Driver(radio_config=config)

    assert sim_driver.get_status()["mode"] == "SIMULATION"
    # Transmit on default benchtop conditions
    success = sim_driver.send_packet(b"TestPacket")
    assert success
    assert sim_driver.packets_sent == 1

    # Inject and receive
    sim_driver.inject_rx_packet(b"RxPacket")
    rx = sim_driver.receive_packet()
    assert rx == b"RxPacket"

    # Physical serial driver offline handling
    phys_driver = PhysicalESP32SerialDriver(port="COM999", timeout_s=0.01)
    assert phys_driver.get_status()["mode"] == "PHYSICAL_SERIAL"
    assert not phys_driver.is_connected
    # Should safely fail without raising an unhandled exception
    assert not phys_driver.send_packet(b"Offline")
    assert phys_driver.receive_packet() is None
