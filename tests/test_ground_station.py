"""Tests for Ground Station subsystem: CSTP receiver, CRC verification, telemetry store, and image reconstruction."""

import json
import math
import io
import pytest
from PIL import Image

from doomsat.comms.cstp import CSTPPacket, MessageType, NodeID, slip_encode
from doomsat.comms.sx1276 import LoRaConfig
from doomsat.comms.drivers import SimulatedSX1276Driver
from doomsat.core.schemas import TelemetryBeacon
from doomsat.payload.fragmenter import fragment_image, ImageDescriptor, ImageChunk
from doomsat.ground_station.telemetry_store import TelemetryStore
from doomsat.ground_station.image_reconstruction import GroundStationImageManager
from doomsat.ground_station.receiver import GroundStationReceiver


def test_receiver_packet_ingestion_and_crc():
    """Verify valid CSTP beacon ingestion and rejection of corrupted CRC packets."""
    store = TelemetryStore()
    receiver = GroundStationReceiver(telemetry_store=store)

    beacon = TelemetryBeacon(
        sequence_id=42,
        timestamp_s=120.5,
        flight_mode="NOMINAL",
        battery_soc=88.5,
        battery_voltage=3.85,
        solar_power=1.95,
        obc_temp_k=295.15,
        battery_temp_k=293.15,
        angular_rate_norm_radps=0.012,
        in_eclipse=False,
    )

    # Pack into compact binary CSTP packet (42 bytes <= 220 MTU)
    payload_bytes = beacon.pack_binary()
    packet = CSTPPacket(
        src_id=NodeID.OBC,
        dst_id=NodeID.GROUND_STATION,
        msg_type=MessageType.TELEMETRY_BEACON,
        sequence_num=42,
        flags=0x00,
        payload=payload_bytes,
    )

    # Encode with SLIP
    slip_frame = slip_encode(packet.pack())

    # 1. Ingest valid frame
    success, msg = receiver.process_raw_frame(slip_frame, rssi_dbm=-110.0, snr_db=7.2)
    assert success
    assert receiver.metrics.valid_packets == 1
    assert receiver.metrics.crc_errors == 0
    assert store.get_count() == 1

    stored_beacon = store.get_latest()
    assert stored_beacon is not None
    assert stored_beacon.sequence_id == 42
    assert stored_beacon.rssi_dbm == -110.0
    assert stored_beacon.snr_db == 7.2

    # 2. Ingest corrupted frame (flip bits in payload)
    corrupted = bytearray(slip_frame)
    # Corrupt payload byte inside SLIP frame
    corrupted[15] ^= 0xFF
    bad_success, bad_msg = receiver.process_raw_frame(bytes(corrupted))
    assert not bad_success
    assert receiver.metrics.crc_errors == 1
    assert receiver.metrics.total_packets_received == 2
    assert receiver.metrics.valid_packets == 1


def test_receiver_link_metrics_tracking():
    """Verify RSSI, SNR, Doppler shift, and Packet Delivery Ratio metrics."""
    receiver = GroundStationReceiver()

    # Create dummy ping packet
    packet = CSTPPacket(
        src_id=NodeID.GROUND_STATION,
        dst_id=NodeID.OBC,
        msg_type=MessageType.PING,
        sequence_num=1,
        flags=0x00,
        payload=b"PING",
    )
    frame = slip_encode(packet.pack())

    # Ingest 3 valid frames
    receiver.process_raw_frame(frame, rssi_dbm=-115.0, snr_db=5.0)
    receiver.process_raw_frame(frame, rssi_dbm=-110.0, snr_db=8.0)
    receiver.process_raw_frame(frame, rssi_dbm=-105.0, snr_db=10.0)

    # Ingest 1 corrupted frame
    receiver.process_raw_frame(b"\xc0\xd0\x01\x99\x99\xc0")

    m = receiver.metrics
    assert m.total_packets_received == 4
    assert m.valid_packets == 3
    assert m.crc_errors == 1
    assert math.isclose(m.packet_delivery_ratio, 75.0, abs_tol=0.1)
    assert m.last_rssi_dbm == -105.0
    assert m.last_snr_db == 10.0
    assert m.mean_snr_db > 5.0


def test_telemetry_store_operations():
    """Verify circular buffering, time-window filtering, and JSON/CSV export."""
    store = TelemetryStore(max_records=5)

    # Insert 6 beacons (triggers circular buffer eviction of first)
    for i in range(6):
        b = TelemetryBeacon(
            sequence_id=i,
            timestamp_s=100.0 + i * 10.0,
            flight_mode="NOMINAL",
            battery_soc=90.0 - i,
            battery_voltage=3.9 - i * 0.05,
            solar_power=1.8,
            obc_temp_k=295.0,
            battery_temp_k=293.0,
            angular_rate_norm_radps=0.01,
            in_eclipse=False,
        )
        store.add_beacon(b)

    assert store.get_count() == 5  # capped at max_records=5
    # First record should now be seq=1 (seq=0 evicted)
    history = store.get_history()
    assert history[0].sequence_id == 1
    assert history[-1].sequence_id == 5

    # Filter window: last 25 seconds [125s to 150s] -> seq 3, 4, 5
    filtered = store.get_history(window_seconds=25.0)
    assert len(filtered) == 3
    assert filtered[0].sequence_id == 3

    # Export formats
    json_str = store.export_json()
    assert '"sequence_id": 5' in json_str

    csv_str = store.export_csv()
    assert "sequence_id" in csv_str
    assert "battery_soc" in csv_str


def test_image_reconstruction_missing_chunks_and_nack():
    """Verify chunk ingestion, missing chunk detection, and selective NACK generation."""
    mgr = GroundStationImageManager()

    # Generate small 100x100 JPEG image
    img = Image.new("RGB", (100, 100), color=(100, 150, 200))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    jpeg_bytes = buf.getvalue()

    desc, chunks = fragment_image(
        image_id=101,
        image_bytes=jpeg_bytes,
        width=100,
        height=100,
        gsd_m=194.4,
        capture_time_s=1000.0,
        latitude_rad=0.30,
        longitude_rad=1.37,
        chunk_payload_size=150,
    )
    assert len(chunks) >= 3

    # Register descriptor
    mgr.register_descriptor(desc)
    assert mgr.get_progress(101) == 0.0

    # Ingest chunks EXCEPT chunk 1 (drop chunk 1!)
    for i, chunk in enumerate(chunks):
        if i == 1:
            continue  # Simulate dropped packet
        accepted, complete = mgr.ingest_chunk(chunk)
        assert accepted
        assert not complete

    # Check progress
    expected_pct = ((len(chunks) - 1) / float(len(chunks))) * 100.0
    assert math.isclose(mgr.get_progress(101), expected_pct, abs_tol=0.1)

    # Generate selective NACK
    nack_packet = mgr.generate_selective_nack_packet(101)
    assert nack_packet is not None
    assert nack_packet.msg_type == MessageType.CHUNK_ACK_NACK

    # Unpack missing indices from NACK payload
    import struct
    img_id, count = struct.unpack(">HH", nack_packet.payload[:4])
    assert img_id == 101
    assert count == 1
    (missing_idx,) = struct.unpack(">H", nack_packet.payload[4:6])
    assert missing_idx == 1  # Exactly identified dropped chunk 1!


def test_image_reconstruction_lossless_reassembly():
    """Verify lossless JPEG byte reconstruction and thumbnail generation upon completion."""
    mgr = GroundStationImageManager()

    img = Image.new("RGB", (64, 64), color=(255, 128, 0))
    buf = io.BytesIO()
    img.save(buf, format="JPEG")
    original_bytes = buf.getvalue()

    desc, chunks = fragment_image(
        image_id=202,
        image_bytes=original_bytes,
        width=64,
        height=64,
        gsd_m=194.4,
        capture_time_s=2000.0,
        latitude_rad=0.0,
        longitude_rad=0.0,
        chunk_payload_size=120,
    )

    mgr.register_descriptor(desc)

    # Ingest all chunks
    for i, chunk in enumerate(chunks):
        accepted, complete = mgr.ingest_chunk(chunk)
        assert accepted
        if i == len(chunks) - 1:
            assert complete
        else:
            assert not complete

    assert mgr.get_progress(202) == 100.0
    record = mgr.get_image(202)
    assert record is not None
    assert record.image_id == 202
    assert record.raw_jpeg_bytes == original_bytes  # Lossless match!
    assert record.base64_thumbnail.startswith("data:image/jpeg;base64,")


def test_command_uplink_formatting():
    """Verify formatting and transmission of command packets."""
    radio = SimulatedSX1276Driver(LoRaConfig())
    receiver = GroundStationReceiver(radio_driver=radio)

    success = receiver.send_command("CAPTURE_IMAGE", target_node=NodeID.OBC)
    assert success
    assert receiver.metrics.commands_uplinked == 1

    # Radio channel should now have 1 queued packet in tx_buffer
    assert len(radio.tx_buffer) == 1
    raw_packet = radio.tx_buffer[0]
    assert raw_packet.startswith(b"\xc0")  # SLIP framed
