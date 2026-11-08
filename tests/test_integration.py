"""Comprehensive end-to-end mission integration tests for DOOMSat 1U CubeSat Digital Twin."""

import math
import io
import numpy as np
import pytest
from PIL import Image

from doomsat.sim.orchestrator import MissionOrchestrator, SimulationConfig, SimulationSnapshot
from doomsat.sim.hil_runner import HILBenchtopRunner
from doomsat.comms.sx1276 import LoRaConfig
from doomsat.mechanical.assembly import build_reference_doomsat_1u
from doomsat.eps.battery import LiIonBattery
from doomsat.comms.drivers import SimulatedSX1276Driver, PhysicalESP32SerialDriver, RadioDriver
from doomsat.comms.cstp import CSTPPacket, MessageType, NodeID, slip_encode
from doomsat.core.constants import RF_BAND_WPC_INDIA_DELICENSED_HZ
from doomsat.payload.fragmenter import fragment_image


def test_orchestrator_initialization_and_coupling():
    """Verify master orchestrator initializes all multi-physics subsystems and advances state."""
    cfg = SimulationConfig(duration_s=10.0, dt_s=1.0)
    orchestrator = MissionOrchestrator(cfg)

    # 1. Verify subsystem coupling
    assert orchestrator.total_mass_kg == 0.746
    assert orchestrator.inertia_tensor.shape == (3, 3)
    assert orchestrator.battery.soc_pct == 95.0
    assert len(orchestrator.thermal.temperatures) == 8
    assert len(orchestrator.solar_array.panels) == 5

    # 2. Advance 5 steps
    for _ in range(5):
        snap = orchestrator.step()
        assert isinstance(snap, SimulationSnapshot)

    assert orchestrator.current_time_s == 5.0
    assert snap.altitude_km > 490.0
    assert snap.battery_soc_pct <= 95.0


def test_full_orbit_simulation_lifecycle():
    """Execute complete 95-minute orbit (5700s) and verify B-dot detumbling, EPS margin, and thermal limits."""
    # Run with dt = 5.0s (1140 simulation steps)
    cfg = SimulationConfig(duration_s=5700.0, dt_s=5.0)
    orchestrator = MissionOrchestrator(cfg)

    initial_tumble_deg_s = float(np.linalg.norm(cfg.initial_tumble_deg_s))
    assert initial_tumble_deg_s > 20.0  # High initial tumble rate (~21.7 deg/s)

    snapshots = orchestrator.run()
    assert len(snapshots) == 1140

    # 1. B-dot Detumbling Verification
    # After detumble phase, angular rate must drop to < 0.5 deg/s
    final_snap = snapshots[-1]
    assert final_snap.omega_norm_deg_s < 0.5, f"Tumble rate not damped: {final_snap.omega_norm_deg_s} deg/s"

    # 2. Battery State of Charge & Energy Balance
    # Battery must end orbit with >= 80% SoC and never trip LVD (20%)
    soc_history = [s.battery_soc_pct for s in snapshots]
    min_soc = min(soc_history)
    final_soc = final_snap.battery_soc_pct
    assert min_soc > 50.0, f"Battery drained excessively: min SoC = {min_soc}%"
    assert final_soc >= 80.0, f"Insufficient energy generation: final SoC = {final_soc}%"
    assert not orchestrator.power_bus.is_lvd_tripped

    # 3. Thermal Envelope Verification
    # Battery operating bounds: >= -20°C in eclipse, <= 45°C in sunlight
    bat_temps = [s.temp_battery_c for s in snapshots]
    elec_temps = [s.temp_electronics_c for s in snapshots]

    assert min(bat_temps) >= -20.0, f"Battery fell below -20°C: {min(bat_temps)}°C"
    assert max(bat_temps) <= 45.0, f"Battery exceeded 45°C: {max(bat_temps)}°C"
    assert min(elec_temps) >= -20.0, f"Electronics fell below -20°C: {min(elec_temps)}°C"
    assert max(elec_temps) <= 70.0, f"Electronics exceeded 70°C: {max(elec_temps)}°C"


def test_ground_station_pass_and_image_downlink():
    """Simulate ground pass over Hyderabad with image downlink, packet drop, selective ARQ, and reassembly."""
    cfg = SimulationConfig(duration_s=100.0, dt_s=1.0)
    orchestrator = MissionOrchestrator(cfg)

    # 1. Create synthetic test JPEG image
    img = Image.new("RGB", (64, 64), color=(0, 200, 100))
    buf = io.BytesIO()
    img.save(buf, format="JPEG", quality=80)
    orig_jpeg = buf.getvalue()

    desc, chunks = fragment_image(
        image_id=999,
        image_bytes=orig_jpeg,
        width=64,
        height=64,
        gsd_m=194.4,
        capture_time_s=100.0,
        latitude_rad=0.30,
        longitude_rad=1.37,
        chunk_payload_size=100,
    )
    assert len(chunks) >= 3

    # 2. Register descriptor with ground station
    orchestrator.image_manager.register_descriptor(desc)

    # 3. Transmit chunks, dropping chunk 1 to trigger ARQ NACK
    for idx, c in enumerate(chunks):
        if idx == 1:
            continue  # Dropped frame in RF channel
        pkt = CSTPPacket(
            src_id=NodeID.PAYLOAD,
            dst_id=NodeID.GROUND_STATION,
            msg_type=MessageType.IMAGE_CHUNK,
            sequence_num=c.chunk_index,
            payload=c.pack(),
        )
        frame = slip_encode(pkt.pack())
        orchestrator.receiver.process_raw_frame(frame, rssi_dbm=-110.0, snr_db=8.0)

    # Image should be incomplete
    assert not orchestrator.image_manager.get_image(999)

    # 4. Generate selective NACK from Ground Station
    nack = orchestrator.image_manager.generate_selective_nack_packet(999)
    assert nack is not None
    assert nack.msg_type == MessageType.CHUNK_ACK_NACK

    # 5. Retransmit missing chunk 1 (ARQ fulfillment)
    missing_chunk = chunks[1]
    retransmit_pkt = CSTPPacket(
        src_id=NodeID.PAYLOAD,
        dst_id=NodeID.GROUND_STATION,
        msg_type=MessageType.IMAGE_CHUNK,
        sequence_num=missing_chunk.chunk_index,
        payload=missing_chunk.pack(),
    )
    retransmit_frame = slip_encode(retransmit_pkt.pack())
    orchestrator.receiver.process_raw_frame(retransmit_frame, rssi_dbm=-108.0, snr_db=9.0)

    # 6. Verify 100% byte-accurate image reconstruction
    rec = orchestrator.image_manager.get_image(999)
    assert rec is not None
    assert rec.raw_jpeg_bytes == orig_jpeg
    assert rec.base64_thumbnail.startswith("data:image/jpeg;base64,")


def test_hil_driver_interchangeability():
    """Verify Hardware-in-the-Loop driver polymorphism and Indian WPC RF regulatory compliance."""
    # 1. Polymorphism verification: both drivers implement RadioDriver
    sim_driver = SimulatedSX1276Driver(radio_config=LoRaConfig())
    assert isinstance(sim_driver, RadioDriver)

    # 2. Physical driver interface compliance
    assert issubclass(PhysicalESP32SerialDriver, RadioDriver)

    # 3. Regulatory compliance: terrestrial HIL default frequency must match WPC delicensed band (865.5 MHz)
    runner = HILBenchtopRunner()
    assert runner.frequency_hz == RF_BAND_WPC_INDIA_DELICENSED_HZ
    assert runner.frequency_hz == 865_500_000.0

    # 4. Available ports querying should execute without raising
    ports = runner.list_available_ports()
    assert isinstance(ports, list)


def test_energy_and_mass_conservation():
    """Verify physical consistency: mass within 1.33 kg spec, CoG centered, energy balance valid."""
    assembly = build_reference_doomsat_1u()

    # 1. Mass margin
    assert assembly.total_mass_kg <= 1.330
    assert assembly.mass_margin_pct > 40.0

    # 2. CoG centering
    is_cog_valid, offset = assembly.verify_cog_limit(max_offset_m=0.020)
    assert is_cog_valid
    assert offset < 0.005  # Within 5 mm of chassis geometric center

    # 3. Battery Coulomb capacity
    batt = LiIonBattery(nominal_capacity_mah=2500.0)
    assert math.isclose(batt.capacity_coulombs, 9000.0)
