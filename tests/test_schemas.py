"""Unit tests for Pydantic schemas and telemetry models."""

import pytest
from pydantic import ValidationError
from doomsat.core.schemas import (
    OrbitalState,
    AttitudeState,
    SensorReadings,
    PowerState,
    ThermalState,
    TelemetryBeacon,
)


def test_orbital_state_valid():
    """Verify that a valid OrbitalState model instantiates correctly."""
    state = OrbitalState(
        time_s=120.0,
        r_eci_m=(6878137.0, 0.0, 0.0),
        v_eci_mps=(0.0, 7612.0, 0.0),
        lat_rad=0.3034,
        lon_rad=1.3698,
        alt_m=500120.0,
        in_eclipse=False,
        eclipse_factor=1.0,
    )
    assert state.time_s == 120.0
    assert state.alt_m == 500120.0
    assert not state.in_eclipse


def test_orbital_state_latitude_bounds():
    """Verify that latitude outside [-pi/2, +pi/2] is rejected."""
    with pytest.raises(ValidationError):
        OrbitalState(
            time_s=0.0,
            r_eci_m=(7000000.0, 0.0, 0.0),
            v_eci_mps=(0.0, 7500.0, 0.0),
            lat_rad=2.0,  # > pi/2
            lon_rad=0.0,
            alt_m=500000.0,
        )


def test_attitude_state_valid():
    """Verify AttitudeState initialization."""
    att = AttitudeState(
        q_body_to_eci=(1.0, 0.0, 0.0, 0.0),
        omega_body_radps=(0.01, -0.02, 0.005),
        roll_rad=0.0,
        pitch_rad=0.0,
        yaw_rad=0.0,
    )
    assert att.q_body_to_eci[0] == 1.0
    assert att.omega_body_radps[2] == 0.005


def test_sensor_readings_valid():
    """Verify SensorReadings contains all required hardware telemetry."""
    sensors = SensorReadings(
        gyro_radps=(0.001, 0.002, -0.001),
        accel_mps2=(0.0, 0.0, -9.81),
        mag_tesla=(2.5e-5, -1.2e-5, 4.1e-5),
        sun_photodiode_volts=(2.8, 0.0, 1.2, 0.0, 0.0, 0.0),
        internal_temp_k=296.5,
        internal_pressure_pa=101325.0,
        internal_humidity_pct=42.5,
    )
    assert sensors.gyro_radps[0] == 0.001
    assert sensors.internal_temp_k == 296.5


def test_power_state_bounds():
    """Verify PowerState validates battery SoC and voltage constraints."""
    power = PowerState(
        battery_soc_pct=88.5,
        battery_voltage_v=3.92,
        battery_current_a=0.45,
        solar_power_w=2.1,
        bus_3v3_v=3.305,
        bus_5v0_v=5.012,
        system_load_w=1.2,
    )
    assert power.battery_soc_pct == 88.5
    assert power.battery_voltage_v == 3.92

    # Negative solar power should be rejected
    with pytest.raises(ValidationError):
        PowerState(
            battery_soc_pct=88.5,
            battery_voltage_v=3.92,
            battery_current_a=0.45,
            solar_power_w=-1.0,  # Invalid
            bus_3v3_v=3.3,
            bus_5v0_v=5.0,
            system_load_w=1.2,
        )


def test_thermal_state_valid():
    """Verify ThermalState validates temperatures in Kelvin."""
    thermal = ThermalState(
        temp_face_pos_x_k=295.0,
        temp_face_neg_x_k=285.0,
        temp_face_pos_y_k=290.0,
        temp_face_neg_y_k=290.0,
        temp_face_pos_z_k=300.0,
        temp_face_neg_z_k=270.0,
        temp_electronics_k=298.0,
        temp_battery_k=294.0,
    )
    assert thermal.temp_battery_k == 294.0


def test_telemetry_beacon_json_serialization():
    """Verify TelemetryBeacon serializes to JSON and recovers cleanly."""
    beacon = TelemetryBeacon(
        sequence_id=42,
        timestamp_s=420.5,
        flight_mode="NOMINAL",
        battery_soc=92.3,
        battery_voltage=4.01,
        solar_power=1.85,
        obc_temp_k=296.2,
        battery_temp_k=293.8,
        angular_rate_norm_radps=0.002,
        in_eclipse=False,
        rssi_dbm=-88.0,
        snr_db=8.5,
    )

    json_str = beacon.model_dump_json()
    assert "NOMINAL" in json_str
    assert "D001" in json_str

    recovered = TelemetryBeacon.model_validate_json(json_str)
    assert recovered.sequence_id == 42
    assert recovered.rssi_dbm == -88.0
