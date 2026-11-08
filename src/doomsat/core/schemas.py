"""Pydantic schemas and data models for DOOMSat telemetry, states, and packets.

All values are strictly typed and use SI units:
- Distances: meters (m)
- Angles: radians (rad)
- Angular rate: rad/s
- Temperature: Kelvin (K)
- Voltage: Volts (V)
- Current: Amperes (A)
- Power: Watts (W)
"""

from typing import List, Tuple, Optional
from pydantic import BaseModel, Field, field_validator


class OrbitalState(BaseModel):
    """Instantaneous orbital state of the 1U CubeSat."""
    time_s: float = Field(..., description="Simulation epoch time since mission start [s]")
    r_eci_m: Tuple[float, float, float] = Field(..., description="ECI position vector [x, y, z] in meters")
    v_eci_mps: Tuple[float, float, float] = Field(..., description="ECI velocity vector [vx, vy, vz] in m/s")
    lat_rad: float = Field(..., ge=-1.5707963268, le=1.5707963268, description="WGS-84 geodetic latitude [rad]")
    lon_rad: float = Field(..., ge=-3.1415926536, le=3.1415926536, description="WGS-84 geodetic longitude [rad]")
    alt_m: float = Field(..., ge=0.0, description="Altitude above WGS-84 ellipsoid [m]")
    in_eclipse: bool = Field(False, description="True if CubeSat is in Earth umbra/penumbra")
    eclipse_factor: float = Field(0.0, ge=0.0, le=1.0, description="1.0 in direct sunlight, 0.0 in total umbra")


class AttitudeState(BaseModel):
    """Instantaneous attitude state of the spacecraft."""
    q_body_to_eci: Tuple[float, float, float, float] = Field(
        ..., description="Scalar-first unit quaternion [qw, qx, qy, qz] representing Body relative to ECI"
    )
    omega_body_radps: Tuple[float, float, float] = Field(
        ..., description="Angular velocity vector [wx, wy, wz] in Body frame [rad/s]"
    )
    roll_rad: float = Field(..., description="Euler roll angle (phi) [rad]")
    pitch_rad: float = Field(..., description="Euler pitch angle (theta) [rad]")
    yaw_rad: float = Field(..., description="Euler yaw angle (psi) [rad]")


class SensorReadings(BaseModel):
    """Raw and calibrated sensor readings from onboard payload and ADCS sensors."""
    # MPU6050 6-DOF IMU
    gyro_radps: Tuple[float, float, float] = Field(..., description="Calibrated 3-axis gyro rates [rad/s]")
    accel_mps2: Tuple[float, float, float] = Field(..., description="Calibrated 3-axis linear acceleration [m/s^2]")
    
    # LIS3MDL 3-Axis Magnetometer
    mag_tesla: Tuple[float, float, float] = Field(..., description="Measured magnetic field [Bx, By, Bz] in Tesla")
    
    # Sun Sensor Network (6x Vishay BPW83 photodiodes via ADS1115 ADCs)
    sun_photodiode_volts: Tuple[float, float, float, float, float, float] = Field(
        ..., description="Analog voltages from photodiodes [+X, -X, +Y, -Y, +Z, -Z] in Volts"
    )
    
    # BME280 Environmental Health Sensor
    internal_temp_k: float = Field(..., description="Internal cavity temperature [K]")
    internal_pressure_pa: float = Field(..., description="Internal pressure [Pa]")
    internal_humidity_pct: float = Field(..., ge=0.0, le=100.0, description="Internal relative humidity [%]")


class PowerState(BaseModel):
    """Electrical Power Subsystem (EPS) state and battery metrics."""
    battery_soc_pct: float = Field(..., ge=0.0, le=100.0, description="Battery State of Charge [%]")
    battery_voltage_v: float = Field(..., ge=2.5, le=4.35, description="Battery terminal voltage [V]")
    battery_current_a: float = Field(..., description="Battery net current [A] (positive = charging, negative = discharging)")
    solar_power_w: float = Field(..., ge=0.0, description="Total instantaneous solar power generation [W]")
    bus_3v3_v: float = Field(3.3, description="Regulated 3.3V bus actual voltage [V]")
    bus_5v0_v: float = Field(5.0, description="Regulated 5.0V bus actual voltage [V]")
    system_load_w: float = Field(..., ge=0.0, description="Total electrical load power drawn [W]")


class ThermalState(BaseModel):
    """Multi-node lumped thermal state of the CubeSat structure and avionics."""
    temp_face_pos_x_k: float = Field(..., description="External +X face temperature [K]")
    temp_face_neg_x_k: float = Field(..., description="External -X face temperature [K]")
    temp_face_pos_y_k: float = Field(..., description="External +Y face temperature [K]")
    temp_face_neg_y_k: float = Field(..., description="External -Y face temperature [K]")
    temp_face_pos_z_k: float = Field(..., description="External +Z face temperature [K]")
    temp_face_neg_z_k: float = Field(..., description="External -Z face temperature [K]")
    temp_electronics_k: float = Field(..., description="Internal electronics stack (OBC/ESP32) temperature [K]")
    temp_battery_k: float = Field(..., description="Battery cell temperature [K]")


class TelemetryBeacon(BaseModel):
    """Housekeeping telemetry beacon packet transmitted over LoRa RF link."""
    sync_bytes: str = Field("D001", description="CSTP sync marker")
    sequence_id: int = Field(..., ge=0, description="Incrementing 16-bit packet sequence number")
    timestamp_s: float = Field(..., description="Mission elapsed time [s]")
    flight_mode: str = Field(..., description="Current OBC flight state mode")
    battery_soc: float = Field(..., description="Battery State of Charge [%]")
    battery_voltage: float = Field(..., description="Battery voltage [V]")
    solar_power: float = Field(..., description="Generated solar power [W]")
    obc_temp_k: float = Field(..., description="OBC temperature [K]")
    battery_temp_k: float = Field(..., description="Battery temperature [K]")
    angular_rate_norm_radps: float = Field(..., description="Norm of angular rate vector [rad/s]")
    in_eclipse: bool = Field(..., description="True if satellite is in eclipse")
    rssi_dbm: Optional[float] = Field(None, description="Ground station received RSSI [dBm]")
    snr_db: Optional[float] = Field(None, description="Ground station received SNR [dB]")
