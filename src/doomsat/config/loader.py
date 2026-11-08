"""Configuration loader and schema validation for DOOMSat.

Enforces physical boundaries, unit consistency, and hardware validation
using Pydantic v2.
"""

from pathlib import Path
from typing import List, Tuple, Optional, Literal
import yaml
from pydantic import BaseModel, Field, field_validator


class OrbitConfig(BaseModel):
    altitude_m: float = Field(..., ge=150000.0, le=2000000.0, description="Altitude in LEO [m]")
    inclination_deg: float = Field(..., ge=0.0, le=180.0, description="Orbit inclination [deg]")
    eccentricity: float = Field(..., ge=0.0, lt=1.0, description="Orbit eccentricity")
    raan_deg: float = Field(..., ge=0.0, le=360.0, description="RAAN [deg]")
    arg_perigee_deg: float = Field(..., ge=0.0, le=360.0, description="Argument of perigee [deg]")
    mean_anomaly_deg: float = Field(..., ge=0.0, le=360.0, description="Initial mean anomaly [deg]")
    epoch_iso: str = Field("2026-11-08T00:00:00Z", description="Simulation epoch ISO timestamp")


class SimulationConfig(BaseModel):
    duration_s: float = Field(..., gt=0.0, description="Simulation duration [s]")
    time_step_s: float = Field(..., gt=0.0, le=10.0, description="Simulation step [s]")


class MissionConfig(BaseModel):
    name: str = Field("DOOMSat-1", description="Mission Name")
    orbit: OrbitConfig
    simulation: SimulationConfig


class GroundStationConfig(BaseModel):
    name: str = Field("Hyderabad-GS", description="Ground Station Identifier")
    latitude_deg: float = Field(..., ge=-90.0, le=90.0)
    longitude_deg: float = Field(..., ge=-180.0, le=180.0)
    altitude_m: float = Field(..., ge=-100.0, le=9000.0)
    min_elevation_deg: float = Field(10.0, ge=0.0, le=45.0)
    antenna_gain_rx_dbi: float = Field(12.0, ge=0.0, le=30.0)


class MomentsOfInertiaConfig(BaseModel):
    ixx: float = Field(..., gt=0.0)
    iyy: float = Field(..., gt=0.0)
    izz: float = Field(..., gt=0.0)
    ixy: float = Field(0.0)
    ixz: float = Field(0.0)
    iyz: float = Field(0.0)


class MechanicalConfig(BaseModel):
    mass_kg: float = Field(..., gt=0.1, le=1.33, description="Total satellite mass (<= 1.33 kg for 1U)")
    dimensions_m: Tuple[float, float, float] = Field((0.100, 0.100, 0.1135))
    moments_of_inertia_kg_m2: MomentsOfInertiaConfig

    @field_validator("mass_kg")
    @classmethod
    def validate_mass_limit(cls, v: float) -> float:
        if v > 1.33:
            raise ValueError(f"Mass {v} kg exceeds standard 1U CubeSat specification limit of 1.33 kg")
        return v


class RadioConfig(BaseModel):
    band_profile: Literal["itu_amateur_space", "wpc_india_delicensed"] = Field("itu_amateur_space")
    frequency_hz: float = Field(..., gt=100e6, lt=10e9)
    bandwidth_hz: float = Field(..., gt=0.0)
    spreading_factor: int = Field(..., ge=6, le=12)
    coding_rate: str = Field("4/5")
    tx_power_dbm: float = Field(..., ge=2.0, le=20.0)
    satellite_antenna_gain_dbi: float = Field(2.15)
    noise_figure_db: float = Field(4.0, ge=0.0)
    cstp_mtu_bytes: int = Field(220, ge=32, le=255)

    @field_validator("frequency_hz")
    def validate_frequency_regulatory(cls, v: float, info) -> float:
        # Check profile alignment
        profile = info.data.get("band_profile")
        if profile == "wpc_india_delicensed" and not (865e6 <= v <= 867e6):
            raise ValueError(f"Frequency {v/1e6:.3f} MHz violates Indian WPC delicensed band (865-867 MHz)")
        if profile == "itu_amateur_space" and not (430e6 <= v <= 440e6):
            raise ValueError(f"Frequency {v/1e6:.3f} MHz outside amateur UHF satellite band (430-440 MHz)")
        return v


class CameraPayloadConfig(BaseModel):
    sensor: str = Field("OmniVision OV5647")
    sensor_width_mm: float = Field(3.674, gt=0.0)
    sensor_height_mm: float = Field(2.760, gt=0.0)
    pixel_width: int = Field(2592, gt=0)
    pixel_height: int = Field(1944, gt=0)
    focal_length_mm: float = Field(3.6, gt=0.0)
    compression_quality_pct: int = Field(75, ge=10, le=100)


class AdcsSensorsConfig(BaseModel):
    mpu6050_gyro_noise_density_radps_sqrthz: float = Field(0.000087, ge=0.0)
    mpu6050_accel_noise_density_mps2_sqrthz: float = Field(0.0039, ge=0.0)
    lis3mdl_mag_noise_std_tesla: float = Field(3.2e-7, ge=0.0)
    photodiode_responsivity_v_per_sun: float = Field(3.0, gt=0.0)


class AdcsActuatorsConfig(BaseModel):
    magnetorquer_max_dipole_am2: float = Field(0.15, gt=0.0)


class AdcsControlConfig(BaseModel):
    bdot_gain: float = Field(2.0e5, gt=0.0)


class AdcsConfig(BaseModel):
    sensors: AdcsSensorsConfig
    actuators: AdcsActuatorsConfig
    control: AdcsControlConfig


class BatteryConfig(BaseModel):
    cell_type: str = Field("Samsung INR18650-25R")
    nominal_capacity_mah: float = Field(2500.0, gt=0.0)
    nominal_voltage_v: float = Field(3.6, gt=0.0)
    charge_voltage_limit_v: float = Field(4.2, gt=0.0)
    discharge_cutoff_v: float = Field(3.2, gt=0.0)
    initial_soc_pct: float = Field(95.0, ge=0.0, le=100.0)


class SolarConfig(BaseModel):
    cell_efficiency: float = Field(0.285, gt=0.0, le=1.0)
    panel_area_per_face_m2: float = Field(0.007, gt=0.0)
    active_faces: List[str] = Field(default_factory=lambda: ["+X", "-X", "+Y", "-Y", "+Z"])


class EpsConfig(BaseModel):
    battery: BatteryConfig
    solar: SolarConfig


class InitialThermalTemperaturesConfig(BaseModel):
    face_pos_x: float = Field(290.0, gt=0.0)
    face_neg_x: float = Field(290.0, gt=0.0)
    face_pos_y: float = Field(290.0, gt=0.0)
    face_neg_y: float = Field(290.0, gt=0.0)
    face_pos_z: float = Field(290.0, gt=0.0)
    face_neg_z: float = Field(290.0, gt=0.0)
    electronics: float = Field(295.0, gt=0.0)
    battery: float = Field(293.0, gt=0.0)


class ThermalConfig(BaseModel):
    solar_absorptivity_alpha: float = Field(0.90, ge=0.0, le=1.0)
    ir_emissivity_epsilon: float = Field(0.82, ge=0.0, le=1.0)
    internal_dissipation_idle_w: float = Field(0.8, ge=0.0)
    internal_dissipation_active_w: float = Field(2.5, ge=0.0)
    initial_temperatures_k: InitialThermalTemperaturesConfig


class DOOMSatConfig(BaseModel):
    """Complete root configuration container for the DOOMSat mission."""
    mission: MissionConfig
    ground_station: GroundStationConfig
    mechanical: MechanicalConfig
    radio: RadioConfig
    payload_camera: CameraPayloadConfig
    adcs: AdcsConfig
    eps: EpsConfig
    thermal: ThermalConfig


def load_config(config_path: Optional[Path | str] = None) -> DOOMSatConfig:
    """Load and validate DOOMSat configuration from YAML file.

    Args:
        config_path: Path to configuration YAML. If None, loads default_config.yaml.

    Returns:
        Validated DOOMSatConfig instance.
    """
    if config_path is None:
        config_path = Path(__file__).parent / "default_config.yaml"
    else:
        config_path = Path(config_path)

    if not config_path.exists():
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    with open(config_path, "r", encoding="utf-8") as f:
        raw_dict = yaml.safe_load(f)

    return DOOMSatConfig.model_validate(raw_dict)
