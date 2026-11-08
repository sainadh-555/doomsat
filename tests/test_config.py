"""Unit tests for configuration loading, Pydantic validation, and physical bounds."""

import pytest
from pydantic import ValidationError
from doomsat.config.loader import load_config, DOOMSatConfig


def test_load_default_config(default_config: DOOMSatConfig):
    """Verify that default YAML configuration loads and validates successfully."""
    assert default_config.mission.name in ("GARUDA-1", "DOOMSat-1")
    assert default_config.mission.orbit.altitude_m == 500000.0
    assert default_config.mission.orbit.inclination_deg == 97.4
    assert default_config.mechanical.mass_kg <= 1.330
    assert default_config.radio.frequency_hz == 436500000.0
    assert default_config.eps.battery.nominal_capacity_mah == 2500.0
    assert default_config.thermal.initial_temperatures_k.electronics == 295.0


def test_mass_limit_violation(default_config: DOOMSatConfig):
    """Verify that a mass exceeding 1.330 kg triggers validation rejection."""
    raw_dict = default_config.model_dump()
    raw_dict["mechanical"]["mass_kg"] = 1.35  # Over limit

    with pytest.raises(ValidationError) as exc_info:
        DOOMSatConfig.model_validate(raw_dict)
    assert "1.33" in str(exc_info.value)


def test_radio_regulatory_profile_enforcement(default_config: DOOMSatConfig):
    """Verify that frequency must match the selected regulatory profile."""
    raw_dict = default_config.model_dump()

    # Case 1: Indian WPC delicensed profile with non-delicensed frequency (e.g. 436.5 MHz)
    raw_dict["radio"]["band_profile"] = "wpc_india_delicensed"
    raw_dict["radio"]["frequency_hz"] = 436500000.0

    with pytest.raises(ValidationError) as exc_info:
        DOOMSatConfig.model_validate(raw_dict)
    assert "violates Indian WPC delicensed band" in str(exc_info.value)

    # Case 2: Legal Indian WPC frequency (865.5 MHz) should succeed
    raw_dict["radio"]["frequency_hz"] = 865500000.0
    valid_hil_config = DOOMSatConfig.model_validate(raw_dict)
    assert valid_hil_config.radio.frequency_hz == 865500000.0


def test_invalid_orbit_altitude(default_config: DOOMSatConfig):
    """Verify that an orbit altitude below 150 km (rapid re-entry) is rejected."""
    raw_dict = default_config.model_dump()
    raw_dict["mission"]["orbit"]["altitude_m"] = 50000.0  # 50 km

    with pytest.raises(ValidationError):
        DOOMSatConfig.model_validate(raw_dict)
