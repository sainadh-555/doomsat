"""Pytest fixtures for DOOMSat unit testing."""

import pytest
import numpy as np
from doomsat.config.loader import load_config, DOOMSatConfig


@pytest.fixture
def default_config() -> DOOMSatConfig:
    """Fixture returning the parsed and validated default mission configuration."""
    return load_config()


@pytest.fixture
def sample_orbit_state() -> dict:
    """Fixture returning canonical LEO orbital state vectors for testing."""
    return {
        "r_eci": np.array([6878137.0, 0.0, 0.0], dtype=np.float64),  # 500 km equatorial
        "v_eci": np.array([0.0, 7612.6, 0.0], dtype=np.float64),     # Circular speed ~7.61 km/s
        "gmst_rad": 0.5235987755982988                                # 30 degrees
    }
