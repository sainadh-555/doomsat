"""Attitude Determination and Control Subsystem (ADCS) package for DOOMSat."""

from doomsat.adcs.dynamics import (
    compute_omega_matrix,
    compute_gravity_gradient_torque,
    compute_magnetic_disturbance_torque,
    compute_aerodynamic_torque,
    compute_srp_torque,
    RigidBodyAttitudeDynamics,
)
from doomsat.adcs.sensors import (
    MPU6050IMU,
    LIS3MDLMagnetometer,
    SunSensorArray,
)
from doomsat.adcs.control import (
    BDotController,
    TRIADEstimator,
)

__all__ = [
    "compute_omega_matrix",
    "compute_gravity_gradient_torque",
    "compute_magnetic_disturbance_torque",
    "compute_aerodynamic_torque",
    "compute_srp_torque",
    "RigidBodyAttitudeDynamics",
    "MPU6050IMU",
    "LIS3MDLMagnetometer",
    "SunSensorArray",
    "BDotController",
    "TRIADEstimator",
]
