"""Hardware ADCS sensor models: MPU6050 IMU, LIS3MDL Magnetometer, and BPW83 Photodiode Sun Sensors.

Implements realistic sensor noise, bias drift, quantization, and digitization
matching manufacturer datasheets:
- MPU6050: 6-DOF IMU (TDK InvenSense)
- LIS3MDL: 3-Axis Magnetometer (STMicroelectronics)
- BPW83: Silicon PIN Photodiode (Vishay) read via 2x ADS1115 16-bit ADCs (Texas Instruments)
"""

import math
from typing import Tuple, List, Optional
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import STANDARD_GRAVITY


class MPU6050IMU:
    """TDK InvenSense MPU6050 6-DOF Inertial Measurement Unit model."""

    def __init__(
        self,
        gyro_noise_density_radps_sqrthz: float = 8.7e-5,  # 0.005 deg/s/sqrt(Hz)
        accel_noise_density_mps2_sqrthz: float = 3.9e-3,  # 400 ug/sqrt(Hz)
        gyro_bias_instability_radps: float = 1.0e-4,
        random_seed: Optional[int] = None,
    ):
        self.gyro_noise_density = gyro_noise_density_radps_sqrthz
        self.accel_noise_density = accel_noise_density_mps2_sqrthz
        self.gyro_bias = np.zeros(3, dtype=np.float64)
        self.bias_drift_rate = gyro_bias_instability_radps
        self.rng = np.random.default_rng(random_seed)

    def measure(
        self,
        true_omega_body_radps: NDArray[np.float64],
        true_linear_accel_body_mps2: NDArray[np.float64],
        dt_s: float = 0.1,
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Simulate noisy, biased gyroscope and accelerometer measurements.

        Args:
            true_omega_body_radps: True angular velocity in Body frame [rad/s].
            true_linear_accel_body_mps2: True non-gravitational specific force [m/s^2].
            dt_s: Sampling interval [s].

        Returns:
            Tuple of (measured_gyro [rad/s], measured_accel [m/s^2]).
        """
        bandwidth = 1.0 / (2.0 * max(1e-4, dt_s))
        sigma_gyro = self.gyro_noise_density * math.sqrt(bandwidth)
        sigma_accel = self.accel_noise_density * math.sqrt(bandwidth)

        # Update random walk bias drift
        self.gyro_bias += self.rng.normal(0.0, self.bias_drift_rate * math.sqrt(dt_s), size=3)

        gyro_noise = self.rng.normal(0.0, sigma_gyro, size=3)
        accel_noise = self.rng.normal(0.0, sigma_accel, size=3)

        measured_gyro = true_omega_body_radps + self.gyro_bias + gyro_noise
        measured_accel = true_linear_accel_body_mps2 + accel_noise

        return measured_gyro, measured_accel


class LIS3MDLMagnetometer:
    """STMicroelectronics LIS3MDL 3-axis digital magnetometer model."""

    def __init__(
        self,
        noise_std_tesla: float = 3.2e-7,  # 3.2 mgauss = 0.32 uT
        full_scale_tesla: float = 0.0016, # +/- 16 gauss
        adc_bits: int = 16,
        random_seed: Optional[int] = None,
    ):
        self.noise_std = noise_std_tesla
        self.full_scale = full_scale_tesla
        self.lsb = (2.0 * full_scale_tesla) / (2.0**adc_bits)
        self.bias = np.zeros(3, dtype=np.float64)
        self.rng = np.random.default_rng(random_seed)

    def measure(self, true_b_body_tesla: NDArray[np.float64]) -> NDArray[np.float64]:
        """Simulate noisy, quantized 3-axis magnetic field measurement [Tesla]."""
        noise = self.rng.normal(0.0, self.noise_std, size=3)
        raw = true_b_body_tesla + self.bias + noise

        # 16-bit ADC quantization
        quantized = np.round(raw / self.lsb) * self.lsb
        return np.clip(quantized, -self.full_scale, self.full_scale)


class SunSensorArray:
    """6-channel photodiode sun sensor network (Vishay BPW83) with TI ADS1115 16-bit ADCs."""

    # Outward normal unit vectors for each CubeSat face: [+X, -X, +Y, -Y, +Z, -Z]
    FACE_NORMALS = np.array([
        [ 1.0,  0.0,  0.0],  # Channel 0: +X face
        [-1.0,  0.0,  0.0],  # Channel 1: -X face
        [ 0.0,  1.0,  0.0],  # Channel 2: +Y face
        [ 0.0, -1.0,  0.0],  # Channel 3: -Y face
        [ 0.0,  0.0,  1.0],  # Channel 4: +Z face
        [ 0.0,  0.0, -1.0],  # Channel 5: -Z face
    ], dtype=np.float64)

    def __init__(
        self,
        v_max_volts: float = 3.0,
        adc_bits: int = 16,
        v_ref_volts: float = 3.3,
        noise_std_volts: float = 0.002,
        random_seed: Optional[int] = None,
    ):
        self.v_max = v_max_volts
        self.lsb = v_ref_volts / (2.0**adc_bits)
        self.noise_std = noise_std_volts
        self.rng = np.random.default_rng(random_seed)

    def measure_voltages(
        self,
        sun_body_unit_vector: NDArray[np.float64],
        eclipse_factor: float = 1.0,
    ) -> NDArray[np.float64]:
        """Simulate analog voltages from 6 photodiodes read through ADS1115 ADCs [V].

        Photodiode response:
            V_k = V_max * max(0, n_hat_k . s_hat_body) * nu + noise
        """
        voltages = np.zeros(6, dtype=np.float64)

        if eclipse_factor <= 1e-4:
            # In eclipse: return dark noise only
            noise = self.rng.normal(0.0, self.noise_std, size=6)
            return np.clip(noise, 0.0, 3.3)

        for k in range(6):
            normal = self.FACE_NORMALS[k]
            cos_theta = float(np.dot(normal, sun_body_unit_vector))
            if cos_theta > 0.0:
                v_signal = self.v_max * cos_theta * eclipse_factor
            else:
                v_signal = 0.0

            v_noisy = v_signal + self.rng.normal(0.0, self.noise_std)
            # 16-bit quantization
            v_quant = np.round(v_noisy / self.lsb) * self.lsb
            voltages[k] = max(0.0, min(3.3, v_quant))

        return voltages

    def estimate_sun_vector(self, measured_voltages: NDArray[np.float64]) -> Tuple[Optional[NDArray[np.float64]], bool]:
        """Estimate the 3D Sun unit vector from the 6 photodiode voltages.

        Equation:
            s_est = sum(V_k * n_hat_k) / ||sum(V_k * n_hat_k)||

        Returns:
            Tuple of (estimated_sun_unit_vector, sun_detected_flag).
        """
        # Differential vector along X, Y, Z axes
        sx = measured_voltages[0] - measured_voltages[1]
        sy = measured_voltages[2] - measured_voltages[3]
        sz = measured_voltages[4] - measured_voltages[5]

        vec = np.array([sx, sy, sz], dtype=np.float64)
        norm = float(np.linalg.norm(vec))

        # Sun detection threshold (at least 0.05 V difference from noise floor)
        if norm < 0.05:
            return None, False

        return (vec / norm), True
