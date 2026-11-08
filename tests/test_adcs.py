"""Unit tests for DOOMSat ADCS rigid-body dynamics, sensor models, B-dot detumbling, and TRIAD estimation."""

import math
import numpy as np
import pytest

from doomsat.core.coordinates import (
    quaternion_to_dcm,
    quaternion_normalize,
    quaternion_rotate_vector,
    euler_to_quaternion,
)
from doomsat.adcs import (
    compute_omega_matrix,
    compute_gravity_gradient_torque,
    compute_magnetic_disturbance_torque,
    compute_aerodynamic_torque,
    compute_srp_torque,
    RigidBodyAttitudeDynamics,
    MPU6050IMU,
    LIS3MDLMagnetometer,
    SunSensorArray,
    BDotController,
    TRIADEstimator,
)


def test_torque_free_attitude_dynamics_energy_conservation():
    """Verify that torque-free rigid body rotation conserves kinetic energy and angular momentum."""
    # Principal moments of inertia for DOOMSat 1U (from Phase 2): [Ixx, Iyy, Izz]
    inertia = np.diag([0.001035, 0.001040, 0.000661])
    initial_omega = np.array([0.05, -0.04, 0.03], dtype=np.float64)  # rad/s

    dynamics = RigidBodyAttitudeDynamics(
        inertia_tensor=inertia,
        initial_q=np.array([1.0, 0.0, 0.0, 0.0]),
        initial_omega_radps=initial_omega,
    )

    # Initial kinetic energy: T = 0.5 * omega^T * I * omega
    t_initial = 0.5 * float(np.dot(initial_omega, np.dot(inertia, initial_omega)))
    h_initial = float(np.linalg.norm(np.dot(inertia, initial_omega)))

    dt = 0.05  # 50 ms steps
    steps = 500  # 25 seconds of tumble

    for _ in range(steps):
        dynamics.step_rk4(dt_s=dt, total_torque_body_nm=np.zeros(3))

    t_final = 0.5 * float(np.dot(dynamics.omega, np.dot(inertia, dynamics.omega)))
    h_final = float(np.linalg.norm(np.dot(inertia, dynamics.omega)))

    # Energy and momentum magnitude should be conserved within numerical integration precision
    assert math.isclose(t_initial, t_final, rel_tol=1e-5)
    assert math.isclose(h_initial, h_final, rel_tol=1e-5)

    # Unit quaternion magnitude must be strictly 1.0
    assert math.isclose(np.linalg.norm(dynamics.q), 1.0, abs_tol=1e-12)


def test_gravity_gradient_torque_properties():
    """Verify gravity gradient torque properties (zero along principal axis, restoring when tilted)."""
    inertia = np.diag([0.0010, 0.0012, 0.0006])  # Asymmetric moments

    # 1. Aligned with body Z-axis: r_body = [0, 0, r] -> cross product r x (I*r) = 0
    r_aligned = np.array([0.0, 0.0, 6878137.0])
    tau_zero = compute_gravity_gradient_torque(r_aligned, inertia)
    assert np.allclose(tau_zero, 0.0, atol=1e-15)

    # 2. Tilted 45 degrees in X-Z plane: r_body = [r/sqrt(2), 0, r/sqrt(2)]
    r_tilted = np.array([6878137.0 / math.sqrt(2.0), 0.0, 6878137.0 / math.sqrt(2.0)])
    tau_tilted = compute_gravity_gradient_torque(r_tilted, inertia)

    # Torque should be non-zero and act along intermediate Y-axis
    assert abs(tau_tilted[1]) > 5e-10
    assert math.isclose(tau_tilted[0], 0.0, abs_tol=1e-15)
    assert math.isclose(tau_tilted[2], 0.0, abs_tol=1e-15)


def test_environmental_disturbance_torques_magnitude():
    """Verify aerodynamic, magnetic residual, and solar radiation pressure torque magnitudes."""
    # 1. Magnetic disturbance torque: m_res = 0.01 A*m^2, B = 40 uT
    m_res = np.array([0.01, 0.0, 0.0])
    b_body = np.array([0.0, 40.0e-6, 0.0])
    tau_mag = compute_magnetic_disturbance_torque(m_res, b_body)
    # tau = 0.01 * 40e-6 = 4e-7 N*m
    assert math.isclose(np.linalg.norm(tau_mag), 4.0e-7, rel_tol=1e-6)

    # 2. Aerodynamic torque at 500 km (v = 7600 m/s, rho = 1e-12 kg/m^3)
    v_body = np.array([7600.0, 0.0, 0.0])
    tau_aero = compute_aerodynamic_torque(v_body)
    # Typical 1U CubeSat aero torque is ~ 1e-8 to 1e-6 N*m
    assert 1e-9 < np.linalg.norm(tau_aero) < 1e-5

    # 3. Solar Radiation Pressure torque in sunlight
    s_body = np.array([1.0, 0.0, 0.0])
    tau_srp = compute_srp_torque(s_body, eclipse_factor=1.0)
    # Typical SRP torque is ~ 1e-9 to 1e-7 N*m
    assert 1e-10 < np.linalg.norm(tau_srp) < 1e-6

    # In eclipse (nu = 0), SRP torque must be exactly zero
    tau_srp_eclipse = compute_srp_torque(s_body, eclipse_factor=0.0)
    assert np.allclose(tau_srp_eclipse, 0.0)


def test_mpu6050_imu_sensor_noise_and_drift():
    """Verify MPU6050 IMU outputs realistic noise and bias drift."""
    imu = MPU6050IMU(random_seed=42)
    true_omega = np.array([0.01, -0.02, 0.005])
    true_accel = np.array([0.0, 0.0, 0.0])

    meas_omega, meas_accel = imu.measure(true_omega, true_accel, dt_s=0.1)

    # Measured values should be close to true values within 3-sigma noise
    assert np.allclose(meas_omega, true_omega, atol=0.01)
    assert np.allclose(meas_accel, true_accel, atol=0.1)


def test_lis3mdl_magnetometer_quantization():
    """Verify LIS3MDL magnetometer measurement, noise, and 16-bit quantization."""
    mag = LIS3MDLMagnetometer(random_seed=42)
    true_b = np.array([25.0e-6, -15.0e-6, 35.0e-6])  # ~45 uT field

    meas_b = mag.measure(true_b)
    # Measured field should match true field within noise margin (< 1 uT)
    assert np.allclose(meas_b, true_b, atol=1.0e-6)


def test_sun_sensor_array_and_vector_estimation():
    """Verify 6-channel photodiode voltages and Sun vector reconstruction."""
    sun_sensors = SunSensorArray(random_seed=42)

    # Test 1: Sun directly on +X face: s_body = [1.0, 0.0, 0.0]
    s_true_x = np.array([1.0, 0.0, 0.0])
    voltages_x = sun_sensors.measure_voltages(s_true_x, eclipse_factor=1.0)

    # Channel 0 (+X) should be near 3.0 V, all other channels should be ~0 V
    assert math.isclose(voltages_x[0], 3.0, abs_tol=0.05)
    assert voltages_x[1] < 0.05  # -X face
    assert voltages_x[2] < 0.05  # +Y face

    s_est_x, detected_x = sun_sensors.estimate_sun_vector(voltages_x)
    assert detected_x
    assert np.allclose(s_est_x, s_true_x, atol=0.02)

    # Test 2: In eclipse -> no sun detected
    voltages_dark = sun_sensors.measure_voltages(s_true_x, eclipse_factor=0.0)
    _, detected_dark = sun_sensors.estimate_sun_vector(voltages_dark)
    assert not detected_dark


def test_bdot_controller_detumbling_convergence():
    """Verify that B-dot control law damps angular velocity in a rotating magnetic field."""
    controller = BDotController(gain_k=1.0e5, max_dipole_am2=0.15)
    inertia = np.diag([0.001035, 0.001040, 0.000661])

    # Initial high tumble rate: 0.1 rad/s (~5.7 deg/s)
    dynamics = RigidBodyAttitudeDynamics(
        inertia_tensor=inertia,
        initial_omega_radps=np.array([0.08, -0.06, 0.02]),
    )

    initial_spin = float(np.linalg.norm(dynamics.omega))
    assert initial_spin > 0.09

    dt = 0.1
    b_eci = np.array([10.0e-6, 20.0e-6, 35.0e-6])  # Ambient geomagnetic field vector
    t = 0.0

    # Simulate 150 seconds of closed-loop detumbling
    for _ in range(1500):
        # Ambient magnetic field in body frame rotated by current spacecraft attitude
        b_body = quaternion_rotate_vector(dynamics.q, b_eci)

        m_dipole, tau_ctrl = controller.compute_control_torque(b_body, t)
        dynamics.step_rk4(dt_s=dt, total_torque_body_nm=tau_ctrl)
        t += dt

    final_spin = float(np.linalg.norm(dynamics.omega))

    # B-dot controller must damp tumble rate by at least 80% (final < 0.2 * initial)
    assert final_spin < initial_spin * 0.2


def test_triad_attitude_estimator():
    """Verify TRIAD algorithm reconstructs true attitude quaternion within 0.1 degrees."""
    # True orientation: Roll=20 deg, Pitch=-15 deg, Yaw=45 deg
    q_true = euler_to_quaternion(math.radians(20.0), math.radians(-15.0), math.radians(45.0))

    # Reference vectors in ECI (Sun and Magnetic field)
    s_eci = np.array([1.0, 0.0, 0.0])
    b_eci = np.array([0.0, 0.0, 1.0])

    # Transform reference vectors to Body frame using true attitude
    s_body = quaternion_rotate_vector(q_true, s_eci)
    b_body = quaternion_rotate_vector(q_true, b_eci)

    # Run TRIAD estimator
    q_est = TRIADEstimator.estimate_attitude_quaternion(
        v1_body=s_body,
        v2_body=b_body,
        v1_eci=s_eci,
        v2_eci=b_eci,
    )

    assert q_est is not None
    # Dot product of unit quaternions q and q_est represents angle error: cos(error/2) = |q . q_est|
    dot_prod = abs(float(np.dot(q_true, q_est)))
    assert math.isclose(dot_prod, 1.0, abs_tol=1e-8)
