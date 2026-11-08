"""Rigid-body attitude rotational dynamics, quaternion kinematics, and disturbance torques for DOOMSat.

All calculations follow first-principles space vehicle dynamics (Hughes, Markley & Crassidis).
All units are strictly SI:
- Time: seconds (s)
- Angular rate: radians/second (rad/s)
- Torque: Newton-meters (N*m)
- Inertia: kg * m^2
- Magnetic field: Tesla (T)
- Dipole: A * m^2
"""

import math
from typing import Tuple, Dict, Optional
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import (
    EARTH_MU,
    SOLAR_CONSTANT,
    SPEED_OF_LIGHT,
)
from doomsat.core.coordinates import (
    quaternion_to_dcm,
    quaternion_normalize,
    quaternion_rotate_vector,
)


def compute_omega_matrix(omega_radps: NDArray[np.float64]) -> NDArray[np.float64]:
    """Construct the 4x4 kinematic Omega matrix for scalar-first quaternion propagation.

    q_dot = 0.5 * Omega(omega) * q
    where q = [q_w, q_x, q_y, q_z]^T

    Args:
        omega_radps: Angular velocity vector [wx, wy, wz] in Body frame [rad/s].

    Returns:
        4x4 skew-symmetric kinematics matrix.
    """
    wx, wy, wz = omega_radps[0], omega_radps[1], omega_radps[2]
    omega_mat = np.array([
        [0.0, -wx,  -wy,  -wz],
        [wx,   0.0,  wz,  -wy],
        [wy,  -wz,   0.0,  wx],
        [wz,   wy,  -wx,   0.0],
    ], dtype=np.float64)
    return omega_mat


def compute_gravity_gradient_torque(
    r_body_m: NDArray[np.float64],
    inertia_tensor: NDArray[np.float64],
    mu: float = EARTH_MU,
) -> NDArray[np.float64]:
    """Calculate the gravity gradient disturbance torque acting on the spacecraft.

    Equation:
        tau_gg = (3 * mu / r^3) * (r_hat_body x (I * r_hat_body))

    Args:
        r_body_m: Spacecraft position vector transformed to Body frame [m].
        inertia_tensor: 3x3 mass moment of inertia tensor about CoG [kg*m^2].
        mu: Gravitational parameter [m^3/s^2].

    Returns:
        3D torque vector [tau_x, tau_y, tau_z] in Body frame [N*m].
    """
    r_norm = float(np.linalg.norm(r_body_m))
    if r_norm < 1000.0:
        return np.zeros(3, dtype=np.float64)

    r_hat = r_body_m / r_norm
    i_r = np.dot(inertia_tensor, r_hat)

    tau_gg = (3.0 * mu / (r_norm**3)) * np.cross(r_hat, i_r)
    return tau_gg


def compute_magnetic_disturbance_torque(
    residual_dipole_am2: NDArray[np.float64],
    b_body_tesla: NDArray[np.float64],
) -> NDArray[np.float64]:
    """Calculate the disturbance torque from the satellite's residual magnetic dipole.

    Equation:
        tau_mag = m_residual x B_body

    Args:
        residual_dipole_am2: Spacecraft internal residual magnetic dipole [A*m^2].
        b_body_tesla: Ambient geomagnetic field in Body frame [Tesla].

    Returns:
        3D torque vector in Body frame [N*m].
    """
    return np.cross(residual_dipole_am2, b_body_tesla)


def compute_aerodynamic_torque(
    v_rel_body_mps: NDArray[np.float64],
    atmospheric_density_kg_m3: float = 1.0e-12,
    drag_coefficient: float = 2.2,
    cross_sectional_area_m2: float = 0.010,
    cp_cg_offset_m: NDArray[np.float64] = np.array([0.005, -0.005, 0.005]),
) -> NDArray[np.float64]:
    """Calculate aerodynamic drag disturbance torque.

    Equation:
        F_drag = - 0.5 * rho * v_rel^2 * C_d * A * v_hat
        tau_aero = r_cp_cg x F_drag

    Args:
        v_rel_body_mps: Spacecraft velocity relative to atmosphere in Body frame [m/s].
        atmospheric_density_kg_m3: Atmospheric density at orbital altitude [kg/m^3] (e.g. ~1e-12 at 500 km).
        drag_coefficient: Hypersonic drag coefficient (typically 2.2 for CubeSat).
        cross_sectional_area_m2: Projected cross-sectional area [m^2] (0.010 m^2 for 1U).
        cp_cg_offset_m: Vector from CoG to Center of Pressure [m].

    Returns:
        3D torque vector in Body frame [N*m].
    """
    v_norm = float(np.linalg.norm(v_rel_body_mps))
    if v_norm < 1e-3 or atmospheric_density_kg_m3 <= 0.0:
        return np.zeros(3, dtype=np.float64)

    v_hat = v_rel_body_mps / v_norm
    f_drag_mag = 0.5 * atmospheric_density_kg_m3 * (v_norm**2) * drag_coefficient * cross_sectional_area_m2
    f_drag_vec = - f_drag_mag * v_hat

    return np.cross(cp_cg_offset_m, f_drag_vec)


def compute_srp_torque(
    sun_body_unit_vector: NDArray[np.float64],
    eclipse_factor: float = 1.0,
    surface_area_m2: float = 0.010,
    reflectivity: float = 0.6,
    cp_cg_offset_m: NDArray[np.float64] = np.array([0.005, 0.005, -0.005]),
) -> NDArray[np.float64]:
    """Calculate Solar Radiation Pressure (SRP) disturbance torque.

    Equation:
        P_srp = S_0 / c * nu
        F_srp = - P_srp * A * (1 + rho_s) * s_hat_body
        tau_srp = r_cp_cg x F_srp

    Args:
        sun_body_unit_vector: Unit vector pointing toward the Sun in Body frame.
        eclipse_factor: Eclipse factor nu in [0.0, 1.0].
        surface_area_m2: Illuminated surface area [m^2].
        reflectivity: Surface optical specular/diffuse reflectivity coefficient.
        cp_cg_offset_m: Center of pressure offset from CoG [m].

    Returns:
        3D torque vector in Body frame [N*m].
    """
    if eclipse_factor <= 1e-4:
        return np.zeros(3, dtype=np.float64)

    p_srp = (SOLAR_CONSTANT / SPEED_OF_LIGHT) * eclipse_factor
    f_srp_mag = p_srp * surface_area_m2 * (1.0 + reflectivity)
    f_srp_vec = - f_srp_mag * sun_body_unit_vector

    return np.cross(cp_cg_offset_m, f_srp_vec)


class RigidBodyAttitudeDynamics:
    """Numerical solver for rigid-body attitude rotational dynamics and kinematics.

    State:
        q: 4-element unit quaternion [qw, qx, qy, qz] (Body relative to ECI).
        omega: 3-element angular velocity vector [wx, wy, wz] in Body frame [rad/s].
    """

    def __init__(
        self,
        inertia_tensor: NDArray[np.float64],
        initial_q: NDArray[np.float64] = np.array([1.0, 0.0, 0.0, 0.0]),
        initial_omega_radps: NDArray[np.float64] = np.array([0.0, 0.0, 0.0]),
    ):
        self.inertia = np.array(inertia_tensor, dtype=np.float64)
        self.inv_inertia = np.linalg.inv(self.inertia)
        self.q = quaternion_normalize(np.array(initial_q, dtype=np.float64))
        self.omega = np.array(initial_omega_radps, dtype=np.float64)

    def compute_derivatives(
        self,
        q: NDArray[np.float64],
        omega: NDArray[np.float64],
        total_torque_body_nm: NDArray[np.float64],
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Compute state derivatives: q_dot and omega_dot.

        Euler equations:
            omega_dot = I^-1 * ( tau - omega x (I * omega) )
        Kinematics:
            q_dot = 0.5 * Omega(omega) * q
        """
        # Gyroscopic cross-coupling term: omega x (I * omega)
        i_omega = np.dot(self.inertia, omega)
        gyroscopic = np.cross(omega, i_omega)

        omega_dot = np.dot(self.inv_inertia, total_torque_body_nm - gyroscopic)

        # Quaternion derivative
        omega_mat = compute_omega_matrix(omega)
        q_dot = 0.5 * np.dot(omega_mat, q)

        return q_dot, omega_dot

    def step_rk4(
        self,
        dt_s: float,
        total_torque_body_nm: NDArray[np.float64],
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Propagate attitude state by dt_s using explicit 4th-order Runge-Kutta.

        Args:
            dt_s: Numerical integration step [s].
            total_torque_body_nm: Net external plus control torque in Body frame [N*m].

        Returns:
            Tuple of (new_q, new_omega).
        """
        q0 = self.q
        w0 = self.omega
        tau = total_torque_body_nm

        # k1
        k1_q, k1_w = self.compute_derivatives(q0, w0, tau)

        # k2
        q_mid1 = quaternion_normalize(q0 + 0.5 * dt_s * k1_q)
        w_mid1 = w0 + 0.5 * dt_s * k1_w
        k2_q, k2_w = self.compute_derivatives(q_mid1, w_mid1, tau)

        # k3
        q_mid2 = quaternion_normalize(q0 + 0.5 * dt_s * k2_q)
        w_mid2 = w0 + 0.5 * dt_s * k2_w
        k3_q, k3_w = self.compute_derivatives(q_mid2, w_mid2, tau)

        # k4
        q_end = quaternion_normalize(q0 + dt_s * k3_q)
        w_end = w0 + dt_s * k3_w
        k4_q, k4_w = self.compute_derivatives(q_end, w_end, tau)

        # Accumulate
        q_next = q0 + (dt_s / 6.0) * (k1_q + 2.0 * k2_q + 2.0 * k3_q + k4_q)
        w_next = w0 + (dt_s / 6.0) * (k1_w + 2.0 * k2_w + 2.0 * k3_w + k4_w)

        self.q = quaternion_normalize(q_next)
        self.omega = w_next

        return self.q, self.omega

