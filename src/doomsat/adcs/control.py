"""Attitude control laws and state estimators: B-dot detumbling and TRIAD attitude determination.

All calculations follow standard aerospace control and attitude determination methods.
All units are strictly SI:
- Magnetic dipole: A * m^2
- Magnetic field: Tesla (T)
- Torque: N * m
"""

import math
from typing import Tuple, Optional
import numpy as np
from numpy.typing import NDArray

from doomsat.core.coordinates import dcm_to_quaternion


class BDotController:
    """B-dot magnetic detumbling controller for 3-axis magnetorquers.

    Control Law:
        m_ctrl = - k_bdot * (B_dot / ||B||)
        tau_ctrl = m_ctrl x B_body

    Where:
        B_dot is the time derivative of the magnetic field in the body frame:
        B_dot ~= (B_k - B_{k-1}) / dt
    """

    def __init__(
        self,
        gain_k: float = 2.0e5,
        max_dipole_am2: float = 0.15,
    ):
        self.gain_k = gain_k
        self.max_dipole = max_dipole_am2
        self.prev_b_body: Optional[NDArray[np.float64]] = None
        self.prev_time_s: float = 0.0

    def compute_control_torque(
        self,
        b_body_tesla: NDArray[np.float64],
        current_time_s: float,
    ) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Compute the magnetic dipole command and resulting control torque.

        Args:
            b_body_tesla: Measured/estimated magnetic field in Body frame [Tesla].
            current_time_s: Current simulation timestamp [s].

        Returns:
            Tuple of (m_dipole_am2 [A*m^2], control_torque_body_nm [N*m]).
        """
        b_norm = float(np.linalg.norm(b_body_tesla))
        if b_norm < 1e-9:
            return np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64)

        if self.prev_b_body is None:
            self.prev_b_body = np.copy(b_body_tesla)
            self.prev_time_s = current_time_s
            return np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64)

        dt = current_time_s - self.prev_time_s
        if dt < 1e-4:
            # Step too small to estimate derivative
            return np.zeros(3, dtype=np.float64), np.zeros(3, dtype=np.float64)

        # Numerical derivative of B in body frame: B_dot = - omega x B
        b_dot = (b_body_tesla - self.prev_b_body) / dt

        # Update history
        self.prev_b_body = np.copy(b_body_tesla)
        self.prev_time_s = current_time_s

        # Normalized B-dot control law
        m_cmd = - self.gain_k * (b_dot / b_norm)

        # Actuator saturation: limit dipole to +/- max_dipole
        m_dipole = np.clip(m_cmd, -self.max_dipole, self.max_dipole)

        # Control torque: tau = m x B
        tau_ctrl = np.cross(m_dipole, b_body_tesla)

        return m_dipole, tau_ctrl


class TRIADEstimator:
    """TRIAD deterministic attitude determination algorithm.

    Constructs an orthonormal triad from two non-parallel vector observations
    (typically Sun vector and Earth magnetic field vector) to determine
    spacecraft orientation relative to ECI.
    """

    @staticmethod
    def estimate_attitude_quaternion(
        v1_body: NDArray[np.float64],
        v2_body: NDArray[np.float64],
        v1_eci: NDArray[np.float64],
        v2_eci: NDArray[np.float64],
    ) -> Optional[NDArray[np.float64]]:
        """Reconstruct the attitude quaternion q_body_to_eci using TRIAD.

        Args:
            v1_body: Primary reference vector measured in Body frame (e.g. Sun vector).
            v2_body: Secondary reference vector measured in Body frame (e.g. B-field).
            v1_eci: Primary reference vector known in ECI frame.
            v2_eci: Secondary reference vector known in ECI frame.

        Returns:
            Scalar-first unit quaternion [qw, qx, qy, qz], or None if vectors are degenerate.
        """
        # Normalize inputs
        n1_b = v1_body / np.linalg.norm(v1_body)
        n2_b = v2_body / np.linalg.norm(v2_body)
        n1_r = v1_eci / np.linalg.norm(v1_eci)
        n2_r = v2_eci / np.linalg.norm(v2_eci)

        # Body triad
        t1_b = n1_b
        cross_b = np.cross(n1_b, n2_b)
        cross_b_norm = np.linalg.norm(cross_b)
        if cross_b_norm < 1e-4:
            return None  # Parallel or collinear vectors

        t2_b = cross_b / cross_b_norm
        t3_b = np.cross(t1_b, t2_b)
        m_body = np.column_stack([t1_b, t2_b, t3_b])

        # Reference triad
        t1_r = n1_r
        cross_r = np.cross(n1_r, n2_r)
        cross_r_norm = np.linalg.norm(cross_r)
        if cross_r_norm < 1e-4:
            return None

        t2_r = cross_r / cross_r_norm
        t3_r = np.cross(t1_r, t2_r)
        m_ref = np.column_stack([t1_r, t2_r, t3_r])

        # Direction Cosine Matrix: A_body_to_eci
        # v_body = A * v_eci => M_body = A * M_ref => A = M_body * M_ref^T
        dcm_eci_to_body = np.dot(m_body, m_ref.T)

        # Convert DCM to unit quaternion
        q = dcm_to_quaternion(dcm_eci_to_body)
        return q

