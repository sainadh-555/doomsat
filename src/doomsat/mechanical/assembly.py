"""CubeSat 1U mechanical assembly, mass accumulation, CoG, and inertia tensor engine.

All calculations follow rigid-body mechanics and Parallel Axis Theorem (Steiner's theorem).
All units are strictly SI:
- Distance: meters (m)
- Mass: kilograms (kg)
- Inertia: kg * m^2
"""

from typing import List, Dict, Tuple, Optional
import numpy as np
from numpy.typing import NDArray

from doomsat.core.constants import CUBESAT_1U_MAX_MASS_KG
from doomsat.mechanical.component import MechanicalComponent, ShapeType, ParameterSource


class CubeSatAssembly:
    """Manages the 1U CubeSat physical assembly, mass budget, CoG, and inertia tensor.

    Attributes:
        components: List of registered MechanicalComponent instances.
        envelope_m: Outer dimensional bounds (dx, dy, dz) [m].
    """

    def __init__(
        self,
        name: str = "DOOMSat-1U",
        envelope_m: Tuple[float, float, float] = (0.100, 0.100, 0.1135),
    ):
        self.name = name
        self.envelope_m = np.array(envelope_m, dtype=np.float64)
        self.components: List[MechanicalComponent] = []

    def add_component(self, component: MechanicalComponent) -> None:
        """Register a component into the mechanical assembly."""
        self.components.append(component)

    def get_component(self, part_id: str) -> Optional[MechanicalComponent]:
        """Retrieve a component by its unique Part ID."""
        for comp in self.components:
            if comp.part_id == part_id:
                return comp
        return None

    @property
    def total_mass_kg(self) -> float:
        """Total accumulated mass of all registered components [kg]."""
        return sum(c.mass_kg for c in self.components)

    @property
    def mass_margin_pct(self) -> float:
        """Mass growth margin against the 1.330 kg standard 1U specification limit [%].

        Equation:
            Margin = ((M_limit - M_total) / M_limit) * 100%
        """
        margin = ((CUBESAT_1U_MAX_MASS_KG - self.total_mass_kg) / CUBESAT_1U_MAX_MASS_KG) * 100.0
        return margin

    def compute_center_of_gravity(self) -> NDArray[np.float64]:
        """Compute the 3D Center of Gravity (CoG) relative to the 1U chassis geometric center.

        Equation:
            r_cg = (1 / M_total) * sum(m_k * r_k)

        Returns:
            3D vector [x_cg, y_cg, z_cg] in meters.
        """
        total_m = self.total_mass_kg
        if total_m <= 0.0:
            return np.zeros(3, dtype=np.float64)

        weighted_positions = sum(c.mass_kg * c.centroid_m for c in self.components)
        return weighted_positions / total_m

    def verify_cog_limit(self, max_offset_m: float = 0.020) -> Tuple[bool, float]:
        """Verify whether CoG resides within allowable radial offset from geometric center.

        Standard CubeSat requirement (REQ-MEC-03): ||r_cg|| <= 20.0 mm (0.020 m).

        Args:
            max_offset_m: Maximum allowable radial distance from origin [m].

        Returns:
            Tuple of (is_compliant, actual_distance_m).
        """
        cog = self.compute_center_of_gravity()
        dist = float(np.linalg.norm(cog))
        return (dist <= max_offset_m), dist

    def compute_inertia_tensor(self) -> NDArray[np.float64]:
        """Compute the complete 3x3 mass moment of inertia tensor about the satellite CoG.

        Applies Steiner's Parallel Axis Theorem for each component:
            I_cg = sum_{k=1}^N [ I_{local}^{(k)} + m_k * ( ||Delta r_k||^2 * I_3 - Delta r_k * Delta r_k^T ) ]

        where:
            Delta r_k = r_k - r_cg

        Returns:
            3x3 symmetric mass moment of inertia matrix [kg * m^2].
        """
        r_cg = self.compute_center_of_gravity()
        i_tensor = np.zeros((3, 3), dtype=np.float64)

        for comp in self.components:
            # Local component inertia about its own centroid
            i_local = comp.compute_local_inertia()

            # Lever arm from satellite CoG to component centroid
            dr = comp.centroid_m - r_cg
            dr_x, dr_y, dr_z = dr
            dr_sq = np.dot(dr, dr)
            m = comp.mass_kg

            # Parallel axis translation matrix
            i_parallel = m * (dr_sq * np.eye(3) - np.outer(dr, dr))

            i_tensor += (i_local + i_parallel)

        return i_tensor

    def compute_principal_moments(self) -> Tuple[NDArray[np.float64], NDArray[np.float64]]:
        """Compute principal moments of inertia and principal axes via eigenvalue decomposition.

        Solves:
            I_tensor * v_i = lambda_i * v_i

        Returns:
            Tuple of (sorted_principal_moments [kg*m^2], principal_axes_matrix).
            Moments are sorted in ascending order: I_1 <= I_2 <= I_3.
        """
        i_tensor = self.compute_inertia_tensor()
        eigenvalues, eigenvectors = np.linalg.eigh(i_tensor)

        sort_indices = np.argsort(eigenvalues)
        sorted_moments = eigenvalues[sort_indices]
        sorted_axes = eigenvectors[:, sort_indices]

        return sorted_moments, sorted_axes

    def verify_inertia_properties(self) -> Dict[str, bool]:
        """Verify physical validity and symmetry of the computed inertia tensor.

        Checks:
        1. Symmetry: I_ij == I_ji
        2. Positive Definiteness: all eigenvalues > 0
        3. Triangle Inequalities: I_1 + I_2 >= I_3, I_2 + I_3 >= I_1, I_1 + I_3 >= I_2
        """
        i_tensor = self.compute_inertia_tensor()
        moments, _ = self.compute_principal_moments()

        is_symmetric = np.allclose(i_tensor, i_tensor.T, atol=1e-12)
        is_positive_definite = bool(np.all(moments > 0.0) and (np.linalg.det(i_tensor) > 0.0))

        i1, i2, i3 = moments
        triangle_check = (
            (i1 + i2 >= i3 - 1e-12) and
            (i2 + i3 >= i1 - 1e-12) and
            (i1 + i3 >= i2 - 1e-12)
        )

        return {
            "symmetric": bool(is_symmetric),
            "positive_definite": is_positive_definite,
            "triangle_inequalities": bool(triangle_check),
        }

    def check_envelope_containment(self) -> Tuple[bool, List[str]]:
        """Verify that all components reside inside the 1U mechanical envelope.

        Envelope boundaries:
            -dx/2 <= x <= +dx/2
            -dy/2 <= y <= +dy/2
            -dz/2 <= z <= +dz/2

        Returns:
            Tuple of (all_contained, list_of_violations).
        """
        half_env = self.envelope_m / 2.0
        violations = []

        for comp in self.components:
            min_pt = comp.aabb_min
            max_pt = comp.aabb_max

            # Allow 0.5 mm tolerance for flush body-mounted solar panels / chassis rails
            tol = 0.0005
            if (min_pt[0] < -half_env[0] - tol) or (max_pt[0] > half_env[0] + tol):
                violations.append(f"{comp.name} ({comp.part_id}) violates X envelope: min={min_pt[0]*1000:.1f}mm, max={max_pt[0]*1000:.1f}mm")
            if (min_pt[1] < -half_env[1] - tol) or (max_pt[1] > half_env[1] + tol):
                violations.append(f"{comp.name} ({comp.part_id}) violates Y envelope: min={min_pt[1]*1000:.1f}mm, max={max_pt[1]*1000:.1f}mm")
            if (min_pt[2] < -half_env[2] - tol) or (max_pt[2] > half_env[2] + tol):
                violations.append(f"{comp.name} ({comp.part_id}) violates Z envelope: min={min_pt[2]*1000:.1f}mm, max={max_pt[2]*1000:.1f}mm")

        return (len(violations) == 0), violations

    def check_internal_collisions(self) -> Tuple[bool, List[str]]:
        """Verify that internal components do not overlap or intersect one another.

        Returns:
            Tuple of (zero_collisions, list_of_collision_pairs).
        """
        collisions = []
        n = len(self.components)

        # Skip components that are distributed or external flush skins
        skip_parts = {"STR-01", "EPS-02", "SEN-03", "HAR-01"}

        for i in range(n):
            for j in range(i + 1, n):
                c1 = self.components[i]
                c2 = self.components[j]

                if (c1.part_id in skip_parts) or (c2.part_id in skip_parts):
                    continue

                if c1.intersects(c2, tolerance_m=0.0005):
                    collisions.append(f"Collision between {c1.name} ({c1.part_id}) and {c2.name} ({c2.part_id})")

        return (len(collisions) == 0), collisions


def build_reference_doomsat_1u() -> CubeSatAssembly:
    """Factory function constructing the complete reference DOOMSat 1U assembly.

    Includes all 12 traceable components:
    - 1U Al6061-T6 Chassis
    - OmniVision OV5647 5MP Camera Payload
    - Raspberry Pi Zero 2 W OBC
    - 18650 Li-ion Battery & EPS PMIC Board
    - ADCS Sensor Board (MPU6050, LIS3MDL, BME280, ADS1115 x2)
    - Comms Transceiver PCB (ESP32 + Semtech SX1276)
    - Deployable Antenna Mechanism
    - 5x Body-Mounted Solar Panels
    - 6x Sun Sensor Photodiodes
    - Harness and Standoffs
    - Precision Brass Trim Ballast

    Returns:
        Fully populated CubeSatAssembly.
    """
    assembly = CubeSatAssembly(name="DOOMSat-1U-Flight")

    # 1. Primary Chassis Structure (Al6061-T6, 120g)
    assembly.add_component(MechanicalComponent(
        name="Chassis Structure (Al6061-T6)",
        part_id="STR-01",
        mass_kg=0.120,
        centroid_m=(0.0, 0.0, 0.0),
        dimensions_m=(0.100, 0.100, 0.1135),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.DATASHEET_DERIVED,
        material="Al6061-T6"
    ))

    # 2. Camera Payload: OV5647 5MP with M12 Lens bracket on -Z face (3.5g)
    assembly.add_component(MechanicalComponent(
        name="Camera Payload (OV5647)",
        part_id="PAY-01",
        mass_kg=0.0035,
        centroid_m=(0.0, 0.0, -0.042),
        dimensions_m=(0.025, 0.024, 0.010),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.DATASHEET_DERIVED,
        material="FR4/Glass/Aluminum"
    ))

    # 3. OBC: Raspberry Pi Zero 2 W (16g, Tier 1 stack at Z = -25mm)
    assembly.add_component(MechanicalComponent(
        name="OBC (RPi Zero 2 W)",
        part_id="OBC-01",
        mass_kg=0.016,
        centroid_m=(0.0, 0.0, -0.025),
        dimensions_m=(0.065, 0.030, 0.005),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.DATASHEET_DERIVED,
        material="FR4/Silicon"
    ))

    # 4. Battery: 18650 Li-ion Cell (45g, cylindrical cell along X-axis at Z = -10mm, Y = +15mm)
    assembly.add_component(MechanicalComponent(
        name="Battery Cell (18650 Li-ion)",
        part_id="EPS-01",
        mass_kg=0.045,
        centroid_m=(0.0, 0.015, -0.010),
        dimensions_m=(0.0652, 0.0185, 0.0185),
        shape=ShapeType.CYLINDER_X,
        source=ParameterSource.DATASHEET_DERIVED,
        material="Li-ion / Steel Can"
    ))

    # 5. EPS PMIC & Buck-Boost Regulators (10g, at Z = -10mm, Y = -20mm)
    assembly.add_component(MechanicalComponent(
        name="EPS PMIC & Power Regulators",
        part_id="EPS-03",
        mass_kg=0.010,
        centroid_m=(0.0, -0.020, -0.010),
        dimensions_m=(0.050, 0.040, 0.004),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.DATASHEET_DERIVED,
        material="FR4/Copper"
    ))

    # 6. ADCS Sensor PCB: MPU6050, LIS3MDL, BME280, 2x ADS1115 (15g, Tier 2 at Z = +5mm)
    assembly.add_component(MechanicalComponent(
        name="ADCS & Environmental Sensors PCB",
        part_id="SEN-01",
        mass_kg=0.015,
        centroid_m=(0.0, 0.0, 0.005),
        dimensions_m=(0.080, 0.080, 0.004),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.DATASHEET_DERIVED,
        material="FR4"
    ))

    # 7. TT&C Comms Controller & SX1276 LoRa PCB (18g, Tier 3 at Z = +25mm)
    assembly.add_component(MechanicalComponent(
        name="Comms Controller (ESP32 + SX1276)",
        part_id="COM-01",
        mass_kg=0.018,
        centroid_m=(0.0, 0.0, 0.025),
        dimensions_m=(0.080, 0.080, 0.006),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.DATASHEET_DERIVED,
        material="FR4/Shielded Metal"
    ))

    # 8. Deployable Monopole Antenna Mechanism (18g, Top deck at Z = +45mm)
    assembly.add_component(MechanicalComponent(
        name="Antenna Deployment Module",
        part_id="ANT-01",
        mass_kg=0.018,
        centroid_m=(0.0, 0.0, 0.045),
        dimensions_m=(0.060, 0.060, 0.010),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.ASSUMED,
        material="Delrin/Nitinol"
    ))

    # 9. Body-Mounted Solar Panels (5x external panels, 55g total, 11g each)
    assembly.add_component(MechanicalComponent(
        name="Solar Panel Array (+X, -X, +Y, -Y, +Z)",
        part_id="EPS-02",
        mass_kg=0.055,
        centroid_m=(0.0, 0.0, 0.002),
        dimensions_m=(0.098, 0.098, 0.0015),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.DATASHEET_DERIVED,
        material="GaAs / PCB"
    ))

    # 10. Sun Sensor Network (6x Vishay BPW83 photodiodes on faces, 1.5g total)
    assembly.add_component(MechanicalComponent(
        name="Sun Sensor Photodiodes (6x BPW83)",
        part_id="SEN-03",
        mass_kg=0.0015,
        centroid_m=(0.0, 0.0, 0.0),
        dimensions_m=(0.008, 0.006, 0.003),
        shape=ShapeType.POINT_MASS,
        source=ParameterSource.DATASHEET_DERIVED,
        material="Silicon/Epoxy"
    ))

    # 11. Standoffs, Screws, and Internal Harness (30g, distributed)
    assembly.add_component(MechanicalComponent(
        name="Harness, Standoffs & Fasteners",
        part_id="HAR-01",
        mass_kg=0.030,
        centroid_m=(0.0, 0.0, 0.0),
        dimensions_m=(0.085, 0.085, 0.090),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.ASSUMED,
        material="Stainless Steel 316 / PTFE"
    ))

    # Calculate unballasted CoG to determine precise trim location
    raw_cog = assembly.compute_center_of_gravity()
    raw_mass = assembly.total_mass_kg

    # 12. Ballast Mass: Split into Lower Tray (Z1 = -33.5mm) and Upper Tray (Z2 = +35.0mm)
    # Total ballast mass = 0.414 kg (414g)
    # Allows precision CoG balancing without interfering with avionics stack tiers.
    total_ballast_mass = 0.414
    z_target = - (raw_mass / total_ballast_mass) * raw_cog[2]
    y_target = - (raw_mass / total_ballast_mass) * raw_cog[1]
    x_target = - (raw_mass / total_ballast_mass) * raw_cog[0]

    z1 = -0.0335  # m (Between Camera and OBC)
    z2 = 0.0350   # m (Between Comms and Antenna)

    # Solve: (m1 * z1 + m2 * z2) / total_ballast_mass = z_target, with m1 + m2 = total_ballast_mass
    m1 = total_ballast_mass * (z2 - z_target) / (z2 - z1)
    m2 = total_ballast_mass - m1

    # Add Lower Ballast Tray
    assembly.add_component(MechanicalComponent(
        name="Lower Trim Ballast (Brass Plate)",
        part_id="BAL-01",
        mass_kg=m1,
        centroid_m=(x_target, y_target, z1),
        dimensions_m=(0.060, 0.060, 0.006),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.ASSUMED,
        material="Brass (CuZn39Pb3)"
    ))

    # Add Upper Ballast Tray
    assembly.add_component(MechanicalComponent(
        name="Upper Trim Ballast (Brass Plate)",
        part_id="BAL-02",
        mass_kg=m2,
        centroid_m=(x_target, y_target, z2),
        dimensions_m=(0.060, 0.060, 0.008),
        shape=ShapeType.RECTANGULAR_PRISM,
        source=ParameterSource.ASSUMED,
        material="Brass (CuZn39Pb3)"
    ))

    return assembly
