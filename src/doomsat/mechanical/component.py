"""Mechanical component representation, mass properties, and local inertia models.

All units are strictly SI:
- Dimensions and positions: meters (m)
- Mass: kilograms (kg)
- Inertia: kg * m^2
"""

from enum import Enum
from typing import Tuple, Optional
import numpy as np
from numpy.typing import NDArray
from pydantic import BaseModel, Field


class ShapeType(str, Enum):
    RECTANGULAR_PRISM = "rectangular_prism"
    CYLINDER_X = "cylinder_x"
    CYLINDER_Y = "cylinder_y"
    CYLINDER_Z = "cylinder_z"
    POINT_MASS = "point_mass"


class ParameterSource(str, Enum):
    DATASHEET_DERIVED = "Datasheet-Derived"
    ASSUMED = "Assumed"
    MEASURED = "Measured"


class MechanicalComponent:
    """Represents a physical component within the 1U CubeSat mechanical assembly.

    Attributes:
        name: Human-readable component name.
        part_id: Traceable ID from COMPONENT_REGISTER.md (e.g., 'OBC-01').
        mass_kg: Mass of the component in kilograms.
        centroid_m: (x, y, z) coordinates of the component center of mass [m] relative to 1U chassis geometric center.
        dimensions_m: (dx, dy, dz) bounding box dimensions [m].
        shape: Geometric primitive used for local inertia tensor calculation.
        source: Provenance of the mass and dimensional parameters.
        material: Primary construction material (e.g., 'Al6061-T6', 'FR4', 'Li-ion').
    """

    def __init__(
        self,
        name: str,
        part_id: str,
        mass_kg: float,
        centroid_m: Tuple[float, float, float],
        dimensions_m: Tuple[float, float, float],
        shape: ShapeType = ShapeType.RECTANGULAR_PRISM,
        source: ParameterSource = ParameterSource.DATASHEET_DERIVED,
        material: str = "Various",
    ):
        if mass_kg <= 0.0:
            raise ValueError(f"Mass must be positive: {mass_kg} kg for {name}")

        self.name = name
        self.part_id = part_id
        self.mass_kg = float(mass_kg)
        self.centroid_m = np.array(centroid_m, dtype=np.float64)
        self.dimensions_m = np.array(dimensions_m, dtype=np.float64)
        self.shape = shape
        self.source = source
        self.material = material

    @property
    def aabb_min(self) -> NDArray[np.float64]:
        """Axis-Aligned Bounding Box (AABB) minimum corner [x_min, y_min, z_min] in meters."""
        return self.centroid_m - (self.dimensions_m / 2.0)

    @property
    def aabb_max(self) -> NDArray[np.float64]:
        """Axis-Aligned Bounding Box (AABB) maximum corner [x_max, y_max, z_max] in meters."""
        return self.centroid_m + (self.dimensions_m / 2.0)

    def intersects(self, other: "MechanicalComponent", tolerance_m: float = 1e-4) -> bool:
        """Check if this component's bounding box intersects another component.

        Args:
            other: Another MechanicalComponent.
            tolerance_m: Permissible surface contact clearance in meters (default 0.1 mm).

        Returns:
            True if internal volumes intersect beyond tolerance.
        """
        min_a, max_a = self.aabb_min, self.aabb_max
        min_b, max_b = other.aabb_min, other.aabb_max

        overlap_x = (min_a[0] < max_b[0] - tolerance_m) and (max_a[0] > min_b[0] + tolerance_m)
        overlap_y = (min_a[1] < max_b[1] - tolerance_m) and (max_a[1] > min_b[1] + tolerance_m)
        overlap_z = (min_a[2] < max_b[2] - tolerance_m) and (max_a[2] > min_b[2] + tolerance_m)

        return overlap_x and overlap_y and overlap_z

    def compute_local_inertia(self) -> NDArray[np.float64]:
        """Compute the 3x3 local mass moment of inertia tensor about the component centroid.

        Equations:
        - Rectangular Prism of mass m and dimensions (dx, dy, dz):
            I_xx = (1/12) * m * (dy^2 + dz^2)
            I_yy = (1/12) * m * (dx^2 + dz^2)
            I_zz = (1/12) * m * (dx^2 + dy^2)

        - Cylinder with diameter d, length L:
            Axis along Z (dx = dy = d, dz = L, R = d/2):
                I_zz = (1/2) * m * R^2
                I_xx = I_yy = (1/12) * m * (3*R^2 + L^2)

        Returns:
            3x3 symmetric inertia tensor [kg * m^2].
        """
        m = self.mass_kg
        dx, dy, dz = self.dimensions_m

        if self.shape == ShapeType.POINT_MASS:
            return np.zeros((3, 3), dtype=np.float64)

        if self.shape == ShapeType.RECTANGULAR_PRISM:
            ixx = (1.0 / 12.0) * m * (dy**2 + dz**2)
            iyy = (1.0 / 12.0) * m * (dx**2 + dz**2)
            izz = (1.0 / 12.0) * m * (dx**2 + dy**2)
            return np.diag([ixx, iyy, izz])

        if self.shape == ShapeType.CYLINDER_Y:
            # Diameter is dx (or dz), length is dy
            radius = dx / 2.0
            length = dy
            iyy = 0.5 * m * (radius**2)
            ixx = izz = (1.0 / 12.0) * m * (3.0 * (radius**2) + length**2)
            return np.diag([ixx, iyy, izz])

        if self.shape == ShapeType.CYLINDER_Z:
            radius = dx / 2.0
            length = dz
            izz = 0.5 * m * (radius**2)
            ixx = iyy = (1.0 / 12.0) * m * (3.0 * (radius**2) + length**2)
            return np.diag([ixx, iyy, izz])

        if self.shape == ShapeType.CYLINDER_X:
            radius = dy / 2.0
            length = dx
            ixx = 0.5 * m * (radius**2)
            iyy = izz = (1.0 / 12.0) * m * (3.0 * (radius**2) + length**2)
            return np.diag([ixx, iyy, izz])

        # Default fallback
        ixx = (1.0 / 12.0) * m * (dy**2 + dz**2)
        iyy = (1.0 / 12.0) * m * (dx**2 + dz**2)
        izz = (1.0 / 12.0) * m * (dx**2 + dy**2)
        return np.diag([ixx, iyy, izz])

    def to_dict(self) -> dict:
        """Convert component properties to a serializable dictionary."""
        return {
            "name": self.name,
            "part_id": self.part_id,
            "mass_kg": self.mass_kg,
            "centroid_m": self.centroid_m.tolist(),
            "dimensions_m": self.dimensions_m.tolist(),
            "shape": self.shape.value,
            "source": self.source.value,
            "material": self.material,
        }
