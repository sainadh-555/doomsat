"""Unit tests for DOOMSat 1U mechanical assembly, mass properties, and inertia tensor."""

import math
from pathlib import Path
import numpy as np
import pytest

from doomsat.mechanical import (
    MechanicalComponent,
    ShapeType,
    ParameterSource,
    CubeSatAssembly,
    build_reference_doomsat_1u,
    export_assembly_json,
    generate_freecad_script,
)
from doomsat.core.constants import CUBESAT_1U_MAX_MASS_KG


def test_component_creation_and_local_inertia():
    """Verify component local inertia tensor calculations against analytical formulas."""
    # Rectangular prism: mass 2.0 kg, dx=0.1, dy=0.2, dz=0.3
    m, dx, dy, dz = 2.0, 0.1, 0.2, 0.3
    comp = MechanicalComponent(
        name="TestPrism",
        part_id="TST-01",
        mass_kg=m,
        centroid_m=(0.0, 0.0, 0.0),
        dimensions_m=(dx, dy, dz),
        shape=ShapeType.RECTANGULAR_PRISM
    )

    i_local = comp.compute_local_inertia()

    # Ixx = 1/12 * m * (dy^2 + dz^2) = (1/12) * 2 * (0.04 + 0.09) = 0.021666...
    expected_ixx = (1.0 / 12.0) * m * (dy**2 + dz**2)
    expected_iyy = (1.0 / 12.0) * m * (dx**2 + dz**2)
    expected_izz = (1.0 / 12.0) * m * (dx**2 + dy**2)

    assert math.isclose(i_local[0, 0], expected_ixx, rel_tol=1e-10)
    assert math.isclose(i_local[1, 1], expected_iyy, rel_tol=1e-10)
    assert math.isclose(i_local[2, 2], expected_izz, rel_tol=1e-10)
    assert np.allclose(i_local - np.diag(np.diag(i_local)), 0.0)


def test_cylinder_local_inertia():
    """Verify cylindrical component inertia (such as 18650 battery cell)."""
    # Cylinder along X-axis: mass 0.045 kg, length=0.065 m, diameter=0.018 m (radius=0.009 m)
    m = 0.045
    radius = 0.009
    length = 0.065

    comp = MechanicalComponent(
        name="Battery18650",
        part_id="EPS-01",
        mass_kg=m,
        centroid_m=(0.0, 0.0, 0.0),
        dimensions_m=(length, 2.0 * radius, 2.0 * radius),
        shape=ShapeType.CYLINDER_X
    )

    i_local = comp.compute_local_inertia()
    expected_ixx = 0.5 * m * (radius**2)
    expected_iyy = (1.0 / 12.0) * m * (3.0 * radius**2 + length**2)

    assert math.isclose(i_local[0, 0], expected_ixx, rel_tol=1e-10)
    assert math.isclose(i_local[1, 1], expected_iyy, rel_tol=1e-10)
    assert math.isclose(i_local[2, 2], expected_iyy, rel_tol=1e-10)


def test_invalid_negative_mass():
    """Verify that zero or negative mass raises a ValueError."""
    with pytest.raises(ValueError):
        MechanicalComponent(
            name="Invalid",
            part_id="ERR-01",
            mass_kg=-0.5,
            centroid_m=(0, 0, 0),
            dimensions_m=(0.1, 0.1, 0.1)
        )


def test_parallel_axis_theorem_benchmark():
    """Verify Parallel Axis Theorem against a classic 2-point-mass analytical benchmark."""
    assembly = CubeSatAssembly(name="TwoPointTest")

    # Mass 1 at x = -1.0 m, Mass 2 at x = +1.0 m, each 1.0 kg
    c1 = MechanicalComponent(
        name="M1",
        part_id="M1",
        mass_kg=1.0,
        centroid_m=(-1.0, 0.0, 0.0),
        dimensions_m=(0.01, 0.01, 0.01),
        shape=ShapeType.POINT_MASS
    )
    c2 = MechanicalComponent(
        name="M2",
        part_id="M2",
        mass_kg=1.0,
        centroid_m=(1.0, 0.0, 0.0),
        dimensions_m=(0.01, 0.01, 0.01),
        shape=ShapeType.POINT_MASS
    )

    assembly.add_component(c1)
    assembly.add_component(c2)

    # CoG must be at (0, 0, 0)
    cog = assembly.compute_center_of_gravity()
    assert np.allclose(cog, [0.0, 0.0, 0.0], atol=1e-12)

    # Inertia about CoG:
    # Ixx = sum(m * (y^2 + z^2)) = 0
    # Iyy = sum(m * (x^2 + z^2)) = 1*(1^2) + 1*((-1)^2) = 2.0 kg*m^2
    # Izz = sum(m * (x^2 + y^2)) = 1*(1^2) + 1*((-1)^2) = 2.0 kg*m^2
    i_tensor = assembly.compute_inertia_tensor()
    assert math.isclose(i_tensor[0, 0], 0.0, abs_tol=1e-12)
    assert math.isclose(i_tensor[1, 1], 2.0, rel_tol=1e-10)
    assert math.isclose(i_tensor[2, 2], 2.0, rel_tol=1e-10)


def test_reference_assembly_mass_and_margin():
    """Verify that DOOMSat reference assembly meets launch mass limits and margin."""
    assembly = build_reference_doomsat_1u()

    assert len(assembly.components) == 13
    # Mass must be strictly <= 1.330 kg
    assert assembly.total_mass_kg <= CUBESAT_1U_MAX_MASS_KG
    # Mass margin must be >= 15% (DOOMSat baseline has ~43.9% margin)
    assert assembly.mass_margin_pct >= 15.0
    assert math.isclose(assembly.total_mass_kg, 0.746, rel_tol=1e-2)


def test_reference_assembly_cog_within_spec():
    """Verify that the Center of Gravity satisfies REQ-MEC-03 (<= 20.0 mm from center)."""
    assembly = build_reference_doomsat_1u()
    is_compliant, dist = assembly.verify_cog_limit(max_offset_m=0.020)

    assert is_compliant
    # With trim ballast, offset should be negligible (< 1.0 mm)
    assert dist < 0.001


def test_reference_assembly_inertia_tensor():
    """Verify physical properties of the reference assembly inertia tensor."""
    assembly = build_reference_doomsat_1u()
    props = assembly.verify_inertia_properties()

    assert props["symmetric"], "Inertia tensor must be symmetric"
    assert props["positive_definite"], "Inertia tensor must be positive definite"
    assert props["triangle_inequalities"], "Principal moments must satisfy triangle inequalities"

    i_tensor = assembly.compute_inertia_tensor()
    # Typical 1U CubeSat moments of inertia are on the order of 1e-3 kg*m^2 (0.0005 - 0.003)
    assert 0.0003 < i_tensor[0, 0] < 0.005
    assert 0.0003 < i_tensor[1, 1] < 0.005
    assert 0.0003 < i_tensor[2, 2] < 0.005


def test_principal_moments_ordering_and_axes():
    """Verify eigenvalue decomposition produces ascending moments and orthogonal axes."""
    assembly = build_reference_doomsat_1u()
    moments, axes = assembly.compute_principal_moments()

    # Ascending order: I1 <= I2 <= I3
    assert moments[0] <= moments[1] <= moments[2]

    # Principal axes matrix must be orthogonal: V * V^T = I
    assert np.allclose(np.dot(axes, axes.T), np.eye(3), atol=1e-10)


def test_envelope_containment_and_no_collisions():
    """Verify that all components fit inside 1U envelope and do not collide."""
    assembly = build_reference_doomsat_1u()

    contained, violations = assembly.check_envelope_containment()
    assert contained, f"Envelope violations found: {violations}"

    zero_collisions, collision_pairs = assembly.check_internal_collisions()
    assert zero_collisions, f"Collisions detected: {collision_pairs}"


def test_cad_export_execution(tmp_path: Path):
    """Verify JSON specification and FreeCAD script generation."""
    assembly = build_reference_doomsat_1u()

    json_file = export_assembly_json(assembly, tmp_path / "structure_spec.json")
    fc_file = generate_freecad_script(assembly, tmp_path / "generate_doomsat_1u.py")

    assert json_file.exists()
    assert fc_file.exists()
    assert json_file.stat().st_size > 500
    assert "DOOMSat_1U_DigitalTwin" in fc_file.read_text()
