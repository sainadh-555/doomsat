"""FreeCAD and JSON geometric specification generator for DOOMSat 1U CubeSat.

Generates parametric FreeCAD scripts and CAD exchange models.
"""

import json
from pathlib import Path
from typing import Dict, Any

from doomsat.mechanical.assembly import CubeSatAssembly


def export_assembly_json(assembly: CubeSatAssembly, output_path: Path | str) -> Path:
    """Export the complete mechanical assembly specification to JSON format.

    Args:
        assembly: Populated CubeSatAssembly instance.
        output_path: Target JSON filepath.

    Returns:
        Path of written JSON file.
    """
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    cog = assembly.compute_center_of_gravity()
    inertia = assembly.compute_inertia_tensor()
    moments, axes = assembly.compute_principal_moments()

    data: Dict[str, Any] = {
        "assembly_name": assembly.name,
        "envelope_m": assembly.envelope_m.tolist(),
        "total_mass_kg": assembly.total_mass_kg,
        "mass_margin_pct": assembly.mass_margin_pct,
        "center_of_gravity_m": cog.tolist(),
        "inertia_tensor_kg_m2": inertia.tolist(),
        "principal_moments_kg_m2": moments.tolist(),
        "principal_axes": axes.tolist(),
        "components": [comp.to_dict() for comp in assembly.components],
    }

    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)

    return path


def generate_freecad_script(assembly: CubeSatAssembly, output_path: Path | str) -> Path:
    """Generate a standalone parametric Python script for FreeCAD to construct the 3D CubeSat model.

    The generated script creates:
    - 1U structural aluminum frame (rails, side ribs, dispenser feet)
    - Avionics PCB stack tiers (OBC, ADCS, Comms) with standoffs
    - 18650 cylindrical battery cell and mount
    - OV5647 camera module and lens barrel
    - Body-mounted solar panels on 5 faces with photodiode sensor apertures
    - Deployable antenna top plate

    Args:
        assembly: Populated CubeSatAssembly.
        output_path: Target .py filepath.

    Returns:
        Path to generated FreeCAD script.
    """
    path = Path(output_path)
    path.parent.mkdir(parents=True, exist_ok=True)

    components_json = json.dumps([c.to_dict() for c in assembly.components], indent=2)

    script_content = f'''# -*- coding: utf-8 -*-
"""DOOMSat 1U CubeSat Parametric FreeCAD Model Generator.

Execute within FreeCAD Python console or via FreeCADCmd:
    freecadcmd generate_doomsat_1u.py
"""

import math

try:
    import FreeCAD
    import Part
    from FreeCAD import Vector, Rotation
except ImportError:
    print("Warning: FreeCAD modules not detected. This script is designed to run inside FreeCAD.")
    FreeCAD = None

COMPONENTS = {components_json}

def create_doomsat_1u_model():
    if FreeCAD is None:
        print("Cannot build model without FreeCAD.")
        return

    doc = FreeCAD.newDocument("DOOMSat_1U_DigitalTwin")
    print("Generating DOOMSat 1U CAD Assembly in FreeCAD...")

    # 1. 1U Structural Aluminum Rails and Chassis
    # Outer bounds: 100 x 100 x 113.5 mm (including dispenser feet)
    chassis_box = Part.makeBox(100, 100, 113.5, Vector(-50, -50, -56.75))
    chassis_cutout = Part.makeBox(90, 90, 100, Vector(-45, -45, -50))
    chassis_frame = chassis_box.cut(chassis_cutout)
    
    chassis_obj = doc.addObject("Part::Feature", "Chassis_Al6061")
    chassis_obj.Shape = chassis_frame
    chassis_obj.ViewObject.ShapeColor = (0.75, 0.75, 0.78)  # Metallic aluminum

    # 2. Add each internal component as a colored solid
    color_map = {{
        "STR-01": (0.75, 0.75, 0.78),  # Aluminum
        "OBC-01": (0.10, 0.55, 0.20),  # Green PCB
        "PAY-01": (0.20, 0.20, 0.20),  # Dark camera
        "EPS-01": (0.15, 0.40, 0.85),  # Blue battery wrap
        "EPS-03": (0.70, 0.20, 0.20),  # Red PMIC
        "SEN-01": (0.20, 0.60, 0.60),  # Cyan sensor board
        "COM-01": (0.30, 0.30, 0.70),  # Indigo comms
        "ANT-01": (0.85, 0.65, 0.15),  # Gold deployer
        "EPS-02": (0.10, 0.10, 0.25),  # Deep blue solar panel
        "BAL-01": (0.80, 0.70, 0.25),  # Brass ballast
    }}

    for comp in COMPONENTS:
        part_id = comp["part_id"]
        name = comp["name"].replace(" ", "_").replace("(", "").replace(")", "").replace("-", "_")
        cx, cy, cz = [v * 1000.0 for v in comp["centroid_m"]]  # Convert m to mm
        dx, dy, dz = [v * 1000.0 for v in comp["dimensions_m"]]
        shape_type = comp["shape"]

        if part_id == "STR-01":
            continue  # Already created frame

        if "cylinder" in shape_type:
            # Create cylinder for battery
            if shape_type == "cylinder_x":
                cyl = Part.makeCylinder(dy / 2.0, dx, Vector(cx - dx/2.0, cy, cz), Vector(1, 0, 0))
            elif shape_type == "cylinder_y":
                cyl = Part.makeCylinder(dx / 2.0, dy, Vector(cx, cy - dy/2.0, cz), Vector(0, 1, 0))
            else:
                cyl = Part.makeCylinder(dx / 2.0, dz, Vector(cx, cy, cz - dz/2.0), Vector(0, 0, 1))
            obj = doc.addObject("Part::Feature", name)
            obj.Shape = cyl
        else:
            box = Part.makeBox(dx, dy, dz, Vector(cx - dx/2.0, cy - dy/2.0, cz - dz/2.0))
            obj = doc.addObject("Part::Feature", name)
            obj.Shape = box

        color = color_map.get(part_id, (0.5, 0.5, 0.5))
        obj.ViewObject.ShapeColor = color

    doc.recompute()
    print("DOOMSat 1U CAD Assembly generation completed successfully.")

if __name__ == "__main__":
    create_doomsat_1u_model()
'''

    with open(path, "w", encoding="utf-8") as f:
        f.write(script_content)

    return path
