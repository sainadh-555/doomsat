# -*- coding: utf-8 -*-
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

COMPONENTS = [
  {
    "name": "Chassis Structure (Al6061-T6)",
    "part_id": "STR-01",
    "mass_kg": 0.12,
    "centroid_m": [
      0.0,
      0.0,
      0.0
    ],
    "dimensions_m": [
      0.1,
      0.1,
      0.1135
    ],
    "shape": "rectangular_prism",
    "source": "Datasheet-Derived",
    "material": "Al6061-T6"
  },
  {
    "name": "Camera Payload (OV5647)",
    "part_id": "PAY-01",
    "mass_kg": 0.0035,
    "centroid_m": [
      0.0,
      0.0,
      -0.042
    ],
    "dimensions_m": [
      0.025,
      0.024,
      0.01
    ],
    "shape": "rectangular_prism",
    "source": "Datasheet-Derived",
    "material": "FR4/Glass/Aluminum"
  },
  {
    "name": "OBC (RPi Zero 2 W)",
    "part_id": "OBC-01",
    "mass_kg": 0.016,
    "centroid_m": [
      0.0,
      0.0,
      -0.025
    ],
    "dimensions_m": [
      0.065,
      0.03,
      0.005
    ],
    "shape": "rectangular_prism",
    "source": "Datasheet-Derived",
    "material": "FR4/Silicon"
  },
  {
    "name": "Battery Cell (18650 Li-ion)",
    "part_id": "EPS-01",
    "mass_kg": 0.045,
    "centroid_m": [
      0.0,
      0.015,
      -0.01
    ],
    "dimensions_m": [
      0.0652,
      0.0185,
      0.0185
    ],
    "shape": "cylinder_x",
    "source": "Datasheet-Derived",
    "material": "Li-ion / Steel Can"
  },
  {
    "name": "EPS PMIC & Power Regulators",
    "part_id": "EPS-03",
    "mass_kg": 0.01,
    "centroid_m": [
      0.0,
      -0.02,
      -0.01
    ],
    "dimensions_m": [
      0.05,
      0.04,
      0.004
    ],
    "shape": "rectangular_prism",
    "source": "Datasheet-Derived",
    "material": "FR4/Copper"
  },
  {
    "name": "ADCS & Environmental Sensors PCB",
    "part_id": "SEN-01",
    "mass_kg": 0.015,
    "centroid_m": [
      0.0,
      0.0,
      0.005
    ],
    "dimensions_m": [
      0.08,
      0.08,
      0.004
    ],
    "shape": "rectangular_prism",
    "source": "Datasheet-Derived",
    "material": "FR4"
  },
  {
    "name": "Comms Controller (ESP32 + SX1276)",
    "part_id": "COM-01",
    "mass_kg": 0.018,
    "centroid_m": [
      0.0,
      0.0,
      0.025
    ],
    "dimensions_m": [
      0.08,
      0.08,
      0.006
    ],
    "shape": "rectangular_prism",
    "source": "Datasheet-Derived",
    "material": "FR4/Shielded Metal"
  },
  {
    "name": "Antenna Deployment Module",
    "part_id": "ANT-01",
    "mass_kg": 0.018,
    "centroid_m": [
      0.0,
      0.0,
      0.045
    ],
    "dimensions_m": [
      0.06,
      0.06,
      0.01
    ],
    "shape": "rectangular_prism",
    "source": "Assumed",
    "material": "Delrin/Nitinol"
  },
  {
    "name": "Solar Panel Array (+X, -X, +Y, -Y, +Z)",
    "part_id": "EPS-02",
    "mass_kg": 0.055,
    "centroid_m": [
      0.0,
      0.0,
      0.002
    ],
    "dimensions_m": [
      0.098,
      0.098,
      0.0015
    ],
    "shape": "rectangular_prism",
    "source": "Datasheet-Derived",
    "material": "GaAs / PCB"
  },
  {
    "name": "Sun Sensor Photodiodes (6x BPW83)",
    "part_id": "SEN-03",
    "mass_kg": 0.0015,
    "centroid_m": [
      0.0,
      0.0,
      0.0
    ],
    "dimensions_m": [
      0.008,
      0.006,
      0.003
    ],
    "shape": "point_mass",
    "source": "Datasheet-Derived",
    "material": "Silicon/Epoxy"
  },
  {
    "name": "Harness, Standoffs & Fasteners",
    "part_id": "HAR-01",
    "mass_kg": 0.03,
    "centroid_m": [
      0.0,
      0.0,
      0.0
    ],
    "dimensions_m": [
      0.085,
      0.085,
      0.09
    ],
    "shape": "rectangular_prism",
    "source": "Assumed",
    "material": "Stainless Steel 316 / PTFE"
  },
  {
    "name": "Lower Trim Ballast (Brass Plate)",
    "part_id": "BAL-01",
    "mass_kg": 0.2166131386861314,
    "centroid_m": [
      -0.0,
      -0.0011473429951690819,
      -0.0335
    ],
    "dimensions_m": [
      0.06,
      0.06,
      0.006
    ],
    "shape": "rectangular_prism",
    "source": "Assumed",
    "material": "Brass (CuZn39Pb3)"
  },
  {
    "name": "Upper Trim Ballast (Brass Plate)",
    "part_id": "BAL-02",
    "mass_kg": 0.19738686131386857,
    "centroid_m": [
      -0.0,
      -0.0011473429951690819,
      0.035
    ],
    "dimensions_m": [
      0.06,
      0.06,
      0.008
    ],
    "shape": "rectangular_prism",
    "source": "Assumed",
    "material": "Brass (CuZn39Pb3)"
  }
]

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
    color_map = {
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
    }

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
