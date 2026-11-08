"""Mechanical assembly, mass properties, and inertia tensor package for DOOMSat."""

from doomsat.mechanical.component import (
    MechanicalComponent,
    ShapeType,
    ParameterSource,
)
from doomsat.mechanical.assembly import (
    CubeSatAssembly,
    build_reference_doomsat_1u,
)
from doomsat.mechanical.cad_export import (
    export_assembly_json,
    generate_freecad_script,
)

__all__ = [
    "MechanicalComponent",
    "ShapeType",
    "ParameterSource",
    "CubeSatAssembly",
    "build_reference_doomsat_1u",
    "export_assembly_json",
    "generate_freecad_script",
]
