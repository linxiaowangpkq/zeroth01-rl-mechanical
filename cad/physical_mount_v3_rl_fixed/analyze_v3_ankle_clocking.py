"""Exact B-Rep comparison of compact ankle-roll clocking candidates."""

from __future__ import annotations

import json
from pathlib import Path

from build123d import Location, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v3_rl_fixed"
    / "ZEROTH01_V3_RL_FIXED_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)


def transformed(shape, matrix):
    trsf = gp_Trsf()
    trsf.SetValues(
        float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(matrix[0][3]),
        float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(matrix[1][3]),
        float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(matrix[2][3]),
    )
    return shape.moved(Location(gp_trsf=trsf))


def rotation(axis_sign: float, flipped: bool):
    # Mirror the cases: local +X is outboard and local +Y is world up.
    z_axis = ((-1.0 if flipped else 1.0) * axis_sign, 0.0, 0.0)
    x_axis = (0.0, z_axis[0], 0.0)
    y_axis = (
        z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
        z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
        z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
    )
    return [[x_axis[row], y_axis[row], z_axis[row]] for row in range(3)]


def common_volume(first, second) -> float:
    """Return the exact OCCT Boolean-common volume in mm^3."""
    common = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    common.Build()
    if not common.IsDone():
        raise RuntimeError("OCCT Boolean common failed")
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(common.Shape(), properties)
    return float(properties.Mass())


def main() -> int:
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in payload["components"]}
    def installed(component_id: str, override_rotation=None):
        row = by_id[component_id]
        path = ROOT / str(row["source"])
        matrix = [list(values) for values in row["transform_local_mm_to_world_mm"]]
        if override_rotation is not None:
            for index in range(3):
                matrix[index][:3] = override_rotation[index]
        # build123d Shape copies share the wrapped TopoDS placement.  Import a
        # fresh B-Rep for every occurrence so candidate transforms cannot move
        # a cached sibling and corrupt the intersection result.
        return transformed(import_step(path), matrix)

    results = []
    for side, axis_sign, pitch, roll, carrier, horn, foot, sole in (
        (
            "right", -1.0, "S15_STS3250_right_ankle_pitch",
            "S17_STS3250_right_ankle_roll", "RIGHT_ANKLE_ROLL_CARRIER",
            "RIGHT_ANKLE_ROLL_HORN", "CARRIER_FOOT_2", "RIGHT_7MM_LIGHTWEIGHT_SOLE",
        ),
        (
            "left", 1.0, "S16_STS3250_left_ankle_pitch",
            "S18_STS3250_left_ankle_roll", "LEFT_ANKLE_ROLL_CARRIER",
            "LEFT_ANKLE_ROLL_HORN", "CARRIER_FOOT", "LEFT_7MM_LIGHTWEIGHT_SOLE",
        ),
    ):
        # Offset search is dominated by the two adjacent servo bodies.  Foot,
        # sole and full-assembly interactions remain covered by SolidWorks.
        fixed = {pitch: installed(pitch)}
        for flipped in (False, True):
          for offset_mm in (25.0, 28.0, 30.0, 32.0, 35.0):
            candidate_rotation = rotation(axis_sign, flipped)

            def installed_at_offset(name):
                row = by_id[name]
                matrix = [list(values) for values in row["transform_local_mm_to_world_mm"]]
                for index in range(3):
                    matrix[index][:3] = candidate_rotation[index]
                matrix[2][3] -= offset_mm - 30.0
                path = ROOT / str(row["source"])
                return transformed(import_step(path), matrix)

            moving = {name: installed_at_offset(name) for name in (roll, carrier, horn)}
            pairs = []
            candidates = []
            for first_name, first_shape in moving.items():
                candidates.extend(
                    (first_name, first_shape, second_name, second_shape)
                    for second_name, second_shape in fixed.items()
                )
            moving_items = list(moving.items())
            for first_index, (first_name, first_shape) in enumerate(moving_items):
                candidates.extend(
                    (first_name, first_shape, second_name, second_shape)
                    for second_name, second_shape in moving_items[first_index + 1 :]
                )
            for first_name, first_shape, second_name, second_shape in candidates:
                volume = common_volume(first_shape, second_shape)
                if volume > 0.01:
                    pairs.append({
                        "components": [first_name, second_name],
                        "volume_mm3": volume,
                    })
            results.append({
                "side": side,
                "flipped_about_joint_axis": flipped,
                "ankle_roll_offset_mm": offset_mm,
                "physical_interference_count": len(pairs),
                "total_interference_mm3": sum(row["volume_mm3"] for row in pairs),
                "pairs": pairs,
            })
    print(json.dumps(results, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
