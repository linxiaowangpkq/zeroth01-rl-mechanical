"""Export the two hip-yaw installations in their joint-local frames.

This is a design diagnostic, not an assembly substitute.  It makes the fixed
torso, moving U-hip, purchased servo and output stack auditable in the one
coordinate system used to design the permanent case cassette.
"""

from __future__ import annotations

import json

import numpy as np
from build123d import Align, Box, Color, Compound, Location, export_step, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Trsf

import diagnose_v5_offline_brep_interference as gate


ROOT = gate.ROOT
OUT = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "diagnostics"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "hip_yaw_mount_space.json"


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        *[float(matrix[row][column]) for row in range(3) for column in range(4)]
    )
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def bbox(shape):
    box = shape.bounding_box()
    return {
        "min_mm": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "max_mm": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
        "size_mm": [float(box.size.X), float(box.size.Y), float(box.size.Z)],
    }


def common_shape(first, second):
    operation = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    operation.SetNonDestructive(True)
    operation.SetRunParallel(True)
    operation.SetUseOBB(True)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("hip-yaw diagnostic window Boolean failed")
    return Compound(operation.Shape())


def main() -> int:
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    OUT.mkdir(parents=True, exist_ok=True)
    results = []
    for side, sid, uhip in (
        ("right", "S07", "U_HIP_R"),
        ("left", "S08", "U_HIP_L"),
    ):
        joint = f"{side}_hip_yaw"
        servo_id = f"{sid}_STS3250_{joint}"
        bridge_id = f"{sid}_PCD14_OUTPUT_BRIDGE_{joint}"
        sleeve_id = f"{sid}_PCD14_FOUR_SLEEVE_SPACER_{joint}"
        old_bridge_id = f"{sid}_CASE_4XM2_FASTENER_BRIDGE_{joint}"
        servo_world = np.asarray(by_id[servo_id]["transform_local_mm_to_world_mm"], dtype=float)
        # The installed servo is +4 mm along joint-local +Z.  Removing that
        # translation gives the unchanged released joint datum.
        joint_world = servo_world.copy()
        joint_world[:3, 3] -= joint_world[:3, 2] * 4.0
        world_to_joint = np.linalg.inv(joint_world)
        wanted = (
            ("fixed_torso", "CARRIER_Z_BOT2_MASTER_BODY_SKELETON", Color("#E5E7EB")),
            ("moving_uhip", f"CARRIER_{uhip}", Color("#9CA3AF")),
            ("purchased_servo", servo_id, Color("#1677FF")),
            ("output_bridge", bridge_id, Color("#0B4FA2")),
            ("output_sleeves", sleeve_id, Color("#60A5FA")),
            ("invalid_old_case_bridge", old_bridge_id, Color("#EF4444")),
        )
        children = []
        rows = []
        torso_window = Box(
            70.0,
            60.0,
            65.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Location((-10.0, 0.0, -20.0)))
        for label, component_id, color in wanted:
            component = by_id[component_id]
            local_tf = world_to_joint @ np.asarray(
                component["transform_local_mm_to_world_mm"], dtype=float
            )
            shape = matrix_shape(import_step(ROOT / str(component["source"])), local_tf)
            display_shape = common_shape(shape, torso_window) if label == "fixed_torso" else shape
            display_shape.label = f"{label}_joint_window" if label == "fixed_torso" else label
            display_shape.color = color
            children.append(display_shape)
            rows.append(
                {
                    "label": label,
                    "component_id": component_id,
                    "owner_link": component["owner_link"],
                    "bbox_joint_local": bbox(shape),
                }
            )
        assembly = Compound(label=f"{side.upper()}_HIP_YAW_JOINT_LOCAL_DIAGNOSTIC", children=children)
        target = OUT / f"{side}_hip_yaw_joint_local_mount_space.step"
        export_step(assembly, target)
        results.append(
            {
                "side": side,
                "joint_world_from_local": joint_world.tolist(),
                "diagnostic_step": target.relative_to(ROOT).as_posix(),
                "components": rows,
            }
        )
    payload = {
        "schema": "zeroth01.v5.hip_yaw_mount_space.v1",
        "coordinate_convention": "joint local +Z is the servo output axis; purchased case is shifted +4.0 mm; all dimensions mm",
        "results": results,
    }
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
