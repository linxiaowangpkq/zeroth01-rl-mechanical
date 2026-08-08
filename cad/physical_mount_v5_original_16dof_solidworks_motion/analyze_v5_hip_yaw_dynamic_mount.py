"""Dynamic screen for the two-screw, form-fit hip-yaw case mount."""

from __future__ import annotations

import json
import math

import numpy as np
from build123d import Align, Box, Compound, Cylinder, Location, import_step
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf

import diagnose_v5_offline_brep_interference as gate
import generate_v5_cad as v5


ROOT = gate.ROOT
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "hip_yaw_dynamic_mount_screen.json"
M2_AXES_MM = ((-28.50, -10.25), (-28.50, 10.25))
M2_RADIUS_MM = 0.95
M2_Z_START_MM = -39.45
M2_Z_END_MM = -31.35
ANGLE_SAMPLES = 9


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        *[float(matrix[row][column]) for row in range(3) for column in range(4)]
    )
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def rotated_z(shape, angle_rad: float):
    transform = gp_Trsf()
    transform.SetRotation(
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
        angle_rad,
    )
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def fixed_screws():
    return v5.hip_yaw_case_screws_2xm2x8()


def candidate_anchor_bar():
    return v5.hip_yaw_case_mount_ribs_2xm2()


def main() -> int:
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    results = []
    for side, sid, uhip in (
        ("right", "S07", "U_HIP_R"),
        ("left", "S08", "U_HIP_L"),
    ):
        joint = f"{side}_hip_yaw"
        servo_id = f"{sid}_STS3250_{joint}"
        spec = next(row for row in data["joint_specs"] if row["name"] == joint)
        servo_world = np.asarray(by_id[servo_id]["transform_local_mm_to_world_mm"], dtype=float)
        joint_world = servo_world.copy()
        joint_world[:3, 3] -= joint_world[:3, 2] * 4.0
        world_to_joint = np.linalg.inv(joint_world)

        def local_component(component_id: str):
            row = by_id[component_id]
            local_tf = world_to_joint @ np.asarray(
                row["transform_local_mm_to_world_mm"], dtype=float
            )
            return matrix_shape(import_step(ROOT / str(row["source"])), local_tf)

        servo = local_component(servo_id)
        torso = local_component("CARRIER_Z_BOT2_MASTER_BODY_SKELETON")
        child_shapes = [
            local_component(str(row["component_id"]))
            for row in data["components"]
            if str(row["owner_link"]) == uhip
        ]
        child_rigid = Compound(children=child_shapes)
        screws = fixed_screws()
        anchor = candidate_anchor_bar()
        lower, upper = (float(value) for value in spec["limits"])
        angles = np.linspace(lower, upper, ANGLE_SAMPLES)
        poses = []
        for angle_rad in angles:
            moving = rotated_z(child_rigid, float(angle_rad))
            poses.append(
                {
                    "angle_rad": float(angle_rad),
                    "angle_deg": math.degrees(float(angle_rad)),
                    "fixed_screw_vs_moving_volume_mm3": gate.common_volume(screws, moving),
                    "anchor_bar_vs_moving_volume_mm3": gate.common_volume(anchor, moving),
                }
            )
        results.append(
            {
                "side": side,
                "joint": joint,
                "limits_rad": [lower, upper],
                "m2_axes_joint_local_mm": [list(axis) for axis in M2_AXES_MM],
                "m2_engagement_z_mm": [M2_Z_START_MM, M2_Z_END_MM],
                "screw_vs_servo_volume_mm3": gate.common_volume(screws, servo),
                "anchor_bar_vs_servo_volume_mm3": gate.common_volume(anchor, servo),
                "anchor_bar_vs_fixed_torso_volume_mm3": gate.common_volume(anchor, torso),
                "poses": poses,
                "max_fixed_screw_vs_moving_volume_mm3": max(
                    row["fixed_screw_vs_moving_volume_mm3"] for row in poses
                ),
                "max_anchor_bar_vs_moving_volume_mm3": max(
                    row["anchor_bar_vs_moving_volume_mm3"] for row in poses
                ),
            }
        )
    for row in results:
        row["status"] = "PASS" if (
            row["screw_vs_servo_volume_mm3"] <= gate.VOLUME_TOLERANCE_MM3
            and row["anchor_bar_vs_servo_volume_mm3"] <= gate.VOLUME_TOLERANCE_MM3
            and row["anchor_bar_vs_fixed_torso_volume_mm3"] > gate.VOLUME_TOLERANCE_MM3
            and row["max_fixed_screw_vs_moving_volume_mm3"] <= gate.VOLUME_TOLERANCE_MM3
            and row["max_anchor_bar_vs_moving_volume_mm3"] <= gate.VOLUME_TOLERANCE_MM3
        ) else "FAIL"
    payload = {
        "schema": "zeroth01.v5.hip_yaw_dynamic_mount_screen.v1",
        "design_intent": "two official-pattern M2x8 rear fasteners provide retention; two narrow lateral ribs connect the bosses to measured fixed torso side-wall material and the close-fit installation bay carries case reaction",
        "angle_samples_per_joint": ANGLE_SAMPLES,
        "rows": results,
        "overall": "PASS" if all(row["status"] == "PASS" for row in results) else "FAIL",
    }
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
