"""Measure exact nearest fixed-torso support for the hip-yaw rear bosses."""

from __future__ import annotations

import json

import numpy as np
from build123d import Align, Compound, Cylinder, Location, import_step
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Trsf

import diagnose_v5_offline_brep_interference as gate


ROOT = gate.ROOT
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "hip_mount_nearest_torso.json"
M2_AXES_MM = ((-28.50, -10.25), (-28.50, 10.25))


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        *[float(matrix[row][column]) for row in range(3) for column in range(4)]
    )
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def distance(first, second) -> dict[str, object]:
    operation = BRepExtrema_DistShapeShape(first.wrapped, second.wrapped)
    operation.Perform()
    if not operation.IsDone() or operation.NbSolution() < 1:
        raise RuntimeError("OCCT distance query failed")
    point_first = operation.PointOnShape1(1)
    point_second = operation.PointOnShape2(1)
    return {
        "distance_mm": float(operation.Value()),
        "point_on_boss_mm": [point_first.X(), point_first.Y(), point_first.Z()],
        "point_on_torso_mm": [point_second.X(), point_second.Y(), point_second.Z()],
    }


def main() -> int:
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    rows = []
    for side, servo_id in (
        ("right", "S07_STS3250_right_hip_yaw"),
        ("left", "S08_STS3250_left_hip_yaw"),
    ):
        servo_world = np.asarray(by_id[servo_id]["transform_local_mm_to_world_mm"], dtype=float)
        joint_world = servo_world.copy()
        joint_world[:3, 3] -= joint_world[:3, 2] * 4.0
        world_to_joint = np.linalg.inv(joint_world)
        torso_row = by_id["CARRIER_Z_BOT2_MASTER_BODY_SKELETON"]
        torso = matrix_shape(
            import_step(ROOT / str(torso_row["source"])),
            world_to_joint @ np.asarray(
                torso_row["transform_local_mm_to_world_mm"], dtype=float
            ),
        )
        bosses = []
        for x_mm, y_mm in M2_AXES_MM:
            boss = Cylinder(
                3.2,
                4.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Location((x_mm, y_mm, -39.45)))
            bosses.append({"axis_xy_mm": [x_mm, y_mm], **distance(boss, torso)})
        rows.append({"side": side, "bosses": bosses})
    payload = {
        "schema": "zeroth01.v5.hip_mount_nearest_torso.v1",
        "boss_radius_mm": 3.2,
        "boss_z_span_mm": [-39.45, -35.45],
        "rows": rows,
    }
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
