"""Screen real STS3250 case-fastener patterns against the moving U-hip.

The old v4 bridge used the official drawing coordinates but stopped at the
case bounding plane.  This diagnostic distinguishes an axial recess from a
wrong pattern and rejects every fixed screw that crosses the moving carrier.
"""

from __future__ import annotations

import json

import numpy as np
from build123d import Align, Compound, Cylinder, Location, import_step
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.gp import gp_Trsf

import diagnose_v5_offline_brep_interference as gate


ROOT = gate.ROOT
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "hip_yaw_fastener_pattern_screen.json"
RADIUS_MM = 0.95
Z_START_MM = -39.45
Z_ENDS_MM = (-35.45, -34.00, -32.85, -31.35)
PATTERNS = {
    "official_sts3250_drawing_4xm2": (
        (-28.50, -10.25),
        (-28.50, 10.25),
        (8.30, -10.25),
        (8.30, 10.25),
    ),
    "step_case_screw_axes": (
        (-29.71, -9.35),
        (-29.71, 9.35),
        (10.14, -10.00),
        (10.14, 10.00),
    ),
    "step_front_cover_hole_axes": (
        (-14.31, -10.25),
        (-14.31, 10.25),
        (6.39, -10.25),
        (6.39, 10.25),
    ),
}


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        *[float(matrix[row][column]) for row in range(3) for column in range(4)]
    )
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def distance_mm(first, second) -> float:
    operation = BRepExtrema_DistShapeShape(first.wrapped, second.wrapped)
    operation.Perform()
    if not operation.IsDone():
        raise RuntimeError("distance operation failed")
    return float(operation.Value())


def pin(x_mm: float, y_mm: float, z_end_mm: float):
    return Cylinder(
        RADIUS_MM,
        z_end_mm - Z_START_MM,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    ).moved(Location((x_mm, y_mm, Z_START_MM)))


def main() -> int:
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    servo_row = by_id["S07_STS3250_right_hip_yaw"]
    servo_world = np.asarray(servo_row["transform_local_mm_to_world_mm"], dtype=float)
    joint_world = servo_world.copy()
    joint_world[:3, 3] -= joint_world[:3, 2] * 4.0
    world_to_joint = np.linalg.inv(joint_world)

    def local_component(component_id: str):
        row = by_id[component_id]
        local_tf = world_to_joint @ np.asarray(row["transform_local_mm_to_world_mm"], dtype=float)
        return matrix_shape(import_step(ROOT / str(row["source"])), local_tf)

    servo = local_component("S07_STS3250_right_hip_yaw")
    uhip = local_component("CARRIER_U_HIP_R")
    torso = local_component("CARRIER_Z_BOT2_MASTER_BODY_SKELETON")
    rows = []
    for pattern_name, axes in PATTERNS.items():
        for z_end_mm in Z_ENDS_MM:
            print(f"{pattern_name} z_end={z_end_mm:.2f}", flush=True)
            pin_rows = []
            for x_mm, y_mm in axes:
                candidate = pin(x_mm, y_mm, z_end_mm)
                servo_overlap = gate.common_volume(candidate, servo)
                uhip_overlap = gate.common_volume(candidate, uhip)
                torso_overlap = gate.common_volume(candidate, torso)
                pin_rows.append(
                    {
                        "axis_xy_mm": [x_mm, y_mm],
                        "servo_distance_mm": distance_mm(candidate, servo),
                        "servo_overlap_mm3": servo_overlap,
                        "moving_uhip_overlap_mm3": uhip_overlap,
                        "fixed_torso_overlap_mm3": torso_overlap,
                    }
                )
            rows.append(
                {
                    "pattern": pattern_name,
                    "z_start_mm": Z_START_MM,
                    "z_end_mm": z_end_mm,
                    "pin_radius_mm": RADIUS_MM,
                    "pins": pin_rows,
                    "max_servo_distance_mm": max(row["servo_distance_mm"] for row in pin_rows),
                    "total_servo_overlap_mm3": sum(row["servo_overlap_mm3"] for row in pin_rows),
                    "total_moving_uhip_overlap_mm3": sum(row["moving_uhip_overlap_mm3"] for row in pin_rows),
                    "total_fixed_torso_overlap_mm3": sum(row["fixed_torso_overlap_mm3"] for row in pin_rows),
                }
            )
    payload = {
        "schema": "zeroth01.v5.hip_yaw_fastener_pattern_screen.v1",
        "source_truth": {
            "official_pattern": "user-supplied/FEETECH STS3250 dimension drawing",
            "step_patterns": "analytic cylindrical axes extracted from the exact step.parts ST-3235M-family STEP",
        },
        "rows": rows,
        "acceptance": "A fixed case fastener must reach the purchased case (distance <= 0.05 mm) and have moving-U-hip overlap <= 0.01 mm^3. Torso overlap is not accepted; any selected pattern requires explicit matching torso holes.",
    }
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
