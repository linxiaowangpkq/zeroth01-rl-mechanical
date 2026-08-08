"""Build the positive-volume fixed torso and two real hip-yaw case mounts.

The released body below the measured head seam, all joint axes, and all link
lengths remain unchanged.  Each exact STS3250 receives a 0.30 mm installation
bay plus two torso-integrated lateral ribs terminating at the official rear
M2 holes.  M2x8 screws and the form-fit pocket retain the housing; the output
bridge remains attached to the moving U-hip.
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from build123d import Align, Box, Compound, Location, export_step, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.gp import gp_Trsf

import diagnose_v5_offline_brep_interference as gate
import generate_v5_cad as v5


ROOT = gate.ROOT
PARTS = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "parts"
MANIFEST = PARTS.parent / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
BASE = PARTS / "source_carriers_solid" / "Z_BOT2_MASTER_BODY_SKELETON_SOLID.step"
OUT = PARTS / "source_carriers_sts3250_clearanced" / "torso_mounted_solid.step"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "torso_hip_yaw_clearance.json"
CLEARANCE_MM = 0.30
HEAD_SEAM_Z_MM = 7.4818544090
HEAD_INTERFACE_TRIM_MM = 2.5
HEAD_KEEP_MAX_Z_MM = HEAD_SEAM_Z_MM - HEAD_INTERFACE_TRIM_MM + 0.002


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[row][column]) for row in range(3) for column in range(4)])
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def cut_shape(base, tool):
    operation = BRepAlgoAPI_Cut(base.wrapped, tool.wrapped)
    operation.SetNonDestructive(True)
    operation.SetRunParallel(True)
    operation.SetUseOBB(True)
    operation.SetFuzzyValue(1.0e-7)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT torso hip-yaw installation-bay cut failed")
    return Compound(operation.Shape())


def common_shape(base, tool):
    operation = BRepAlgoAPI_Common(base.wrapped, tool.wrapped)
    operation.SetNonDestructive(True)
    operation.SetRunParallel(True)
    operation.SetUseOBB(True)
    operation.SetFuzzyValue(1.0e-7)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT torso head-seam common failed")
    return Compound(operation.Shape())


def fuse_shape(base, tool):
    operation = BRepAlgoAPI_Fuse(base.wrapped, tool.wrapped)
    operation.SetNonDestructive(True)
    operation.SetRunParallel(True)
    operation.SetUseOBB(True)
    operation.SetFuzzyValue(1.0e-7)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT torso case-mount fuse failed")
    return Compound(operation.Shape())


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    torso_row = by_id["CARRIER_Z_BOT2_MASTER_BODY_SKELETON"]
    torso_world = np.asarray(torso_row["transform_local_mm_to_world_mm"], dtype=float)
    world_to_torso = np.linalg.inv(torso_world)
    solid_torso = import_step(BASE)
    keep_below_head_seam = Box(
        400.0,
        400.0,
        600.0,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).moved(Location((0.0, 0.0, HEAD_KEEP_MAX_Z_MM)))
    torso = common_shape(solid_torso, keep_below_head_seam)

    # The exact purchased STEP establishes these dimensions and the shaft-frame
    # offset.  A rectangular installation bay is printable and avoids copying
    # cosmetic vendor threads into the load-bearing torso.
    clearance_case = Box(
        45.22 + 2.0 * CLEARANCE_MM,
        24.72 + 2.0 * CLEARANCE_MM,
        37.40 + 2.0 * CLEARANCE_MM,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((-10.11, 0.0, -20.75)))

    rows = []
    for servo_id in (
        "S07_STS3250_right_hip_yaw",
        "S08_STS3250_left_hip_yaw",
    ):
        servo_row = by_id[servo_id]
        relative = world_to_torso @ np.asarray(
            servo_row["transform_local_mm_to_world_mm"], dtype=float
        )
        joint_world = np.asarray(servo_row["transform_local_mm_to_world_mm"], dtype=float).copy()
        joint_world[:3, 3] -= joint_world[:3, 2] * 4.0
        joint_relative = world_to_torso @ joint_world
        exact_servo = matrix_shape(import_step(ROOT / str(servo_row["source"])), relative)
        before_mm3 = gate.common_volume(torso, exact_servo)
        torso = cut_shape(torso, matrix_shape(clearance_case, relative))
        mount = matrix_shape(v5.hip_yaw_case_mount_ribs_2xm2(), joint_relative)
        mount_torso_overlap_mm3 = gate.common_volume(torso, mount)
        mount_servo_overlap_mm3 = gate.common_volume(mount, exact_servo)
        torso = fuse_shape(torso, mount)
        after_mm3 = gate.common_volume(torso, exact_servo)
        rows.append(
            {
                "servo_component_id": servo_id,
                "clearance_mm": CLEARANCE_MM,
                "nominal_overlap_before_mm3": before_mm3,
                "nominal_overlap_after_mm3": after_mm3,
                "integrated_mount_overlap_with_torso_mm3": mount_torso_overlap_mm3,
                "integrated_mount_overlap_with_exact_servo_mm3": mount_servo_overlap_mm3,
                "retained_case_fasteners": "2x M2x8 at official negative-X rear-cover pattern",
                "status": "PASS" if (
                    before_mm3 > gate.VOLUME_TOLERANCE_MM3
                    and after_mm3 <= gate.VOLUME_TOLERANCE_MM3
                    and mount_torso_overlap_mm3 > gate.VOLUME_TOLERANCE_MM3
                    and mount_servo_overlap_mm3 <= gate.VOLUME_TOLERANCE_MM3
                ) else "FAIL",
            }
        )

    OUT.parent.mkdir(parents=True, exist_ok=True)
    export_step(torso, OUT)
    box = torso.bounding_box()
    payload = {
        "schema": "zeroth01.v5.torso_hip_yaw_clearance.v1",
        "source": BASE.relative_to(ROOT).as_posix(),
        "target": OUT.relative_to(ROOT).as_posix(),
        "vendor_case_size_mm": [45.22, 24.72, 37.40],
        "clearance_mm": CLEARANCE_MM,
        "head_keep_max_z_mm": HEAD_KEEP_MAX_Z_MM,
        "joint_axes_changed": False,
        "link_lengths_changed": False,
        "released_body_below_head_seam_unchanged_except_internal_case_bays_and_mount_fusion": True,
        "bbox_mm": {
            "min": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
            "max": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
        },
        "solid_count": len(torso.solids()),
        "volume_mm3": float(torso.volume),
        "occt_valid": bool(BRepCheck_Analyzer(torso.wrapped).IsValid()),
        "case_retention": "2x M2x8 per hip-yaw servo through torso-integrated lateral ribs plus 0.30 mm form-fit installation bay; no floating bridge",
        "rows": rows,
    }
    payload["overall"] = "PASS" if payload["occt_valid"] and payload["solid_count"] > 0 and payload["volume_mm3"] > 0.0 and all(row["status"] == "PASS" for row in rows) else "FAIL"
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
