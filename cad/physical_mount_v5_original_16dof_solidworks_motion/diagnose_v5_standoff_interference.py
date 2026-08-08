"""Exact OCCT diagnosis of the four 1.95 mm PCD14 standoff occurrences."""

from __future__ import annotations

import json
import math
from pathlib import Path

from build123d import Align, Cylinder, Location, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "standoff_1p95_interference_diagnosis.json"


def transformed(shape, matrix):
    trsf = gp_Trsf()
    trsf.SetValues(
        float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(matrix[0][3]),
        float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(matrix[1][3]),
        float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(matrix[2][3]),
    )
    return shape.moved(Location(gp_trsf=trsf))


def common_volume(first, second):
    common = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    common.Build()
    if not common.IsDone():
        raise RuntimeError("OCCT Boolean common failed")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(common.Shape(), props)
    return float(props.Mass())


def spacer(center_radius_mm):
    result = Cylinder(9.975, 1.95, align=(Align.CENTER, Align.CENTER, Align.MIN))
    result = result.cut(Cylinder(center_radius_mm, 2.95, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle = math.radians(angle_deg)
        result = result.cut(
            Cylinder(1.6, 2.95, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
                Location((7.0 * math.cos(angle), 7.0 * math.sin(angle), 0.0))
            )
        )
    return result


def four_posts(outer_radius_mm):
    result = None
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle = math.radians(angle_deg)
        location = Location((7.0 * math.cos(angle), 7.0 * math.sin(angle), 0.0))
        post = Cylinder(outer_radius_mm, 1.95, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(location)
        bore = Cylinder(1.6, 2.95, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(location)
        post = post.cut(bore)
        result = post if result is None else result.fuse(post)
    return result


def main():
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    results = []
    for sid, joint in (("S03", "right_shoulder_yaw"), ("S06", "left_shoulder_yaw"), ("S07", "right_hip_yaw"), ("S08", "left_hip_yaw")):
        servo_id = f"{sid}_STS3250_{joint}"
        standoff_id = f"{sid}_PCD14_CHILD_STANDOFF_{joint}"
        servo_row = by_id[servo_id]
        standoff_row = by_id[standoff_id]
        servo = transformed(import_step(ROOT / servo_row["source"]), servo_row["transform_local_mm_to_world_mm"])
        trials = []
        for radius in (4.8, 5.0, 5.2, 5.3, 5.35):
            candidate = transformed(spacer(radius), standoff_row["transform_local_mm_to_world_mm"])
            trials.append({"center_clearance_radius_mm": radius, "overlap_mm3": common_volume(servo, candidate)})
        for radius in (2.7, 2.4, 2.2, 2.0):
            candidate = transformed(four_posts(radius), standoff_row["transform_local_mm_to_world_mm"])
            trials.append({"four_post_outer_radius_mm": radius, "overlap_mm3": common_volume(servo, candidate)})
        results.append({"joint": joint, "servo": servo_id, "standoff": standoff_id, "trials": trials})
    payload = {"schema": "zeroth01.v5.standoff_interference_diagnosis.v1", "results": results}
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))


if __name__ == "__main__":
    main()
