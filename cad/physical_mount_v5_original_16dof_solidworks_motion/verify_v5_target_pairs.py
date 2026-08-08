"""Fast exact-BRep verification for the last v5 repair pairs."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from build123d import Compound, Location, export_step, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "target_pair_interference.json"
OVERLAPS = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "diagnostic_overlaps"

PAIRS = (
    ("CARRIER_R_ARM_MIRROR_1", "LEFT_FIXED_WRIST_SUPPORT"),
    ("CARRIER_L_ARM_MIRROR_1", "RIGHT_FIXED_WRIST_SUPPORT"),
    ("S07_PCD14_FOUR_SLEEVE_SPACER_right_hip_yaw", "S07_STS3250_right_hip_yaw"),
    ("S08_PCD14_FOUR_SLEEVE_SPACER_left_hip_yaw", "S08_STS3250_left_hip_yaw"),
    ("CARRIER_Z_BOT2_MASTER_BODY_SKELETON", "S07_2XM2X8_CASE_SCREWS_right_hip_yaw"),
    ("CARRIER_Z_BOT2_MASTER_BODY_SKELETON", "S08_2XM2X8_CASE_SCREWS_left_hip_yaw"),
)


def transformed(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[row][column]) for row in range(3) for column in range(4)])
    return shape.moved(Location(gp_trsf=transform))


def bbox(shape):
    box = shape.bounding_box()
    return {
        "min_mm": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
        "max_mm": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
    }


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[row][column]) for row in range(3) for column in range(4)])
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    metadata = {str(row["component_id"]): row for row in data["components"]}
    wanted = {component_id for pair in PAIRS for component_id in pair}
    installed = {}
    for component_id in sorted(wanted):
        row = metadata[component_id]
        print(f"load {component_id}", flush=True)
        installed[component_id] = transformed(
            import_step(ROOT / str(row["source"])), row["transform_local_mm_to_world_mm"]
        )
    OVERLAPS.mkdir(parents=True, exist_ok=True)
    rows = []
    for first_id, second_id in PAIRS:
        print(f"common {first_id} / {second_id}", flush=True)
        operation = BRepAlgoAPI_Common(installed[first_id].wrapped, installed[second_id].wrapped)
        operation.Build()
        if not operation.IsDone():
            raise RuntimeError(f"common failed: {first_id}, {second_id}")
        common = Compound(operation.Shape())
        properties = GProp_GProps()
        BRepGProp.VolumeProperties_s(common.wrapped, properties)
        volume = abs(float(properties.Mass()))
        row = {"component_ids": [first_id, second_id], "volume_mm3": volume}
        if volume > 0.01:
            row["overlap_bbox_world"] = bbox(common)
            first_world = np.asarray(metadata[first_id]["transform_local_mm_to_world_mm"], dtype=float)
            second_world = np.asarray(metadata[second_id]["transform_local_mm_to_world_mm"], dtype=float)
            row["overlap_bbox_first_local"] = bbox(matrix_shape(common, np.linalg.inv(first_world)))
            row["overlap_bbox_second_local"] = bbox(matrix_shape(common, np.linalg.inv(second_world)))
            export_step(common, OVERLAPS / f"{first_id}__{second_id}.step")
        rows.append(row)
    payload = {
        "schema": "zeroth01.v5.target_pair_interference.v1",
        "rows": rows,
        "positive_interference_count": sum(float(row["volume_mm3"]) > 0.01 for row in rows),
    }
    payload["overall"] = "PASS" if payload["positive_interference_count"] == 0 else "FAIL"
    REPORT.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
