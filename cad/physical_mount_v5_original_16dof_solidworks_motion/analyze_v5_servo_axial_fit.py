"""Find the minimum collision-free STS3250 axial installation per joint.

Only translation along the servo output axis is considered.  Lateral moves
would move the shaft away from the released Zeroth-01 joint axis and are not
mechanically admissible.  The result decides whether an output spacer can
solve the fit or a released carrier truly needs a pocket revision.
"""

from __future__ import annotations

import json
from pathlib import Path

from build123d import Compound, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "servo_axial_fit.json"
# Four mechanically meaningful stack heights keep the complete scan below the
# command time budget: current placement, vendor output-face difference, the
# existing proven hip spacer, and one conservative upper bound.
OFFSETS_MM = (0.0, 2.05, 4.0, 6.0)


def transformed(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(matrix[0][3]),
        float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(matrix[1][3]),
        float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(matrix[2][3]),
    )
    # Deep-copy the already imported B-Rep.  Re-reading the 12 MB vendor STEP
    # for every offset made a 16-joint scan exceed the user's time limit.
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def translated_along_local_z(matrix, offset_mm):
    result = [list(row) for row in matrix]
    for index in range(3):
        result[index][3] += float(matrix[index][2]) * offset_mm
    return result


def bbox_overlap(first, second) -> bool:
    a, b = first.bounding_box(), second.bounding_box()
    amin, amax = (a.min.X, a.min.Y, a.min.Z), (a.max.X, a.max.Y, a.max.Z)
    bmin, bmax = (b.min.X, b.min.Y, b.min.Z), (b.max.X, b.max.Y, b.max.Z)
    return all(amin[i] < bmax[i] - 1.0e-5 and bmin[i] < amax[i] - 1.0e-5 for i in range(3))


def common_volume(first, second):
    if not bbox_overlap(first, second):
        return 0.0
    operation = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT Boolean common failed")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(operation.Shape(), props)
    return float(props.Mass())


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    carrier_rows = {
        str(row["owner_link"]): row
        for row in data["components"]
        if row["role"] == "source_load_bearing_carrier"
    }
    carriers = {
        owner: transformed(import_step(ROOT / str(row["source"])), row["transform_local_mm_to_world_mm"])
        for owner, row in carrier_rows.items()
    }
    exact_source = ROOT / str(next(
        row["source"] for row in data["components"] if row["role"] == "purchased_exact_sts3250"
    ))
    exact_shape = import_step(exact_source)
    results = []
    for joint_index, spec in enumerate(data["joint_specs"], start=1):
        joint = str(spec["name"])
        sid = str(spec["id"])
        servo_row = by_id[f"{sid}_STS3250_{joint}"]
        ring_owner = str(by_id[f"{sid}_PARENT_THRUST_RING_{joint}"]["owner_link"])
        bridge_owner = str(by_id[f"{sid}_PCD14_OUTPUT_BRIDGE_{joint}"]["owner_link"])
        base = servo_row["transform_local_mm_to_world_mm"]
        print(f"joint {joint_index}/16 {joint}", flush=True)
        trials = []
        for offset in OFFSETS_MM:
            servo = transformed(exact_shape, translated_along_local_z(base, offset))
            housing_overlap = common_volume(servo, carriers[ring_owner])
            output_overlap = common_volume(servo, carriers[bridge_owner])
            trials.append({
                "additional_axial_offset_mm": offset,
                "housing_owner_overlap_mm3": housing_overlap,
                "output_owner_overlap_mm3": output_overlap,
                "total_two_carrier_overlap_mm3": housing_overlap + output_overlap,
            })
        best = min(trials, key=lambda row: row["total_two_carrier_overlap_mm3"])
        results.append({
            "joint": joint,
            "housing_owner": ring_owner,
            "output_owner": bridge_owner,
            "manifest_existing_axial_shim_mm": float(data.get("servo_axial_shims_mm", {}).get(joint, 0.0)),
            "best_trial": best,
            "trials": trials,
        })
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.servo_axial_fit.v1",
        "offsets_mm": list(OFFSETS_MM),
        "results": results,
        "all_zero_by_axial_shift_only": all(row["best_trial"]["total_two_carrier_overlap_mm3"] <= 0.01 for row in results),
    }
    payload["overall"] = "PASS" if payload["all_zero_by_axial_shift_only"] else "FAIL"
    REPORT.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "overall": payload["overall"],
        "best": [{"joint": row["joint"], **row["best_trial"]} for row in results],
    }, ensure_ascii=False, indent=2), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
