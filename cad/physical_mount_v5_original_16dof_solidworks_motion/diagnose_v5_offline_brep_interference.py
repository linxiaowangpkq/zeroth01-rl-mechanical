"""Exact OCCT B-Rep root-cause diagnosis for the complete v5 assembly.

This is the deterministic fallback when the running SOLIDWORKS process has
not registered its COM server.  It does not replace the final SOLIDWORKS
interference gate.  It uses the exact same STEP files and manifest transforms
to tell us which physical parts must be changed before rerunning Motion.
"""

from __future__ import annotations

import json
import hashlib
import time
from pathlib import Path

from build123d import Location, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf
from OCP.TopAbs import TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "offline_brep_component_interference.json"
VOLUME_TOLERANCE_MM3 = 0.01
BBOX_TOLERANCE_MM = 1.0e-5
DISTANCE_PREFILTER_TOLERANCE_MM = 1.0e-6


def transformed(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(matrix[0][3]),
        float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(matrix[1][3]),
        float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(matrix[2][3]),
    )
    return shape.moved(Location(gp_trsf=transform))


def boxes_overlap(a, b) -> bool:
    a_min = (float(a.min.X), float(a.min.Y), float(a.min.Z))
    a_max = (float(a.max.X), float(a.max.Y), float(a.max.Z))
    b_min = (float(b.min.X), float(b.min.Y), float(b.min.Z))
    b_max = (float(b.max.X), float(b.max.Y), float(b.max.Z))
    return all(
        a_min[index] < b_max[index] - BBOX_TOLERANCE_MM
        and b_min[index] < a_max[index] - BBOX_TOLERANCE_MM
        for index in range(3)
    )


def common_volume(first, second) -> float:
    common = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    common.SetNonDestructive(True)
    common.SetRunParallel(True)
    common.SetUseOBB(True)
    common.SetFuzzyValue(1.0e-7)
    common.Build()
    if not common.IsDone():
        raise RuntimeError("OCCT Boolean common failed")
    # Imported STEP compounds can contain solids with opposite shell
    # orientations.  Aggregate signed mass can therefore cancel and hide a
    # real overlap.  Interference volume is the sum of each solid magnitude.
    total = 0.0
    solid_count = 0
    explorer = TopExp_Explorer(common.Shape(), TopAbs_SOLID)
    while explorer.More():
        properties = GProp_GProps()
        BRepGProp.VolumeProperties_s(TopoDS.Solid_s(explorer.Current()), properties)
        total += abs(float(properties.Mass()))
        solid_count += 1
        explorer.Next()
    if solid_count:
        return total
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(common.Shape(), properties)
    return abs(float(properties.Mass()))


def exact_distance(first, second) -> float:
    distance = BRepExtrema_DistShapeShape(first.wrapped, second.wrapped)
    distance.Perform()
    if not distance.IsDone():
        raise RuntimeError("OCCT exact distance failed")
    return float(distance.Value())


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    rows = data["components"]
    installed = {}
    for index, row in enumerate(rows, start=1):
        component_id = str(row["component_id"])
        print(f"load {index}/{len(rows)} {component_id}", flush=True)
        source = ROOT / str(row["source"])
        installed[component_id] = transformed(
            import_step(source), row["transform_local_mm_to_world_mm"]
        )

    metadata = {str(row["component_id"]): row for row in rows}
    print(f"compute {len(installed)} cached assembly bounding boxes", flush=True)
    bounding_boxes = {component_id: shape.bounding_box() for component_id, shape in installed.items()}
    interferences = []
    component_ids = sorted(installed)
    all_pairs = [
        (first_id, second_id)
        for first_index, first_id in enumerate(component_ids)
        for second_id in component_ids[first_index + 1:]
    ]
    candidates = [
        pair for pair in all_pairs
        if boxes_overlap(bounding_boxes[pair[0]], bounding_boxes[pair[1]])
    ]
    print(f"aabb candidates={len(candidates)} all_pairs={len(all_pairs)}", flush=True)
    # With OBB, parallel and non-destructive options enabled, direct Boolean
    # common is much faster than BRepExtrema distance on the detailed STS3250
    # and faceted source carriers (0.1-0.8 s instead of 30-40 s in the local
    # benchmark).  Keep exact_distance() for targeted diagnosis only.
    distance_rejected = 0
    boolean_pair_count = 0
    for index, (first_id, second_id) in enumerate(candidates, start=1):
        started = time.perf_counter()
        print(
            f"candidate {index}/{len(candidates)} {first_id} :: {second_id}",
            flush=True,
        )
        first = installed[first_id]
        second = installed[second_id]
        try:
            boolean_pair_count += 1
            volume = common_volume(first, second)
        except Exception as exc:
            interferences.append({
                "component_ids": [first_id, second_id],
                "boolean_error": repr(exc),
            })
            continue
        print(
            f"  boolean volume={volume:.9g} mm^3 in {time.perf_counter() - started:.3f}s",
            flush=True,
        )
        if volume <= VOLUME_TOLERANCE_MM3:
            continue
        first_meta = metadata[first_id]
        second_meta = metadata[second_id]
        interferences.append({
            "component_ids": [first_id, second_id],
            "roles": [first_meta["role"], second_meta["role"]],
            "owner_links": [first_meta["owner_link"], second_meta["owner_link"]],
            "same_rigid_link": first_meta["owner_link"] == second_meta["owner_link"],
            "volume_mm3": volume,
        })

    interferences.sort(key=lambda row: float(row.get("volume_mm3", 0.0)), reverse=True)
    errors = [row for row in interferences if "boolean_error" in row]
    positives = [row for row in interferences if "volume_mm3" in row]
    boxes = list(bounding_boxes.values())
    overall_min = [min(float(getattr(box.min, axis)) for box in boxes) for axis in ("X", "Y", "Z")]
    overall_max = [max(float(getattr(box.max, axis)) for box in boxes) for axis in ("X", "Y", "Z")]
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.offline_brep_component_interference.v2",
        "manifest": str(MANIFEST),
        "manifest_sha256": hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
        "component_count": len(rows),
        "all_component_pair_count": len(all_pairs),
        "candidate_pair_count": len(candidates),
        "distance_prefilter_tolerance_mm": DISTANCE_PREFILTER_TOLERANCE_MM,
        "distance_rejected_pair_count": distance_rejected,
        "boolean_pair_count": boolean_pair_count,
        "positive_interference_count": len(positives),
        "total_positive_volume_mm3": sum(float(row["volume_mm3"]) for row in positives),
        "boolean_error_count": len(errors),
        "volume_tolerance_mm3": VOLUME_TOLERANCE_MM3,
        "assembly_bbox_world_mm": {"min": overall_min, "max": overall_max, "size": [overall_max[index] - overall_min[index] for index in range(3)]},
        "standing_height_mm": overall_max[2] - overall_min[2],
        "standing_height_limit_mm": 500.0,
        "standing_height_gate": "PASS" if overall_max[2] - overall_min[2] <= 500.0 else "FAIL",
        "rows": interferences,
        "truth_boundary": "All unordered pairs of the exact STEP B-Reps at manifest neutral transforms. Final acceptance still requires SOLIDWORKS static interference and isolated Motion gates.",
        "overall": "PASS" if not positives and not errors and overall_max[2] - overall_min[2] <= 500.0 else "FAIL",
    }
    REPORT.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: payload[key] for key in (
        "component_count", "all_component_pair_count", "candidate_pair_count", "positive_interference_count",
        "total_positive_volume_mm3", "boolean_error_count", "overall"
    )}, ensure_ascii=False, indent=2), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
