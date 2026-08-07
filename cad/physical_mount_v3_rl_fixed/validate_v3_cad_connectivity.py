"""Verify that every actuator is physically bridged by parent/child CAD.

This gate is intentionally separate from URDF kinematics: a mathematically
valid joint may still look connected while its servo floats in CAD.  For each
of the 18 joints we measure exact B-Rep distance from the controlled STS3250
shape to at least one parent-side and one child-side load-bearing component.
"""

from __future__ import annotations

import importlib.util
import json
import re
from pathlib import Path

import FreeCAD as App
import Part


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed" / "ZEROTH01_V3_RL_FIXED_18DOF_FULL_ASSEMBLY_MANIFEST.json"
REPORT = ROOT / "reports" / "physical_mount_v3_rl_fixed" / "cad_connectivity_gate.json"
MAX_INTERFACE_GAP_MM = 1.0
MAX_UNINTENDED_OVERLAP_MM3 = 1.0
STRUCTURAL_ROLES = {
    "source_load_bearing_carrier",
    "new_ankle_roll_parent_carrier",
    "new_ankle_roll_child_horn_adapter",
    "reversible_slotted_inboard_adapter",
    "replaceable_7mm_perimeter_rib_sole",
}


def load_urdf_source():
    path = Path(__file__).with_name("build_v3_urdf.py")
    spec = importlib.util.spec_from_file_location("zeroth_v3_connectivity_urdf", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def placement(values) -> App.Placement:
    matrix = App.Matrix()
    for row in range(4):
        for column in range(4):
            setattr(matrix, f"A{row + 1}{column + 1}", float(values[row][column]))
    return App.Placement(matrix)


def load_shape(path: Path):
    shape = Part.Shape()
    shape.read(str(path))
    if shape.isNull() or not shape.isValid():
        raise RuntimeError(f"invalid shape: {path}")
    return shape


def main() -> int:
    u = load_urdf_source()
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    cache = {}
    components = []
    for row in payload["components"]:
        source = ROOT / str(row["source"])
        if source not in cache:
            cache[source] = load_shape(source)
        shape = cache[source].copy()
        shape.Placement = placement(row["transform_local_mm_to_world_mm"])
        components.append((row, shape))

    by_owner = {}
    servos = {}
    for row, shape in components:
        role = str(row["role"])
        owner = str(row["owner_link"])
        if role in STRUCTURAL_ROLES:
            by_owner.setdefault(owner, []).append((row, shape))
        if role == "dimension_controlled_sts3250":
            match = re.match(r"S\d+_STS3250_(.+)", str(row["component_id"]))
            if match:
                servos[match.group(1)] = (row, shape)

    rows = []
    for joint_name, parent, child, _, _ in u.JOINT_SPECS:
        servo_row, servo_shape = servos[joint_name]
        parent_parts = by_owner.get(parent, [])
        child_parts = by_owner.get(child, [])
        parent_distances = [
            (
                float(servo_shape.distToShape(shape)[0]),
                str(row["component_id"]),
                float(servo_shape.common(shape).Volume),
            )
            for row, shape in parent_parts
        ]
        child_distances = [
            (
                float(servo_shape.distToShape(shape)[0]),
                str(row["component_id"]),
                float(servo_shape.common(shape).Volume),
            )
            for row, shape in child_parts
        ]
        parent_best = min(parent_distances, default=(float("inf"), "MISSING", float("inf")))
        child_best = min(child_distances, default=(float("inf"), "MISSING", float("inf")))
        gate = "PASS" if (
            max(parent_best[0], child_best[0]) <= MAX_INTERFACE_GAP_MM
            and max(parent_best[2], child_best[2]) <= MAX_UNINTENDED_OVERLAP_MM3
        ) else "FAIL"
        rows.append({
            "joint": joint_name,
            "servo_component": servo_row["component_id"],
            "parent_link": parent,
            "parent_component": parent_best[1],
            "parent_interface_gap_mm": parent_best[0],
            "parent_unintended_overlap_mm3": parent_best[2],
            "child_link": child,
            "child_component": child_best[1],
            "child_interface_gap_mm": child_best[0],
            "child_unintended_overlap_mm3": child_best[2],
            "gate": gate,
        })
        print(
            f"{joint_name}: gap(parent={parent_best[0]:.4f}, child={child_best[0]:.4f}) "
            f"overlap(parent={parent_best[2]:.3f}, child={child_best[2]:.3f}) {gate}",
            flush=True,
        )

    report = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.cad_connectivity_gate.v1",
        "method": "exact FreeCAD B-Rep minimum distance from each STS3250 to parent-side and child-side structural components",
        "maximum_allowed_interface_gap_mm": MAX_INTERFACE_GAP_MM,
        "maximum_unintended_overlap_mm3": MAX_UNINTENDED_OVERLAP_MM3,
        "joint_count": len(rows),
        "pass_count": sum(row["gate"] == "PASS" for row in rows),
        "rows": rows,
        "overall": "PASS" if rows and all(row["gate"] == "PASS" for row in rows) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("joint_count", "pass_count", "overall")}, indent=2), flush=True)
    return 0 if report["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
