"""Cut only measured STS3250/ring clashes from released solid carriers.

The released carrier silhouette, joint axes, link lengths and PCD14 output
horns remain unchanged.  Each cut is the exact purchased B-Rep (or thrust
ring) swept by +/-0.30 mm along its local XYZ directions, giving a printable
FDM installation clearance without using a large generic box pocket.
"""

from __future__ import annotations

import json
from collections import defaultdict
from pathlib import Path

import numpy as np
from build123d import Align, Box, Compound, Cylinder, Location, export_step, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
PARTS = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "parts"
BASE = PARTS / "source_carriers_solid"
OUT = PARTS / "source_carriers_sts3250_clearanced"
MANIFEST = PARTS.parent / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
DIAGNOSIS = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "offline_brep_component_interference.json"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "source_carrier_sts3250_clearance.json"
CLEARANCE_MM = 0.30


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[row][column]) for row in range(3) for column in range(4)])
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def volume(shape) -> float:
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props)
    return abs(float(props.Mass()))


def common_volume(first, second) -> float:
    operation = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT Boolean common failed")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(operation.Shape(), props)
    return abs(float(props.Mass()))


def cut_shape(base, tool):
    operation = BRepAlgoAPI_Cut(base.wrapped, tool.wrapped)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT carrier clearance cut failed")
    return Compound(operation.Shape())


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    diagnosis = json.loads(DIAGNOSIS.read_text(encoding="utf-8"))
    metadata = {str(row["component_id"]): row for row in manifest["components"]}

    targets = defaultdict(list)
    accepted_roles = {"purchased_exact_sts3250", "sts3250_parent_axis_and_thrust_mate"}
    for row in diagnosis["rows"]:
        ids = list(row.get("component_ids", []))
        roles = list(row.get("roles", []))
        if len(ids) != 2 or "source_load_bearing_carrier" not in roles:
            continue
        carrier_index = roles.index("source_load_bearing_carrier")
        other_index = 1 - carrier_index
        if roles[other_index] not in accepted_roles:
            continue
        carrier_id = str(ids[carrier_index])
        other_id = str(ids[other_index])
        if carrier_id not in metadata or other_id not in metadata:
            continue
        owner = str(metadata[carrier_id]["owner_link"])
        if owner == "Z_BOT2_MASTER_BODY_SKELETON":
            # The fixed torso is a released zero-volume faceted reference and
            # had no reported positive-volume STS/ring clash.
            continue
        targets[owner].append(other_id)

    # Installation pockets are machined/printed from a smooth control
    # envelope, not from every vendor thread face.  Dimensions and shaft
    # eccentricity come directly from the exact STEP bbox.
    servo_clearance = Box(
        45.22 + 2.0 * CLEARANCE_MM,
        24.72 + 2.0 * CLEARANCE_MM,
        37.40 + 2.0 * CLEARANCE_MM,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((-10.11, 0.0, -20.75)))
    ring_clearance = Cylinder(
        12.5 + CLEARANCE_MM,
        2.0 + 2.0 * CLEARANCE_MM,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).moved(Location((0.0, 0.0, CLEARANCE_MM)))
    exact_sources = {}
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    carrier_rows = [row for row in manifest["components"] if row["role"] == "source_load_bearing_carrier" and row["owner_link"] != "Z_BOT2_MASTER_BODY_SKELETON"]
    for index, carrier_row in enumerate(carrier_rows, start=1):
        owner = str(carrier_row["owner_link"])
        print(f"carrier {index}/{len(carrier_rows)} {owner}", flush=True)
        source = BASE / f"{owner}_SOLID.step"
        if not source.is_file():
            raise FileNotFoundError(source)
        carrier = import_step(source)
        initial_volume = volume(carrier)
        owner_world = np.asarray(carrier_row["transform_local_mm_to_world_mm"], dtype=float)
        inverse_owner = np.linalg.inv(owner_world)
        operations = []
        for tool_id in sorted(set(targets.get(owner, []))):
            tool_row = metadata[tool_id]
            tool_local_source = (
                servo_clearance
                if tool_row["role"] == "purchased_exact_sts3250"
                else ring_clearance
            )
            tool_world = np.asarray(tool_row["transform_local_mm_to_world_mm"], dtype=float)
            relative = inverse_owner @ tool_world
            clearance_tool = matrix_shape(tool_local_source, relative)
            before_overlap = common_volume(carrier, clearance_tool)
            carrier = cut_shape(carrier, clearance_tool)
            exact_path = ROOT / str(tool_row["source"])
            if str(exact_path) not in exact_sources:
                exact_sources[str(exact_path)] = import_step(exact_path)
            exact_source = exact_sources[str(exact_path)]
            nominal_tool = matrix_shape(exact_source, relative)
            after_overlap = common_volume(carrier, nominal_tool)
            operations.append({
                "tool_component_id": tool_id,
                "tool_role": tool_row["role"],
                "removed_clearance_sweep_volume_mm3": before_overlap,
                "remaining_nominal_overlap_mm3": after_overlap,
            })
        target = OUT / f"{owner}_STS3250_CLEARANCED.step"
        export_step(carrier, target)
        final_volume = volume(carrier)
        valid = bool(BRepCheck_Analyzer(carrier.wrapped).IsValid())
        rows.append({
            "owner_link": owner,
            "source": source.relative_to(ROOT).as_posix(),
            "target": target.relative_to(ROOT).as_posix(),
            "clearance_mm": CLEARANCE_MM,
            "operation_count": len(operations),
            "initial_volume_mm3": initial_volume,
            "final_volume_mm3": final_volume,
            "removed_volume_mm3": initial_volume - final_volume,
            "remaining_nominal_overlap_mm3": sum(float(item["remaining_nominal_overlap_mm3"]) for item in operations),
            "occt_valid": valid,
            "operations": operations,
            "status": "PASS" if valid and final_volume > 1.0 and all(float(item["remaining_nominal_overlap_mm3"]) <= 0.01 for item in operations) else "FAIL",
        })

    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.source_carrier_sts3250_clearance.v1",
        "clearance_mm": CLEARANCE_MM,
        "carrier_count": len(rows),
        "modified_carrier_count": sum(row["operation_count"] > 0 for row in rows),
        "rows": rows,
        "joint_axes_changed": False,
        "link_lengths_changed": False,
        "released_outer_silhouette_policy": "only exact internal actuator/ring clash volumes swept by 0.30 mm are removed",
    }
    payload["overall"] = "PASS" if rows and all(row["status"] == "PASS" for row in rows) else "FAIL"
    REPORT.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "carrier_count": payload["carrier_count"],
        "modified_carrier_count": payload["modified_carrier_count"],
        "overall": payload["overall"],
    }, ensure_ascii=False, indent=2), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
