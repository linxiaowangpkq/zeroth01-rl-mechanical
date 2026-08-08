"""Add the two physical M3 wrist-support holes to both forearm carriers."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from build123d import Align, Compound, Cylinder, Location, export_step, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
PARTS = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "parts"
BASE = PARTS / "source_carriers_sts3250_clearanced"
OUT = PARTS / "source_carriers_sts3250_wrist_mount"
MANIFEST = PARTS.parent / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "wrist_support_mount_holes.json"


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[row][column]) for row in range(3) for column in range(4)])
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def volume(shape) -> float:
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, properties)
    return abs(float(properties.Mass()))


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    metadata = {str(row["component_id"]): row for row in manifest["components"]}
    rows = []
    OUT.mkdir(parents=True, exist_ok=True)
    pairs = (
        ("R_ARM_MIRROR_1", "LEFT_FIXED_WRIST_SUPPORT", "FINGER_1"),
        ("L_ARM_MIRROR_1", "RIGHT_FIXED_WRIST_SUPPORT", "FINGER_1_2"),
    )
    # Same local datum as generate_v5_cad.wrist_support().
    cx, cz = -3.4808, 18.7993
    holes = Compound(children=[
        Cylinder(1.7, 42.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((x_pos, 0.0, cz), (90.0, 0.0, 0.0))
        )
        for x_pos in (cx - 5.0, cx + 5.0)
    ])
    for owner, support_id, _finger in pairs:
        carrier_path = BASE / f"{owner}_STS3250_CLEARANCED.step"
        carrier = import_step(carrier_path)
        carrier_row = metadata[f"CARRIER_{owner}"]
        support_row = metadata[support_id]
        carrier_world = np.asarray(carrier_row["transform_local_mm_to_world_mm"], dtype=float)
        support_world = np.asarray(support_row["transform_local_mm_to_world_mm"], dtype=float)
        holes_in_carrier = matrix_shape(holes, np.linalg.inv(carrier_world) @ support_world)
        initial_volume = volume(carrier)
        operation = BRepAlgoAPI_Cut(carrier.wrapped, holes_in_carrier.wrapped)
        operation.Build()
        if not operation.IsDone():
            raise RuntimeError(f"wrist M3 hole cut failed: {owner}")
        carrier = Compound(operation.Shape())
        target = OUT / f"{owner}_STS3250_WRIST_MOUNT.step"
        export_step(carrier, target)
        final_volume = volume(carrier)
        valid = bool(BRepCheck_Analyzer(carrier.wrapped).IsValid())
        removed = initial_volume - final_volume
        status = "PASS" if valid and removed > 1.0 and final_volume > 1000.0 else "FAIL"
        rows.append({
            "owner_link": owner,
            "source": carrier_path.relative_to(ROOT).as_posix(),
            "target": target.relative_to(ROOT).as_posix(),
            "hole_count": 2,
            "hole_diameter_mm": 3.4,
            "fastener": "2x M3 through-bolt, washer and locknut",
            "removed_volume_mm3": removed,
            "occt_valid": valid,
            "status": status,
        })
    payload = {
        "schema": "zeroth01.v5.wrist_support_mount_holes.v1",
        "rows": rows,
        "joint_axes_changed": False,
        "link_lengths_changed": False,
        "overall": "PASS" if all(row["status"] == "PASS" for row in rows) else "FAIL",
    }
    REPORT.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
