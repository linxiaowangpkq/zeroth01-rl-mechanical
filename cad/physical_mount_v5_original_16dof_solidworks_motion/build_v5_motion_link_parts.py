"""Build one positive-volume multibody STEP per URDF rigid link.

The complete occurrence-level review assembly remains the installation/BOM
deliverable; this builder derives its count from the current manifest.
For SOLIDWORKS Motion, every URDF link must behave as one rigid body.  This
builder therefore collects the released carrier, its fixed STS3250 housing or
output hardware, and its fixed accessories into 17 coloured multibody STEP
parts.  Adjacent link parts are connected only by the 16 real revolute pairs.
"""

from __future__ import annotations

import json
from pathlib import Path

from build123d import Color, Compound, Location, export_step, import_step
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v5_original_16dof_solidworks_motion"
    / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
)
OUT = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v5_original_16dof_solidworks_motion"
    / "motion_link_parts"
)
REPORT = (
    ROOT
    / "reports"
    / "v5_original_16dof_solidworks_motion"
    / "motion_link_part_gate.json"
)


def safe(value: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in value)


def output_path(owner: str) -> Path:
    return OUT / f"ZEROTH01_V5_MOTION_LINK_{safe(owner).upper()}.step"


def transformed(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(matrix[0][3]),
        float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(matrix[1][3]),
        float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(matrix[2][3]),
    )
    return shape.moved(Location(gp_trsf=transform))


def main() -> int:
    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    carrier_order = [
        str(row["owner_link"])
        for row in data["components"]
        if row["role"] == "source_load_bearing_carrier"
    ]
    rows = []
    OUT.mkdir(parents=True, exist_ok=True)
    for index, owner in enumerate(carrier_order, start=1):
        owned = [row for row in data["components"] if str(row["owner_link"]) == owner]
        children = []
        source_solid_count = 0
        source_surface_count = 0
        purchased_exact_servo_count = 0
        for row in owned:
            source = ROOT / str(row["source"])
            shape = import_step(source)
            if row["role"] == "purchased_exact_sts3250":
                purchased_exact_servo_count += 1
            source_solid_count += len(shape.solids())
            source_surface_count += len(shape.shells()) - len(shape.solids())
            try:
                shape.color = Color(str(row["color_hex"]))
            except Exception:
                pass
            children.append(transformed(shape, row["transform_local_mm_to_world_mm"]))
        compound = Compound(children=children)
        target = output_path(owner)
        print(f"motion link {index}/{len(carrier_order)} {owner}: components={len(owned)}", flush=True)
        export_step(compound, target)
        verified = import_step(target)
        solids = len(verified.solids())
        valid = BRepCheck_Analyzer(verified.wrapped).IsValid()
        moving = owner != "Z_BOT2_MASTER_BODY_SKELETON"
        status = "PASS" if target.stat().st_size > 1024 and valid and (not moving or solids >= 1) else "FAIL"
        rows.append(
            {
                "owner_link": owner,
                "component_count": len(owned),
                "blue_sts3250_count": sum(row["role"] == "purchased_exact_sts3250" for row in owned),
                "purchased_exact_sts3250_count": purchased_exact_servo_count,
                "output_bridge_count": sum(row["role"] == "sts3250_pcd14_output_bridge_to_child" for row in owned),
                "source_solid_count": source_solid_count,
                "source_non_solid_shell_estimate": source_surface_count,
                "reimported_solid_count": solids,
                "reimported_face_count": len(verified.faces()),
                "occt_brep_valid": bool(valid),
                "step": target.relative_to(ROOT).as_posix(),
                "bytes": target.stat().st_size,
                "status": status,
            }
        )
        if status != "PASS":
            raise RuntimeError(rows[-1])
    payload = {
        "schema": "zeroth01.v5.motion_link_parts.v1",
        "method": "one coloured multibody STEP per URDF rigid link; exact carriers, exact purchased STS3250 STEP and exact output interfaces",
        "link_part_count": len(rows),
        "component_count": sum(row["component_count"] for row in rows),
        "blue_sts3250_count": sum(row["blue_sts3250_count"] for row in rows),
        "purchased_exact_sts3250_count": sum(row["purchased_exact_sts3250_count"] for row in rows),
        "output_bridge_count": sum(row["output_bridge_count"] for row in rows),
        "moving_links_all_positive_volume": all(
            row["reimported_solid_count"] >= 1
            for row in rows
            if row["owner_link"] != "Z_BOT2_MASTER_BODY_SKELETON"
        ),
        "links": rows,
        "overall": "PASS" if len(rows) == 17 and sum(row["component_count"] for row in rows) == len(data["components"]) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
