"""Build a lightweight, coloured neutral-pose review GLB from the v5 manifest.

The GLB is a review derivative.  Manufacturing and interference truth remains
the occurrence-level STEP manifest plus the exact B-Rep reports.
"""

from __future__ import annotations

import json
from pathlib import Path

from build123d import Color, Compound, Location, export_gltf, import_step
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
CAD_ROOT = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion"
MANIFEST = CAD_ROOT / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
OUTPUT = CAD_ROOT / "ZEROTH01_V5_ORIGINAL_16DOF_NEUTRAL_REVIEW.glb"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "review_glb_gate.json"


def moved(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(matrix[0][3]),
        float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(matrix[1][3]),
        float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(matrix[2][3]),
    )
    return shape.moved(Location(gp_trsf=transform))


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    cached = {}
    children = []
    for index, row in enumerate(manifest["components"], start=1):
        source = ROOT / str(row["source"])
        if source not in cached:
            cached[source] = import_step(source)
        placed = moved(cached[source], row["transform_local_mm_to_world_mm"])
        placed.label = str(row["component_id"])
        try:
            placed.color = Color(str(row["color_hex"]))
        except Exception:
            pass
        children.append(placed)
        if index % 10 == 0 or index == len(manifest["components"]):
            print(f"review GLB occurrence {index}/{len(manifest['components'])}", flush=True)
    assembly = Compound(children=children)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    ok = export_gltf(
        assembly,
        OUTPUT,
        binary=True,
        linear_deflection=0.08,
        angular_deflection=0.15,
    )
    bbox = assembly.bounding_box()
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.review_glb_gate.v1",
        "source_manifest": MANIFEST.relative_to(ROOT).as_posix(),
        "component_count": len(children),
        "unique_step_source_count": len(cached),
        "solid_count": len(assembly.solids()),
        "bbox_mm": {
            "min": [bbox.min.X, bbox.min.Y, bbox.min.Z],
            "max": [bbox.max.X, bbox.max.Y, bbox.max.Z],
            "size": [bbox.size.X, bbox.size.Y, bbox.size.Z],
        },
        "output": OUTPUT.relative_to(ROOT).as_posix(),
        "bytes": OUTPUT.stat().st_size if OUTPUT.is_file() else 0,
        "truth_boundary": "lightweight coloured neutral-pose review derivative; not the manufacturing or interference authority",
        "overall": "PASS" if ok and OUTPUT.is_file() and OUTPUT.stat().st_size > 1024 and len(children) == 85 else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
