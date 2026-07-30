from __future__ import annotations

import csv
import json
from pathlib import Path

import trimesh


ROOT = Path(__file__).resolve().parents[1]
SOURCE_MESH_DIR = ROOT / "generated" / "urdf" / "meshes"
OVERLAY_MESH_DIR = ROOT / "generated" / "print" / "round_v1" / "final"
REPORT_JSON = ROOT / "reports" / "round_v3_arm_fit_gate.json"
REPORT_CSV = ROOT / "reports" / "round_v3_arm_fit_gate.csv"

# Manifold booleans can retain microscopic slivers at nearly coincident
# facets.  Anything above 0.1 mm^3 is treated as a real
# source-link/cosmetic-shell intrusion and fails the fit gate.
MAX_INTERSECTION_VOLUME_MM3 = 0.1

CASES = [
    {
        "fit": "right_upper_arm_sleeve",
        "source_link": "right_shoulder_yaw_motor",
        "source_mesh": "right_shoulder_yaw_motor.stl",
        "overlay_mesh": "ZEROTH01_ROUND_V3_RIGHT_UPPER_ARM_SLEEVE.stl",
    },
    {
        "fit": "left_upper_arm_sleeve",
        "source_link": "left_shoulder_yaw_motor",
        "source_mesh": "left_shoulder_yaw_motor.stl",
        "overlay_mesh": "ZEROTH01_ROUND_V3_LEFT_UPPER_ARM_SLEEVE.stl",
    },
    {
        "fit": "right_forearm_sleeve",
        "source_link": "Left_Hand",
        "source_mesh": "Left_Hand.stl",
        "overlay_mesh": "ZEROTH01_ROUND_V3_RIGHT_FOREARM_SLEEVE.stl",
    },
    {
        "fit": "left_forearm_sleeve",
        "source_link": "hand_right",
        "source_mesh": "hand_right.stl",
        "overlay_mesh": "ZEROTH01_ROUND_V3_LEFT_FOREARM_SLEEVE.stl",
    },
    {
        "fit": "right_chibi_hand",
        "source_link": "Left_Hand",
        "source_mesh": "Left_Hand.stl",
        "overlay_mesh": "ZEROTH01_ROUND_V3_RIGHT_CHIBI_HAND.stl",
    },
    {
        "fit": "left_chibi_hand",
        "source_link": "hand_right",
        "source_mesh": "hand_right.stl",
        "overlay_mesh": "ZEROTH01_ROUND_V3_LEFT_CHIBI_HAND.stl",
    },
]


def bbox_payload(mesh: trimesh.Trimesh) -> dict[str, str]:
    bbox = mesh.bounds
    size = mesh.extents
    return {
        "min_x_mm": f"{bbox[0, 0]:.9f}",
        "min_y_mm": f"{bbox[0, 1]:.9f}",
        "min_z_mm": f"{bbox[0, 2]:.9f}",
        "size_x_mm": f"{size[0]:.9f}",
        "size_y_mm": f"{size[1]:.9f}",
        "size_z_mm": f"{size[2]:.9f}",
    }


def main() -> None:
    rows: list[dict[str, object]] = []
    for case in CASES:
        source_path = SOURCE_MESH_DIR / str(case["source_mesh"])
        overlay_path = OVERLAY_MESH_DIR / str(case["overlay_mesh"])
        if not source_path.is_file():
            raise FileNotFoundError(source_path)
        if not overlay_path.is_file():
            raise FileNotFoundError(overlay_path)

        source = trimesh.load_mesh(source_path, process=True)
        overlay = trimesh.load_mesh(overlay_path, process=True)
        if not isinstance(source, trimesh.Trimesh):
            raise TypeError(f"expected one source mesh: {source_path}")
        if not isinstance(overlay, trimesh.Trimesh):
            raise TypeError(f"expected one overlay mesh: {overlay_path}")
        # URDF source meshes are stored in metres; printable CAD is authored
        # in millimetres.  Normalize both to millimetres before the fit check.
        source.apply_scale(1000.0)
        if not overlay.is_watertight:
            raise ValueError(f"overlay mesh is not watertight: {overlay_path}")
        # The frozen upstream visual meshes contain open decorative surfaces.
        # Their convex hull is a watertight conservative keep-out: if the
        # printed shell misses the hull, it necessarily misses the source mesh.
        source_keepout = source.convex_hull
        if not source_keepout.is_watertight:
            raise ValueError(
                f"source convex hull is not watertight: {source_path}"
            )
        intersection = trimesh.boolean.intersection(
            [source_keepout, overlay],
            engine="manifold",
            check_volume=True,
        )
        if intersection is None:
            intersection_fragment_count = 0
            intersection_volume = 0.0
        else:
            if not isinstance(intersection, trimesh.Trimesh):
                raise TypeError(
                    f"expected one boolean result mesh: {case['fit']}"
                )
            intersection_fragment_count = len(
                intersection.split(only_watertight=False)
            )
            intersection_volume = abs(float(intersection.volume))
        gate = intersection_volume <= MAX_INTERSECTION_VOLUME_MM3

        source_bbox = bbox_payload(source)
        overlay_bbox = bbox_payload(overlay)
        rows.append(
            {
                "fit": case["fit"],
                "source_link": case["source_link"],
                "source_mesh": source_path.relative_to(ROOT).as_posix(),
                "source_mesh_native_units": "m",
                "analysis_units": "mm",
                "overlay_mesh": overlay_path.relative_to(ROOT).as_posix(),
                "source_watertight": bool(source.is_watertight),
                "source_convex_hull_watertight": bool(
                    source_keepout.is_watertight
                ),
                "overlay_watertight": bool(overlay.is_watertight),
                "intersection_fragment_count": intersection_fragment_count,
                "intersection_volume_mm3": f"{intersection_volume:.9f}",
                "max_allowed_intersection_volume_mm3": (
                    f"{MAX_INTERSECTION_VOLUME_MM3:.9f}"
                ),
                "source_bbox_min_x_mm": source_bbox["min_x_mm"],
                "source_bbox_min_y_mm": source_bbox["min_y_mm"],
                "source_bbox_min_z_mm": source_bbox["min_z_mm"],
                "source_bbox_size_x_mm": source_bbox["size_x_mm"],
                "source_bbox_size_y_mm": source_bbox["size_y_mm"],
                "source_bbox_size_z_mm": source_bbox["size_z_mm"],
                "overlay_bbox_min_x_mm": overlay_bbox["min_x_mm"],
                "overlay_bbox_min_y_mm": overlay_bbox["min_y_mm"],
                "overlay_bbox_min_z_mm": overlay_bbox["min_z_mm"],
                "overlay_bbox_size_x_mm": overlay_bbox["size_x_mm"],
                "overlay_bbox_size_y_mm": overlay_bbox["size_y_mm"],
                "overlay_bbox_size_z_mm": overlay_bbox["size_z_mm"],
                "fit_gate": "PASS" if gate else "FAIL",
            }
        )

    overall = all(row["fit_gate"] == "PASS" for row in rows)
    payload = {
        "schema": "zeroth01.round_v3.arm_fit_gate.v1",
        "method": (
            "Manifold exact intersection volume between the conservative "
            "convex hull of each frozen source-link STL and its local-frame "
            "final printable STL"
        ),
        "case_count": len(rows),
        "fit_gate": "PASS" if overall else "FAIL",
        "rows": rows,
    }
    REPORT_JSON.parent.mkdir(parents=True, exist_ok=True)
    REPORT_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if not overall:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
