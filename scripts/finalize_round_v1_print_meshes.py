from __future__ import annotations

import csv
import json
from pathlib import Path

import numpy as np
import trimesh
from build123d import import_step


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
RAW_DIR = ROOT / "generated" / "print" / "round_v1"
FINAL_DIR = RAW_DIR / "final"
STEP_DIR = ROOT / "generated" / "cad" / "round_v1" / "parts"
REPORT_CSV = ROOT / "reports" / "round_v1_print_mesh_gate.csv"
REPORT_JSON = ROOT / "reports" / "round_v1_print_mesh_gate.json"

PREFIX = "ZEROTH01_ROUND_V1_"
MIN_COMPONENT_VOLUME_MM3 = 1e-3
MAX_STEP_VOLUME_ERROR_RATIO = 0.005


def edge_counts(mesh: trimesh.Trimesh) -> tuple[int, int]:
    counts = np.bincount(
        mesh.edges_unique_inverse,
        minlength=len(mesh.edges_unique),
    )
    return int(np.sum(counts == 1)), int(np.sum(counts > 2))


def main() -> None:
    sources = sorted(
        path
        for path in RAW_DIR.glob(f"{PREFIX}*.stl")
        if path.parent == RAW_DIR
    )
    if not sources:
        raise FileNotFoundError(f"no round-v1 STL files under {RAW_DIR}")
    FINAL_DIR.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []

    for source in sources:
        step = STEP_DIR / f"{source.stem}.step"
        if not step.is_file():
            raise FileNotFoundError(step)
        raw = trimesh.load_mesh(source, process=True)
        if not isinstance(raw, trimesh.Trimesh):
            raise TypeError(f"expected one mesh: {source}")
        components = list(raw.split(only_watertight=False))
        meaningful = [
            component
            for component in components
            if abs(float(component.volume)) >= MIN_COMPONENT_VOLUME_MM3
            and len(component.faces) >= 4
        ]
        if len(meaningful) != 1:
            raise ValueError(
                f"{source.name}: expected one meaningful component, "
                f"found {len(meaningful)}"
            )
        final = meaningful[0].copy()
        final.remove_unreferenced_vertices()
        if float(final.volume) < 0.0:
            final.invert()
        output = FINAL_DIR / source.name
        final.export(output)

        # Reload the actual deliverable rather than trusting the in-memory mesh.
        check = trimesh.load_mesh(output, process=True)
        if not isinstance(check, trimesh.Trimesh):
            raise TypeError(f"expected one final mesh: {output}")
        boundary, nonmanifold = edge_counts(check)
        step_shape = import_step(step)
        step_volume = float(step_shape.volume)
        mesh_volume = float(check.volume)
        volume_error = abs(mesh_volume - step_volume) / step_volume
        components_after = list(check.split(only_watertight=False))
        gate = bool(
            check.is_watertight
            and check.is_winding_consistent
            and boundary == 0
            and nonmanifold == 0
            and len(components_after) == 1
            and volume_error <= MAX_STEP_VOLUME_ERROR_RATIO
        )
        bounds = check.bounds
        extents = check.extents
        rows.append(
            {
                "part": source.stem,
                "raw_stl": source.relative_to(ROOT).as_posix(),
                "final_stl": output.relative_to(ROOT).as_posix(),
                "units": "mm",
                "raw_component_count": len(components),
                "removed_zero_volume_artifacts": (
                    len(components) - len(meaningful)
                ),
                "final_component_count": len(components_after),
                "final_watertight": bool(check.is_watertight),
                "final_winding_consistent": bool(
                    check.is_winding_consistent
                ),
                "boundary_edge_count": boundary,
                "nonmanifold_edge_count": nonmanifold,
                "step_volume_mm3": f"{step_volume:.9f}",
                "mesh_volume_mm3": f"{mesh_volume:.9f}",
                "step_mesh_volume_error_ratio": f"{volume_error:.9g}",
                "bbox_min_x_mm": f"{bounds[0, 0]:.9f}",
                "bbox_min_y_mm": f"{bounds[0, 1]:.9f}",
                "bbox_min_z_mm": f"{bounds[0, 2]:.9f}",
                "bbox_size_x_mm": f"{extents[0]:.9f}",
                "bbox_size_y_mm": f"{extents[1]:.9f}",
                "bbox_size_z_mm": f"{extents[2]:.9f}",
                "slicer_profile_gate": "BLOCKED_EXPLICIT_PROFILE_REQUIRED",
                "functional_load_path_gate": (
                    "FAIL_COSMETIC_OR_FIT_PROTOTYPE_ONLY"
                ),
                "mesh_topology_gate": "PASS" if gate else "FAIL",
            }
        )

    overall = all(row["mesh_topology_gate"] == "PASS" for row in rows)
    REPORT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    payload = {
        "schema": "zeroth01.round_v1.print_mesh_gate.v1",
        "part_count": len(rows),
        "mesh_topology_gate": "PASS" if overall else "FAIL",
        "slicer_profile_gate": "BLOCKED_EXPLICIT_PROFILE_REQUIRED",
        "functional_load_path_gate": (
            "FAIL: these outputs are cosmetic shells, badges, sole fit "
            "prototypes and a ring coupon; they do not replace missing "
            "production servo brackets, horn interfaces, bearings, fasteners "
            "or cable management"
        ),
        "rows": rows,
    }
    REPORT_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if not overall:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
