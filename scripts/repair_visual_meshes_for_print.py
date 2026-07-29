from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh


ROOT = Path(__file__).resolve().parents[1]
SOURCE_MANIFEST = ROOT / "reports" / "solidworks_part_import.csv"
OUTPUT_DIR = ROOT / "generated" / "print" / "visual_proxies_mm"
REPORT_CSV = ROOT / "reports" / "printability_mesh_audit.csv"
REPORT_JSON = ROOT / "reports" / "printability_mesh_audit.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def edge_counts(mesh: trimesh.Trimesh) -> tuple[int, int]:
    counts = np.bincount(
        mesh.edges_unique_inverse, minlength=len(mesh.edges_unique)
    )
    return int(np.count_nonzero(counts == 1)), int(np.count_nonzero(counts > 2))


def load_mesh(path: Path, *, process: bool, validate: bool = False) -> trimesh.Trimesh:
    loaded = trimesh.load(path, force="mesh", process=process, validate=validate)
    if not isinstance(loaded, trimesh.Trimesh):
        raise TypeError(f"expected Trimesh for {path}, got {type(loaded).__name__}")
    return loaded


def repair_mesh(source: Path) -> trimesh.Trimesh:
    processed = load_mesh(source, process=True, validate=True)
    repaired = trimesh.boolean.union(
        [processed],
        engine="manifold",
        check_volume=False,
    )
    if not isinstance(repaired, trimesh.Trimesh):
        raise TypeError(f"manifold repair did not return Trimesh for {source}")
    repaired.remove_unreferenced_vertices()
    if repaired.is_winding_consistent and repaired.volume < 0:
        repaired.invert()
    if not repaired.is_watertight:
        raise RuntimeError(f"repair is not watertight: {source}")
    if not repaired.is_winding_consistent:
        raise RuntimeError(f"repair has inconsistent winding: {source}")
    if repaired.volume <= 0:
        raise RuntimeError(f"repair has non-positive volume: {source}")
    return repaired


def generate() -> dict[str, object]:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    with SOURCE_MANIFEST.open("r", encoding="utf-8-sig", newline="") as stream:
        manifest = list(csv.DictReader(stream))

    for item in manifest:
        link = item["link"]
        source = Path(item["mesh"])
        raw = load_mesh(source, process=False)
        raw_boundary, raw_nonmanifold = edge_counts(raw)
        output = OUTPUT_DIR / f"{link}_VISUAL_PROXY_MM.stl"
        try:
            repaired_m = repair_mesh(source)
            repaired_mm = repaired_m.copy()
            repaired_mm.apply_scale(1000.0)
            repaired_mm.export(output, file_type="stl")
            check = load_mesh(output, process=True, validate=True)
            out_boundary, out_nonmanifold = edge_counts(check)
            extents = check.extents
            output_gate = (
                check.is_watertight
                and check.is_winding_consistent
                and out_boundary == 0
                and out_nonmanifold == 0
                and check.volume > 0
            )
            repair_error = ""
        except Exception as error:
            if output.exists():
                output.unlink()
            check = None
            out_boundary = -1
            out_nonmanifold = -1
            extents = np.array([np.nan, np.nan, np.nan])
            output_gate = False
            repair_error = f"{type(error).__name__}: {error}"
        rows.append(
            {
                "link": link,
                "source_mesh_m": str(source),
                "source_sha256": sha256(source),
                "source_faces": len(raw.faces),
                "source_vertices": len(raw.vertices),
                "source_watertight": raw.is_watertight,
                "source_winding_consistent": raw.is_winding_consistent,
                "source_boundary_edges": raw_boundary,
                "source_nonmanifold_edges": raw_nonmanifold,
                "output_mesh_mm": str(output),
                "output_sha256": sha256(output) if check is not None else "",
                "output_faces": len(check.faces) if check is not None else 0,
                "output_vertices": len(check.vertices) if check is not None else 0,
                "output_components": (
                    len(check.split(only_watertight=False))
                    if check is not None
                    else 0
                ),
                "output_watertight": (
                    check.is_watertight if check is not None else False
                ),
                "output_winding_consistent": (
                    check.is_winding_consistent if check is not None else False
                ),
                "output_boundary_edges": out_boundary,
                "output_nonmanifold_edges": out_nonmanifold,
                "bbox_x_mm": f"{extents[0]:.6f}",
                "bbox_y_mm": f"{extents[1]:.6f}",
                "bbox_z_mm": f"{extents[2]:.6f}",
                "volume_mm3": f"{check.volume:.6f}" if check is not None else "",
                "semantic": "DISPLAY_VISUAL_PROXY_NOT_FUNCTIONAL_SERVO_BRACKET",
                "repair_error": repair_error,
                "gate": "PASS" if output_gate else "FAIL",
            }
        )

    fieldnames = list(rows[0])
    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    summary = {
        "input_unit": "m",
        "output_unit": "mm",
        "count": len(rows),
        "all_output_watertight": all(bool(row["output_watertight"]) for row in rows),
        "all_output_winding_consistent": all(
            bool(row["output_winding_consistent"]) for row in rows
        ),
        "all_gates_pass": all(row["gate"] == "PASS" for row in rows),
        "semantic": (
            "These are solidified link-level visual proxies for display/fit review. "
            "They are not servo-separated load-bearing fabrication parts."
        ),
        "rows": rows,
    }
    REPORT_JSON.write_text(
        json.dumps(summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args()
    summary = generate()
    if args.json:
        print(json.dumps(summary, indent=2, ensure_ascii=False))
    else:
        print(
            f"repaired={summary['count']} "
            f"all_gates_pass={summary['all_gates_pass']} "
            f"report={REPORT_CSV}"
        )
    return 0 if summary["all_gates_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
