from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import trimesh
from build123d import import_step


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
RAW_DIR = (
    ROOT
    / "generated"
    / "print"
    / "round_v1"
    / "fit_check_non_load_bearing"
)
FINAL_DIR = RAW_DIR / "final"
STEP_DIR = ROOT / "generated" / "cad" / "round_v1" / "parts"
REPORT = ROOT / "reports" / "round_v1_interface_fit_mesh_gate.json"

FILES = {
    "ZEROTH01_ROUND_V1_SERVO_CAGE_FIT_CHECK_ONLY.stl": (
        "ZEROTH01_ROUND_V1_SERVO_CAGE.step"
    ),
    "ZEROTH01_ROUND_V1_OUTPUT_HUB_FRONT_FIT_CHECK_ONLY.stl": (
        "ZEROTH01_ROUND_V1_OUTPUT_HUB_FRONT.step"
    ),
    "ZEROTH01_ROUND_V1_OUTPUT_HUB_REAR_FIT_CHECK_ONLY.stl": (
        "ZEROTH01_ROUND_V1_OUTPUT_HUB_REAR.step"
    ),
}

MIN_COMPONENT_VOLUME_MM3 = 1e-3
MAX_STEP_VOLUME_ERROR_RATIO = 0.005


def edge_counts(mesh: trimesh.Trimesh) -> tuple[int, int]:
    counts = np.bincount(
        mesh.edges_unique_inverse,
        minlength=len(mesh.edges_unique),
    )
    return int(np.sum(counts == 1)), int(np.sum(counts > 2))


def main() -> int:
    FINAL_DIR.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []

    for stl_name, step_name in FILES.items():
        source = RAW_DIR / stl_name
        step = STEP_DIR / step_name
        if not source.is_file():
            raise FileNotFoundError(source)
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

        check = trimesh.load_mesh(output, process=True)
        if not isinstance(check, trimesh.Trimesh):
            raise TypeError(f"expected one final mesh: {output}")
        boundary_edges, nonmanifold_edges = edge_counts(check)
        components_after = list(check.split(only_watertight=False))
        step_volume = float(import_step(step).volume)
        mesh_volume = float(check.volume)
        volume_error = abs(mesh_volume - step_volume) / step_volume
        passed = bool(
            check.is_watertight
            and check.is_winding_consistent
            and boundary_edges == 0
            and nonmanifold_edges == 0
            and len(components_after) == 1
            and volume_error <= MAX_STEP_VOLUME_ERROR_RATIO
        )
        rows.append(
            {
                "part": source.stem,
                "raw_stl": source.relative_to(ROOT).as_posix(),
                "final_stl": output.relative_to(ROOT).as_posix(),
                "raw_component_count": len(components),
                "removed_zero_volume_artifacts": (
                    len(components) - len(meaningful)
                ),
                "final_component_count": len(components_after),
                "final_watertight": bool(check.is_watertight),
                "final_winding_consistent": bool(
                    check.is_winding_consistent
                ),
                "boundary_edge_count": boundary_edges,
                "nonmanifold_edge_count": nonmanifold_edges,
                "step_volume_mm3": step_volume,
                "mesh_volume_mm3": mesh_volume,
                "step_mesh_volume_error_ratio": volume_error,
                "scope": (
                    "dimensional fit check only; production load path "
                    "requires CNC 6061-T6 and hardware qualification"
                ),
                "gate": "PASS" if passed else "FAIL",
            }
        )

    overall = all(row["gate"] == "PASS" for row in rows)
    payload = {
        "schema": "zeroth01.round_v1.interface_fit_mesh_gate.v1",
        "part_count": len(rows),
        "mesh_topology_gate": "PASS" if overall else "FAIL",
        "functional_load_path_gate": (
            "BLOCKED_PENDING_PURCHASED_HORN_FASTENER_BEARING_CABLE_AND_"
            "TOLERANCE_RFQ"
        ),
        "rows": rows,
    }
    REPORT.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if overall else 2


if __name__ == "__main__":
    raise SystemExit(main())
