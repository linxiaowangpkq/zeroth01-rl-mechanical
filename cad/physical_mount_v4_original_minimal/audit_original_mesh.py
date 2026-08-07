"""Measure connected components in the official Zeroth-01 torso STL.

This audit is intentionally independent of the faceted STEP conversion.  It
uses shared, quantised mesh vertices to recover connected source components
and emits only measurements; it does not repair or rewrite source geometry.
"""

from __future__ import annotations

import json
import struct
from collections import defaultdict
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v1"
    / "skeleton"
    / "Z_BOT2_MASTER_BODY_SKELETON.stl"
)
REPORT = ROOT / "reports" / "v4_original_minimal" / "audit" / "torso_mesh_components.json"


def read_binary_stl(path: Path) -> np.ndarray:
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    expected = 84 + 50 * triangle_count
    if len(payload) != expected:
        raise ValueError(f"binary STL expected {expected} bytes, got {len(payload)}")
    record = np.dtype(
        [
            ("normal", "<f4", (3,)),
            ("vertices", "<f4", (3, 3)),
            ("attribute", "<u2"),
        ]
    )
    return np.frombuffer(payload, dtype=record, count=triangle_count, offset=84)[
        "vertices"
    ].astype(np.float64)


def connected_components(triangles: np.ndarray, tolerance: float = 1.0e-5):
    quantised = np.rint(triangles.reshape(-1, 3) / tolerance).astype(np.int64)
    _, inverse = np.unique(quantised, axis=0, return_inverse=True)
    triangle_vertices = inverse.reshape(-1, 3)

    parent = np.arange(len(triangles), dtype=np.int64)

    def find(index: int) -> int:
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = int(parent[index])
        return index

    def union(left: int, right: int) -> None:
        left_root, right_root = find(left), find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    by_vertex: dict[int, list[int]] = defaultdict(list)
    for triangle_index, vertices in enumerate(triangle_vertices):
        for vertex in vertices:
            by_vertex[int(vertex)].append(triangle_index)
    for incident in by_vertex.values():
        first = incident[0]
        for other in incident[1:]:
            union(first, other)

    groups: dict[int, list[int]] = defaultdict(list)
    for triangle_index in range(len(triangles)):
        groups[find(triangle_index)].append(triangle_index)
    return list(groups.values())


def main() -> int:
    triangles = read_binary_stl(SOURCE)
    rows = []
    for indices in connected_components(triangles):
        # The released URDF/STL bundle is authored in metres, while the
        # mechanical STEP/SolidWorks handoff uses millimetres.
        points = triangles[np.asarray(indices)].reshape(-1, 3) * 1000.0
        minimum = points.min(axis=0)
        maximum = points.max(axis=0)
        size = maximum - minimum
        rows.append(
            {
                "triangle_count": len(indices),
                "minimum_mm": minimum.tolist(),
                "maximum_mm": maximum.tolist(),
                "size_mm": size.tolist(),
                "center_mm": ((minimum + maximum) / 2.0).tolist(),
            }
        )
    rows.sort(key=lambda row: row["triangle_count"], reverse=True)
    payload = {
        "schema": "zeroth01.v4.original_mesh_component_audit.v1",
        "source": SOURCE.relative_to(ROOT).as_posix(),
        "triangle_count": int(len(triangles)),
        "connected_component_count": len(rows),
        "components": rows,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(REPORT)
    for index, row in enumerate(rows[:20]):
        print(index, row)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
