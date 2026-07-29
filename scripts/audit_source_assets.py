from __future__ import annotations

import csv
from collections import Counter
import hashlib
import json
import struct
import subprocess
from pathlib import Path


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
MESH_DIR = ROOT / "source_assets" / "stompymicro" / "meshes"
MAPPING_PATH = ROOT / "config" / "mesh_name_map.json"
SIM_REPO = ROOT / "upstream" / "zeroth-sim"
BOT_REPO = ROOT / "upstream" / "zeroth-bot"
DOCS_REPO = ROOT / "upstream" / "kscale-docs"
KOS_REPO = ROOT / "upstream" / "zeroth-bot" / "kos-zbot"
ASSETS_REPO = ROOT / "upstream" / "kscale-assets"
REPORT_CSV = ROOT / "reports" / "source_asset_manifest.csv"
REPORT_JSON = ROOT / "reports" / "source_lock.json"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_head(repo: Path) -> str:
    result = subprocess.run(
        [
            "git",
            "-c",
            f"safe.directory={repo.as_posix()}",
            "-C",
            str(repo),
            "rev-parse",
            "HEAD",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def binary_stl_stats(path: Path) -> dict[str, object]:
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"STL is too short: {path}")
    face_count = struct.unpack_from("<I", payload, 80)[0]
    expected_bytes = 84 + 50 * face_count
    if len(payload) != expected_bytes:
        raise ValueError(
            f"only binary STL is supported: {path}; "
            f"bytes={len(payload)} expected={expected_bytes}"
        )
    vertex_ids: dict[tuple[float, float, float], int] = {}
    parent: list[int] = []
    minimum = [float("inf")] * 3
    maximum = [float("-inf")] * 3
    edges: Counter[tuple[int, int]] = Counter()
    edge_orientation: Counter[tuple[int, int]] = Counter()

    def vertex_id(values: tuple[float, float, float]) -> int:
        key = tuple(round(float(value), 9) for value in values)
        if key not in vertex_ids:
            vertex_ids[key] = len(parent)
            parent.append(len(parent))
        return vertex_ids[key]

    def find(value: int) -> int:
        while parent[value] != value:
            parent[value] = parent[parent[value]]
            value = parent[value]
        return value

    def union(first: int, second: int) -> None:
        first_root = find(first)
        second_root = find(second)
        if first_root != second_root:
            parent[second_root] = first_root

    offset = 84
    for _ in range(face_count):
        record = struct.unpack_from("<12fH", payload, offset)
        vertices = [
            tuple(float(record[start + axis]) for axis in range(3))
            for start in (3, 6, 9)
        ]
        ids = [vertex_id(vertex) for vertex in vertices]
        union(ids[0], ids[1])
        union(ids[1], ids[2])
        for vertex in vertices:
            for axis in range(3):
                minimum[axis] = min(minimum[axis], vertex[axis])
                maximum[axis] = max(maximum[axis], vertex[axis])
        for first, second in ((ids[0], ids[1]), (ids[1], ids[2]), (ids[2], ids[0])):
            edge = tuple(sorted((first, second)))
            edges[edge] += 1
            edge_orientation[edge] += 1 if (first, second) == edge else -1
        offset += 50

    body_count = len({find(index) for index in range(len(parent))})
    return {
        "vertices": len(parent),
        "faces": face_count,
        "body_count": body_count,
        "watertight": bool(edges) and all(count == 2 for count in edges.values()),
        "winding_consistent": all(
            count == 1 or edge_orientation[edge] == 0
            for edge, count in edges.items()
        ),
        "extents": [maximum[axis] - minimum[axis] for axis in range(3)],
    }


def main() -> None:
    mapping_payload = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
    mapping: dict[str, str] = mapping_payload["target_to_downloaded"]
    expected_downloads = set(mapping.values())
    actual_downloads = {path.name for path in MESH_DIR.glob("*.stl")}
    missing = sorted(expected_downloads - actual_downloads)
    extra = sorted(actual_downloads - expected_downloads)
    if missing or extra:
        raise RuntimeError(f"mesh set mismatch: missing={missing}; extra={extra}")

    rows: list[dict[str, str]] = []
    for target_name, downloaded_name in sorted(mapping.items()):
        path = MESH_DIR / downloaded_name
        stats = binary_stl_stats(path)
        extents = stats["extents"]
        rows.append(
            {
                "target_name": target_name,
                "downloaded_name": downloaded_name,
                "sha256": sha256(path),
                "bytes": str(path.stat().st_size),
                "vertices": str(stats["vertices"]),
                "faces": str(stats["faces"]),
                "body_count": str(stats["body_count"]),
                "watertight": str(bool(stats["watertight"])).lower(),
                "winding_consistent": str(
                    bool(stats["winding_consistent"])
                ).lower(),
                "extent_x_m": f"{extents[0]:.9f}",
                "extent_y_m": f"{extents[1]:.9f}",
                "extent_z_m": f"{extents[2]:.9f}",
            }
        )

    REPORT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    lock = {
        "zeroth_bot_head": git_head(BOT_REPO),
        "zeroth_sim_head": git_head(SIM_REPO),
        "kscale_docs_head": git_head(DOCS_REPO),
        "kos_zbot_head": git_head(KOS_REPO),
        "kscale_assets_head": git_head(ASSETS_REPO),
        "geometry_compatible_urdf_commit": mapping_payload["evidence_commit"],
        "newer_incompatible_without_replacement_meshes_commit": (
            "43c5baa1287db078bef638308ef077445704be1d"
        ),
        "mesh_mapping_evidence_commit": mapping_payload["evidence_commit"],
        "mesh_count": len(rows),
        "all_winding_consistent": all(
            row["winding_consistent"] == "true" for row in rows
        ),
        "all_watertight": all(row["watertight"] == "true" for row in rows),
        "manifest": str(REPORT_CSV),
    }
    REPORT_JSON.write_text(json.dumps(lock, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(lock, indent=2))


if __name__ == "__main__":
    main()
