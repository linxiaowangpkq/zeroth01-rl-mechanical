from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
import trimesh


ROOT = Path(__file__).resolve().parents[1]
CAD_SOURCE_DIR = ROOT / "cad" / "round_v1"
sys.path.insert(0, str(CAD_SOURCE_DIR))

from round_v1_common import (  # noqa: E402
    _mat_mul,
    _mat_vec,
    load_neutral_kinematics,
)


URDF_MESH_DIR = ROOT / "generated" / "urdf" / "meshes"
VENDOR_STL = (
    ROOT
    / "generated"
    / "print"
    / "round_v1"
    / "FEETECH_STS3250_VENDOR_REFERENCE.stl"
)
OUTPUT_JSON = (
    ROOT
    / "generated"
    / "config"
    / "zeroth01_sts3250_mount_phase.json"
)
REPORT_CSV = ROOT / "reports" / "sts3250_mount_phase_fit.csv"


def transform_points(
    vertices: np.ndarray,
    rotation: list[list[float]],
    translation: list[float],
) -> np.ndarray:
    matrix = np.asarray(rotation, dtype=float)
    return vertices @ matrix.T + np.asarray(translation, dtype=float)


def load_link_vertices(
    link: str,
    transforms: dict[str, tuple[list[list[float]], list[float]]],
) -> np.ndarray:
    mesh = trimesh.load(
        URDF_MESH_DIR / f"{link}.stl",
        force="mesh",
        process=True,
        validate=True,
    )
    if not isinstance(mesh, trimesh.Trimesh):
        raise TypeError(link)
    rotation, translation = transforms[link]
    return transform_points(np.asarray(mesh.vertices), rotation, translation)


def rotation_z(angle_rad: float) -> list[list[float]]:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]


def fit() -> dict[str, object]:
    joints, transforms = load_neutral_kinematics()
    moving = [
        item for item in joints if item["type"] in {"revolute", "continuous"}
    ]
    vendor = trimesh.load(
        VENDOR_STL,
        force="mesh",
        process=True,
        validate=True,
    )
    if not isinstance(vendor, trimesh.Trimesh):
        raise TypeError(VENDOR_STL)
    vendor_vertices_m = np.asarray(vendor.vertices, dtype=float) / 1000.0
    if len(vendor_vertices_m) > 5000:
        sample_indices = np.linspace(
            0,
            len(vendor_vertices_m) - 1,
            5000,
            dtype=int,
        )
        vendor_vertices_m = vendor_vertices_m[sample_indices]

    link_cache: dict[str, np.ndarray] = {}
    rows: list[dict[str, object]] = []
    phase_map: dict[str, dict[str, object]] = {}
    flip_x = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]

    for joint in moving:
        name = str(joint["name"])
        parent = str(joint["parent"])
        child = str(joint["child"])
        for link in (parent, child):
            if link not in link_cache and (URDF_MESH_DIR / f"{link}.stl").is_file():
                link_cache[link] = load_link_vertices(link, transforms)
        target_sets = [
            link_cache[link]
            for link in (parent, child)
            if link in link_cache
        ]
        if not target_sets:
            raise RuntimeError(f"no target geometry for {name}")
        target = np.vstack(target_sets)
        tree = cKDTree(target)
        target_min = target.min(axis=0) - 0.004
        target_max = target.max(axis=0) + 0.004

        joint_rotation, joint_translation = transforms[child]
        axis_local = [float(value) for value in joint["axis"]]
        positive_base = (
            _mat_mul(joint_rotation, flip_x)
            if axis_local[2] < 0.0
            else joint_rotation
        )
        candidates: list[dict[str, object]] = []
        for output_axis_sign in (1, -1):
            sign_base = (
                positive_base
                if output_axis_sign == 1
                else _mat_mul(positive_base, flip_x)
            )
            for phase_deg in range(0, 360, 5):
                candidate_rotation = _mat_mul(
                    sign_base,
                    rotation_z(math.radians(phase_deg)),
                )
                world_vertices = transform_points(
                    vendor_vertices_m,
                    candidate_rotation,
                    joint_translation,
                )
                distances = tree.query(world_vertices, k=1, workers=-1)[0]
                inside_padded_bbox = np.logical_and(
                    world_vertices >= target_min,
                    world_vertices <= target_max,
                ).all(axis=1)
                median_mm = float(np.median(distances) * 1000.0)
                p90_mm = float(np.percentile(distances, 90) * 1000.0)
                within_bbox_pct = float(inside_padded_bbox.mean() * 100.0)
                score = (
                    median_mm
                    + 0.25 * p90_mm
                    + 0.20 * (100.0 - within_bbox_pct)
                )
                candidates.append(
                    {
                        "phase_deg": phase_deg,
                        "output_axis_sign": output_axis_sign,
                        "rotation": candidate_rotation,
                        "median_mm": median_mm,
                        "p90_mm": p90_mm,
                        "match_lt_1mm_pct": float(
                            np.mean(distances < 0.001) * 100.0
                        ),
                        "match_lt_3mm_pct": float(
                            np.mean(distances < 0.003) * 100.0
                        ),
                        "within_bbox_4mm_pct": within_bbox_pct,
                        "score": score,
                    }
                )
        candidates.sort(key=lambda item: float(item["score"]))
        best = candidates[0]
        next_best = candidates[1]
        phase_confidence = (
            "MEDIUM_GEOMETRY_FIT"
            if float(best["median_mm"]) <= 3.0
            and float(best["match_lt_3mm_pct"]) >= 40.0
            else "LOW_REQUIRES_HARDWARE_OR_SOURCE_CAD_CONFIRMATION"
        )
        phase_map[name] = {
            "phase_deg": int(best["phase_deg"]),
            "output_axis_sign": int(best["output_axis_sign"]),
            "confidence": phase_confidence,
            "method": (
                "5-degree exhaustive phase search against neutral parent+child "
                "aggregate STL vertex surfaces; shaft origin fixed to URDF joint"
            ),
        }
        rows.append(
            {
                "joint": name,
                "parent": parent,
                "child": child,
                "phase_deg": best["phase_deg"],
                "output_axis_sign": best["output_axis_sign"],
                "median_surface_distance_mm": f"{best['median_mm']:.6f}",
                "p90_surface_distance_mm": f"{best['p90_mm']:.6f}",
                "match_lt_1mm_pct": f"{best['match_lt_1mm_pct']:.3f}",
                "match_lt_3mm_pct": f"{best['match_lt_3mm_pct']:.3f}",
                "within_parent_child_bbox_plus_4mm_pct": (
                    f"{best['within_bbox_4mm_pct']:.3f}"
                ),
                "score": f"{best['score']:.6f}",
                "next_best_score_margin": (
                    f"{float(next_best['score']) - float(best['score']):.6f}"
                ),
                "confidence": phase_confidence,
            }
        )

    REPORT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    payload = {
        "servo_model": "Feetech STS3250",
        "vendor_mesh": str(VENDOR_STL),
        "phase_is_hardware_verified": False,
        "phase_semantics": (
            "Best geometric fit only. The URDF does not encode housing phase; "
            "confirm each phase against native Onshape/SolidWorks source or hardware."
        ),
        "joint_mount_phase": phase_map,
        "report": str(REPORT_CSV),
    }
    OUTPUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_JSON.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return payload


if __name__ == "__main__":
    result = fit()
    counts: dict[str, int] = {}
    for item in result["joint_mount_phase"].values():
        confidence = str(item["confidence"])
        counts[confidence] = counts.get(confidence, 0) + 1
    print(
        f"fitted={len(result['joint_mount_phase'])} confidence_counts={counts} "
        f"report={REPORT_CSV}"
    )
