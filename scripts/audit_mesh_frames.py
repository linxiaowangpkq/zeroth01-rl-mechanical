from __future__ import annotations

import csv
import json
import math
import struct
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_ready.urdf"
REPORT_CSV = ROOT / "reports" / "mesh_frame_audit.csv"
REPORT_JSON = ROOT / "reports" / "mesh_frame_audit.json"

IDENTITY = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]


def parse_vec(text: str | None) -> list[float]:
    return [float(value) for value in text.split()] if text else [0.0, 0.0, 0.0]


def mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [sum(a[row][index] * b[index][column] for index in range(3)) for column in range(3)]
        for row in range(3)
    ]


def mat_vec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def vec_add(a: list[float], b: list[float]) -> list[float]:
    return [a[index] + b[index] for index in range(3)]


def rpy_matrix(rpy: list[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]]
    ry = [[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]]
    rz = [[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]]
    return mat_mul(rz, mat_mul(ry, rx))


def tf_mul(
    a: tuple[list[list[float]], list[float]],
    b: tuple[list[list[float]], list[float]],
) -> tuple[list[list[float]], list[float]]:
    ar, at = a
    br, bt = b
    return mat_mul(ar, br), vec_add(mat_vec(ar, bt), at)


def distance(a: list[float], b: list[float]) -> float:
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def transformed_bounds(
    minimum: list[float],
    maximum: list[float],
    transform: tuple[list[list[float]], list[float]],
) -> tuple[list[float], list[float]]:
    rotation, translation = transform
    corners = []
    for x in (minimum[0], maximum[0]):
        for y in (minimum[1], maximum[1]):
            for z in (minimum[2], maximum[2]):
                corners.append(vec_add(mat_vec(rotation, [x, y, z]), translation))
    return (
        [min(corner[axis] for corner in corners) for axis in range(3)],
        [max(corner[axis] for corner in corners) for axis in range(3)],
    )


def aabb_gap(
    a_minimum: list[float],
    a_maximum: list[float],
    b_minimum: list[float],
    b_maximum: list[float],
) -> float:
    separated = [
        max(0.0, a_minimum[axis] - b_maximum[axis], b_minimum[axis] - a_maximum[axis])
        for axis in range(3)
    ]
    return math.sqrt(sum(value * value for value in separated))


def binary_stl_bounds(path: Path) -> tuple[list[float], list[float], int]:
    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    expected_bytes = 84 + 50 * triangle_count
    if len(payload) != expected_bytes:
        raise ValueError(
            f"only binary STL is supported: {path}; "
            f"bytes={len(payload)} expected={expected_bytes}"
        )
    minimum = [math.inf, math.inf, math.inf]
    maximum = [-math.inf, -math.inf, -math.inf]
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack_from("<12fH", payload, offset)
        for vertex_offset in (3, 6, 9):
            for axis in range(3):
                value = float(values[vertex_offset + axis])
                minimum[axis] = min(minimum[axis], value)
                maximum[axis] = max(maximum[axis], value)
        offset += 50
    return minimum, maximum, triangle_count


def neutral_transforms(root: ET.Element) -> dict[str, tuple[list[list[float]], list[float]]]:
    transforms = {"base": ([row[:] for row in IDENTITY], [0.0, 0.0, 0.0])}
    pending = list(root.findall("joint"))
    while pending:
        changed = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link", "")
            if parent not in transforms:
                continue
            origin = joint.find("origin")
            joint_tf = (
                rpy_matrix(parse_vec(origin.get("rpy") if origin is not None else None)),
                parse_vec(origin.get("xyz") if origin is not None else None),
            )
            transforms[joint.find("child").get("link", "")] = tf_mul(
                transforms[parent], joint_tf
            )
            pending.remove(joint)
            changed = True
        if not changed:
            raise ValueError("URDF tree did not resolve")
    return transforms


def main() -> None:
    root = ET.parse(URDF).getroot()
    transforms = neutral_transforms(root)
    rows: list[dict[str, object]] = []
    mesh_bounds: dict[str, tuple[list[float], list[float]]] = {}
    for link in root.findall("link"):
        name = link.get("name", "")
        mesh = link.find("./visual/geometry/mesh")
        if mesh is None:
            continue
        mesh_path = (URDF.parent / mesh.get("filename", "")).resolve()
        minimum, maximum, triangle_count = binary_stl_bounds(mesh_path)
        mesh_bounds[name] = (minimum, maximum)
        center = [(minimum[index] + maximum[index]) / 2.0 for index in range(3)]
        inertial_origin = link.find("./inertial/origin")
        local_com = parse_vec(
            inertial_origin.get("xyz") if inertial_origin is not None else None
        )
        rotation, translation = transforms[name]
        world_com = vec_add(mat_vec(rotation, local_com), translation)
        current_world_center = vec_add(mat_vec(rotation, center), translation)
        local_error = distance(center, local_com)
        world_baked_error = distance(center, world_com)
        rows.append(
            {
                "link": name,
                "mesh": mesh_path.name,
                "triangles": triangle_count,
                "bbox_center_file_m": " ".join(f"{value:.9f}" for value in center),
                "inertial_com_link_m": " ".join(f"{value:.9f}" for value in local_com),
                "inertial_com_world_m": " ".join(f"{value:.9f}" for value in world_com),
                "current_render_bbox_center_world_m": " ".join(
                    f"{value:.9f}" for value in current_world_center
                ),
                "bbox_center_to_local_com_m": local_error,
                "bbox_center_to_world_com_m": world_baked_error,
                "inertial_com_inside_mesh_bbox": all(
                    minimum[axis] - 1e-6 <= local_com[axis] <= maximum[axis] + 1e-6
                    for axis in range(3)
                ),
                "closer_frame": (
                    "LINK_LOCAL"
                    if local_error + 1e-9 < world_baked_error
                    else "WORLD_BAKED"
                ),
            }
        )

    joint_rows: list[dict[str, object]] = []
    for joint in root.findall("joint"):
        parent = joint.find("parent").get("link", "")
        child = joint.find("child").get("link", "")
        if parent not in mesh_bounds or child not in mesh_bounds:
            continue
        parent_minimum, parent_maximum = transformed_bounds(
            *mesh_bounds[parent], transforms[parent]
        )
        child_minimum, child_maximum = transformed_bounds(
            *mesh_bounds[child], transforms[child]
        )
        joint_rows.append(
            {
                "joint": joint.get("name", ""),
                "parent": parent,
                "child": child,
                "neutral_world_aabb_gap_m": aabb_gap(
                    parent_minimum,
                    parent_maximum,
                    child_minimum,
                    child_maximum,
                ),
            }
        )

    REPORT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    counts = {
        label: sum(row["closer_frame"] == label for row in rows)
        for label in ("LINK_LOCAL", "WORLD_BAKED")
    }
    summary = {
        "urdf": str(URDF),
        "mesh_count": len(rows),
        "closer_frame_counts": counts,
        "median_bbox_center_to_local_com_m": sorted(
            float(row["bbox_center_to_local_com_m"]) for row in rows
        )[len(rows) // 2],
        "median_bbox_center_to_world_com_m": sorted(
            float(row["bbox_center_to_world_com_m"]) for row in rows
        )[len(rows) // 2],
        "inertial_com_outside_mesh_bbox": [
            str(row["link"])
            for row in rows
            if not bool(row["inertial_com_inside_mesh_bbox"])
        ],
        "neutral_parent_child_aabb_gaps_over_5mm": [
            row for row in joint_rows if float(row["neutral_world_aabb_gap_m"]) > 0.005
        ],
        "joint_aabb_gaps": joint_rows,
        "report_csv": str(REPORT_CSV),
    }
    REPORT_JSON.write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
