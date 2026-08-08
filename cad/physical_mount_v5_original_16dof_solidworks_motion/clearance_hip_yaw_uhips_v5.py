"""Cut minimal full-limit clearance for the fixed hip-yaw mounts and screws.

The original 16DoF axes and current RL limits remain unchanged.  Only the
small internal volumes swept past the new torso-integrated case-mount ribs and
two M2x8 screws are removed from U_HIP_L/R.  The output horn, exterior datum,
and neutral link transform are not moved or rescaled.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
from build123d import Align, Box, Compound, Cylinder, Location, export_step, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Cut
from OCP.BRepBuilderAPI import (
    BRepBuilderAPI_MakeFace,
    BRepBuilderAPI_MakePolygon,
    BRepBuilderAPI_Transform,
)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepPrimAPI import BRepPrimAPI_MakePrism
from OCP.gp import gp_Ax1, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec

import diagnose_v5_offline_brep_interference as gate


ROOT = gate.ROOT
PARTS = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "parts"
BASE = PARTS / "source_carriers_sts3250_clearanced"
OUT = PARTS / "source_carriers_sts3250_clearanced"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "hip_yaw_uhip_mount_clearance.json"
CLEARANCE_MM = 0.30
SWEEP_SAMPLES = 33
VALIDATION_SAMPLES = 17


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        *[float(matrix[row][column]) for row in range(3) for column in range(4)]
    )
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def rotated_z(shape, angle_rad: float):
    transform = gp_Trsf()
    transform.SetRotation(
        gp_Ax1(gp_Pnt(0.0, 0.0, 0.0), gp_Dir(0.0, 0.0, 1.0)),
        angle_rad,
    )
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def cut_shape(base, tool):
    operation = BRepAlgoAPI_Cut(base.wrapped, tool.wrapped)
    operation.SetNonDestructive(True)
    operation.SetRunParallel(True)
    operation.SetUseOBB(True)
    operation.SetFuzzyValue(1.0e-7)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT U-hip fixed-mount clearance cut failed")
    return Compound(operation.Shape())


def fixed_obstacle(clearance_mm: float):
    """Outer manufacturing envelope of ribs, bosses, and installed screws."""

    pieces = []
    for y_mm in (-10.25, 10.25):
        sign = -1.0 if y_mm < 0.0 else 1.0
        outer_y = sign * 28.0
        center_y = (y_mm + outer_y) / 2.0
        pieces.append(
            Box(
                6.4 + 2.0 * clearance_mm,
                abs(outer_y - y_mm) + 2.0 * clearance_mm,
                4.0 + 2.0 * clearance_mm,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Location((-28.5, center_y, -37.45)))
        )
        pieces.append(
            Cylinder(
                3.2 + clearance_mm,
                4.0 + 2.0 * clearance_mm,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Location((-28.5, y_mm, -39.45 - clearance_mm)))
        )
        pieces.append(
            Cylinder(
                0.95 + clearance_mm,
                8.0 + 2.0 * clearance_mm,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Location((-28.5, y_mm, -39.45 - clearance_mm)))
        )
        pieces.append(
            Cylinder(
                2.0 + clearance_mm,
                1.5 + 2.0 * clearance_mm,
                align=(Align.CENTER, Align.CENTER, Align.MAX),
            ).moved(Location((-28.5, y_mm, -39.45 + clearance_mm)))
        )
    return Compound(children=pieces)


def rotate_xy(point: tuple[float, float], angle_rad: float) -> tuple[float, float]:
    x_mm, y_mm = point
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return (cosine * x_mm - sine * y_mm, sine * x_mm + cosine * y_mm)


def convex_hull(points: list[tuple[float, float]]) -> list[tuple[float, float]]:
    ordered = sorted(set(points))
    if len(ordered) < 3:
        raise RuntimeError("swept footprint has fewer than three unique points")

    def cross(origin, first, second):
        return (
            (first[0] - origin[0]) * (second[1] - origin[1])
            - (first[1] - origin[1]) * (second[0] - origin[0])
        )

    lower = []
    for point in ordered:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0.0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(ordered):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0.0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def prism(points_xy: list[tuple[float, float]], z_min_mm: float, z_max_mm: float):
    hull = convex_hull(points_xy)
    polygon = BRepBuilderAPI_MakePolygon()
    for x_mm, y_mm in hull:
        polygon.Add(gp_Pnt(x_mm, y_mm, z_min_mm))
    polygon.Close()
    if not polygon.IsDone():
        raise RuntimeError("swept clearance polygon construction failed")
    face = BRepBuilderAPI_MakeFace(polygon.Wire(), True)
    if not face.IsDone():
        raise RuntimeError("swept clearance face construction failed")
    return Compound(
        BRepPrimAPI_MakePrism(
            face.Face(), gp_Vec(0.0, 0.0, z_max_mm - z_min_mm), True
        ).Shape()
    )


def swept_clearance_tools(lower: float, upper: float):
    """Eight conservative continuous swept prisms in the joint frame."""

    angles = np.linspace(-upper, -lower, SWEEP_SAMPLES)
    tools = []
    clearance = CLEARANCE_MM
    for y_mm in (-10.25, 10.25):
        sign = -1.0 if y_mm < 0.0 else 1.0
        outer_y = sign * 28.0
        center = (-28.5, (y_mm + outer_y) / 2.0)
        width = 6.4 + 2.0 * clearance
        height = abs(outer_y - y_mm) + 2.0 * clearance
        corners = [
            (center[0] + sx * width / 2.0, center[1] + sy * height / 2.0)
            for sx in (-1.0, 1.0)
            for sy in (-1.0, 1.0)
        ]
        rib_points = [rotate_xy(corner, float(angle)) for angle in angles for corner in corners]
        tools.append(prism(rib_points, -39.75, -35.15))

        for radius, z_min, z_max in (
            (3.2 + clearance, -39.75, -35.15),
            (0.95 + clearance, -39.75, -31.05),
            (2.0 + clearance, -41.25, -39.15),
        ):
            circle_points = []
            for angle in angles:
                rotated_center = rotate_xy((-28.5, y_mm), float(angle))
                for phase in np.linspace(0.0, 2.0 * math.pi, 25)[:-1]:
                    circle_points.append(
                        (
                            rotated_center[0] + radius * math.cos(float(phase)),
                            rotated_center[1] + radius * math.sin(float(phase)),
                        )
                    )
            tools.append(prism(circle_points, z_min, z_max))
    return tools


def main() -> int:
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    rows = []
    for side, sid, owner in (
        ("right", "S07", "U_HIP_R"),
        ("left", "S08", "U_HIP_L"),
    ):
        joint = f"{side}_hip_yaw"
        servo_id = f"{sid}_STS3250_{joint}"
        source = BASE / f"{owner}_STS3250_CLEARANCED.step"
        if not source.is_file():
            source = PARTS / "source_carriers_solid" / f"{owner}_SOLID.step"
        carrier = import_step(source)
        initial_volume = float(carrier.volume)

        owner_row = by_id[f"CARRIER_{owner}"]
        owner_world = np.asarray(owner_row["transform_local_mm_to_world_mm"], dtype=float)
        servo_world = np.asarray(by_id[servo_id]["transform_local_mm_to_world_mm"], dtype=float)
        joint_world = servo_world.copy()
        joint_world[:3, 3] -= joint_world[:3, 2] * 4.0
        joint_to_owner = np.linalg.inv(owner_world) @ joint_world
        spec = next(row for row in data["joint_specs"] if row["name"] == joint)
        lower, upper = (float(value) for value in spec["limits"])

        tools = [matrix_shape(tool, joint_to_owner) for tool in swept_clearance_tools(lower, upper)]
        swept_tool_initial_overlaps = []
        applied_cut_count = 0
        for tool in tools:
            overlap = gate.common_volume(carrier, tool)
            swept_tool_initial_overlaps.append(overlap)
            if overlap > gate.VOLUME_TOLERANCE_MM3:
                carrier = cut_shape(carrier, tool)
                applied_cut_count += 1
        target = OUT / f"{owner}_MOUNT_CLEARANCED.step"
        export_step(carrier, target)

        exact_obstacle = fixed_obstacle(0.0)
        validation = []
        for angle in np.linspace(lower, upper, VALIDATION_SAMPLES):
            fixed_in_owner = matrix_shape(
                rotated_z(exact_obstacle, -float(angle)), joint_to_owner
            )
            validation.append(
                {
                    "angle_rad": float(angle),
                    "angle_deg": math.degrees(float(angle)),
                    "remaining_overlap_mm3": gate.common_volume(carrier, fixed_in_owner),
                }
            )
        final_volume = float(carrier.volume)
        max_overlap = max(row["remaining_overlap_mm3"] for row in validation)
        valid = bool(BRepCheck_Analyzer(carrier.wrapped).IsValid())
        rows.append(
            {
                "side": side,
                "joint": joint,
                "owner_link": owner,
                "source": source.relative_to(ROOT).as_posix(),
                "target": target.relative_to(ROOT).as_posix(),
                "limits_rad": [lower, upper],
                "sweep_samples": SWEEP_SAMPLES,
                "swept_tool_count": len(tools),
                "applied_cut_count": applied_cut_count,
                "swept_tool_initial_overlaps_mm3": swept_tool_initial_overlaps,
                "validation_samples": VALIDATION_SAMPLES,
                "clearance_mm": CLEARANCE_MM,
                "initial_volume_mm3": initial_volume,
                "final_volume_mm3": final_volume,
                "removed_volume_mm3": initial_volume - final_volume,
                "max_remaining_overlap_mm3": max_overlap,
                "occt_valid": valid,
                "joint_axis_changed": False,
                "neutral_transform_changed": False,
                "validation": validation,
                "status": "PASS" if (
                    valid
                    and final_volume > 0.0
                    and max_overlap <= gate.VOLUME_TOLERANCE_MM3
                ) else "FAIL",
            }
        )

    payload = {
        "schema": "zeroth01.v5.hip_yaw_uhip_mount_clearance.v1",
        "clearance_mm": CLEARANCE_MM,
        "rows": rows,
        "overall": "PASS" if all(row["status"] == "PASS" for row in rows) else "FAIL",
    }
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "rows": [
                    {
                        "side": row["side"],
                        "removed_volume_mm3": row["removed_volume_mm3"],
                        "max_remaining_overlap_mm3": row["max_remaining_overlap_mm3"],
                        "status": row["status"],
                    }
                    for row in rows
                ],
                "overall": payload["overall"],
            },
            indent=2,
        ),
        flush=True,
    )
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
