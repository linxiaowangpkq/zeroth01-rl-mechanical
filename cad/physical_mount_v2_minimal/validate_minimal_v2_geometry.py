"""Validate print geometry and derive RL mass properties for v2-minimal."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np
from build123d import CenterOf, import_step
from vtkmodules.vtkFiltersCore import vtkCleanPolyData, vtkFeatureEdges, vtkMassProperties
from vtkmodules.vtkFiltersGeneral import vtkTransformPolyDataFilter
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkIOGeometry import vtkSTLReader

import minimal_v2_common as common


ROOT = Path(__file__).resolve().parents[2]
PART_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
REPLACEMENT_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements"
REPORT_ROOT = ROOT / "reports" / "physical_mount_v2_minimal"
GEOMETRY_REPORT = REPORT_ROOT / "geometry_gate.json"
GEOMETRY_CSV = REPORT_ROOT / "geometry_gate.csv"
MASS_REPORT = ROOT / "generated" / "config" / "physical_mount_v2_minimal_mass_properties.json"
MASS_CSV = REPORT_ROOT / "mass_properties.csv"

PETG_DENSITY_KG_PER_MM3 = 1.27e-6
TPU_DENSITY_KG_PER_MM3 = 1.20e-6


def _read_stl(path: Path, scale: float = 1.0):
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    if scale == 1.0:
        source = reader.GetOutputPort()
    else:
        transform = vtkTransform()
        transform.Scale(scale, scale, scale)
        apply = vtkTransformPolyDataFilter()
        apply.SetInputConnection(reader.GetOutputPort())
        apply.SetTransform(transform)
        apply.Update()
        source = apply.GetOutputPort()
    clean = vtkCleanPolyData()
    clean.SetInputConnection(source)
    clean.Update()
    return clean.GetOutput()


def _defect_edges(polydata) -> int:
    edges = vtkFeatureEdges()
    edges.SetInputData(polydata)
    edges.BoundaryEdgesOn()
    edges.NonManifoldEdgesOn()
    edges.FeatureEdgesOff()
    edges.ManifoldEdgesOff()
    edges.Update()
    return int(edges.GetOutput().GetNumberOfCells())


def _mesh_volume_m3(path: Path) -> float:
    mass = vtkMassProperties()
    mass.SetInputData(_read_stl(path))
    mass.Update()
    return float(mass.GetVolume())


def _inertia_dict(matrix: np.ndarray) -> dict[str, float]:
    return {
        "ixx": float(matrix[0, 0]),
        "iyy": float(matrix[1, 1]),
        "izz": float(matrix[2, 2]),
        "ixy": float(matrix[0, 1]),
        "ixz": float(matrix[0, 2]),
        "iyz": float(matrix[1, 2]),
    }


def main() -> int:
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    MASS_REPORT.parent.mkdir(parents=True, exist_ok=True)
    records = common.part_records()
    geometry_rows: list[dict[str, object]] = []
    mass_rows: list[dict[str, object]] = []
    mass_parts: dict[str, object] = {}
    loaded_shapes: dict[str, object] = {}

    for key, record in records.items():
        step_path = PART_ROOT / f"{key}.step"
        stl_path = PART_ROOT / f"{key}.stl"
        if not step_path.is_file() or not stl_path.is_file():
            raise FileNotFoundError(f"missing STEP/STL pair: {key}")
        shape = import_step(step_path)
        loaded_shapes[key] = shape
        solids = list(shape.solids())
        defects = _defect_edges(_read_stl(stl_path))
        positive = bool(solids) and all(float(item.volume) > 0.0 for item in solids)
        printable_gate = (
            len(solids) == 1 and positive and defects == 0
            if record.printable
            else positive and defects == 0
        )
        bounds = shape.bounding_box()
        size = [bounds.size.X, bounds.size.Y, bounds.size.Z]
        dimensional_gate = True
        dimensional_note = ""
        if key in {"left_q_hand", "right_q_hand"}:
            dimensional_gate = max(size) <= 48.0 and min(size) >= 35.0
            dimensional_note = "compact hand max<=48 mm; no old claw visual"
        elif key in {"left_sole", "right_sole"}:
            dimensional_gate = abs(sorted(size)[0] - 9.0) <= 0.05
            dimensional_note = "replaceable sole thickness=9 mm"
        geometry_gate = printable_gate and dimensional_gate
        geometry_rows.append(
            {
                "part": key,
                "installed_link": record.installed_link,
                "classification": record.classification,
                "printable": record.printable,
                "step_solid_count": len(solids),
                "stl_boundary_or_nonmanifold_edges": defects,
                "bbox_x_mm": size[0],
                "bbox_y_mm": size[1],
                "bbox_z_mm": size[2],
                "dimensional_note": dimensional_note,
                "geometry_gate": "PASS" if geometry_gate else "FAIL",
            }
        )
        if not record.printable:
            continue
        volume_mm3 = float(shape.volume)
        density = (
            TPU_DENSITY_KG_PER_MM3
            if record.material.startswith("TPU")
            else PETG_DENSITY_KG_PER_MM3
        )
        mass_kg = volume_mm3 * density
        center = shape.center(CenterOf.MASS)
        com_m = np.array([center.X, center.Y, center.Z], dtype=float) / 1000.0
        inertia = np.array(shape.matrix_of_inertia, dtype=float) * density * 1e-6
        eigenvalues = np.linalg.eigvalsh(inertia)
        inertia_gate = bool(mass_kg > 0.0 and np.all(eigenvalues > 0.0))
        payload = {
            "installed_link": record.installed_link,
            "classification": record.classification,
            "material_assumption": record.material,
            "density_kg_m3": density * 1e9,
            "volume_mm3": volume_mm3,
            "nominal_mass_kg": mass_kg,
            "com_m": [float(value) for value in com_m],
            "inertia_kg_m2_at_com": _inertia_dict(inertia),
            "inertia_eigenvalues_kg_m2": [float(value) for value in eigenvalues],
            "gate": "PASS" if inertia_gate else "FAIL",
            "hardware_override_required": True,
        }
        mass_parts[key] = payload
        mass_rows.append(
            {
                "part": key,
                "installed_link": record.installed_link,
                "nominal_mass_kg": mass_kg,
                "com_x_m": com_m[0],
                "com_y_m": com_m[1],
                "com_z_m": com_m[2],
                **_inertia_dict(inertia),
                "inertia_gate": payload["gate"],
            }
        )

    replacement_rows = []
    source_root = common.BASE_URDF.parent / "meshes" / "skeleton"
    for source_name, replacement_name in (
        ("R_ARM_MIRROR_1.stl", "R_ARM_MIRROR_1_WRIST_TRIMMED.stl"),
        ("L_ARM_MIRROR_1.stl", "L_ARM_MIRROR_1_WRIST_TRIMMED.stl"),
    ):
        source_path = source_root / source_name
        replacement_path = REPLACEMENT_ROOT / replacement_name
        source_volume = _mesh_volume_m3(source_path)
        replacement_volume = _mesh_volume_m3(replacement_path)
        defects = _defect_edges(_read_stl(replacement_path))
        ratio = replacement_volume / source_volume
        gate = defects == 0 and 0.45 < ratio < 0.95
        replacement_rows.append(
            {
                "source": source_path.relative_to(ROOT).as_posix(),
                "replacement": replacement_path.relative_to(ROOT).as_posix(),
                "source_volume_m3": source_volume,
                "replacement_volume_m3": replacement_volume,
                "retained_volume_ratio": ratio,
                "boundary_or_nonmanifold_edges": defects,
                "gate": "PASS" if gate else "FAIL",
            }
        )

    interface_rows: list[dict[str, object]] = []
    for first_key, second_key, requirement in (
        ("head_front", "head_back", "0.45 mm clamshell seam"),
        ("head_front", "chest_panel", "0.60 mm zero-neck assembly seam"),
        ("head_back", "chest_panel", "0.60 mm zero-neck assembly seam"),
    ):
        intersection = loaded_shapes[first_key].intersect(loaded_shapes[second_key])
        volume_mm3 = (
            0.0
            if intersection is None
            else sum(float(solid.volume) for solid in intersection.solids())
        )
        interface_rows.append(
            {
                "first_part": first_key,
                "second_part": second_key,
                "requirement": requirement,
                "intersection_volume_mm3": volume_mm3,
                "gate": "PASS" if volume_mm3 <= 1.0e-4 else "FAIL",
            }
        )

    # Measure the only potentially visible strip of the retained source head
    # post at its lateral edge.  This uses the released carrier mesh rather
    # than a hand-entered post height.  The new ellipsoidal shell must cover
    # that edge within 5 mm of the post base.
    body_mesh = (
        common.BASE_URDF.parent
        / "meshes"
        / "skeleton"
        / "Z_BOT2_MASTER_BODY_SKELETON.stl"
    )
    body_poly = _read_stl(body_mesh)
    body_points_m = np.array(
        [body_poly.GetPoint(index) for index in range(body_poly.GetNumberOfPoints())],
        dtype=float,
    )
    post_points = body_points_m[
        (body_points_m[:, 2] > 0.010)
        & (body_points_m[:, 1] > -0.007)
        & (body_points_m[:, 1] < 0.025)
    ]
    post_half_width_mm = float(np.max(np.abs(post_points[:, 0]))) * 1000.0
    edge_points = post_points[
        np.abs(np.abs(post_points[:, 0]) * 1000.0 - post_half_width_mm) < 0.25
    ]
    post_base_z_mm = float(np.min(edge_points[:, 2])) * 1000.0
    head_center_z_mm = 126.0 + common.HEAD_Z_SHIFT_MM
    shell_cover_z_mm = head_center_z_mm - 65.0 * math.sqrt(
        1.0 - (post_half_width_mm / 78.0) ** 2
    )
    exposed_post_height_mm = max(0.0, shell_cover_z_mm - post_base_z_mm)
    neck_visibility = {
        "source_post_half_width_mm": post_half_width_mm,
        "source_post_base_z_mm": post_base_z_mm,
        "head_shell_cover_z_at_post_edge_mm": shell_cover_z_mm,
        "maximum_exposed_post_height_mm": exposed_post_height_mm,
        "requirement_mm": 5.0,
        "gate": "PASS" if exposed_post_height_mm <= 5.0 else "FAIL",
    }

    geometry_overall = all(row["geometry_gate"] == "PASS" for row in geometry_rows)
    replacement_overall = all(row["gate"] == "PASS" for row in replacement_rows)
    interface_overall = all(row["gate"] == "PASS" for row in interface_rows)
    inertia_overall = all(row["inertia_gate"] == "PASS" for row in mass_rows)
    geometry_document = {
        "schema": "zeroth01.physical_mount_v2_minimal.geometry_gate.v1",
        "change_policy": "v1 base unchanged except head/chest/sole and complete claw removal",
        "head_z_shift_mm": common.HEAD_Z_SHIFT_MM,
        "part_rows": geometry_rows,
        "forearm_replacement_rows": replacement_rows,
        "static_part_interface_rows": interface_rows,
        "retained_head_post_visibility": neck_visibility,
        "overall": (
            "PASS"
            if geometry_overall
            and replacement_overall
            and interface_overall
            and neck_visibility["gate"] == "PASS"
            else "FAIL"
        ),
        "claim_boundary": (
            "Proves released STEP/STL topology, compact hand envelope, 9 mm sole thickness, "
            "watertight forearm claw trims, and non-overlapping head/chest seams. "
            "The retained head post is exposed by no more than 5 mm at its "
            "lateral edge. Dynamic inter-link clearance is separate."
        ),
    }
    GEOMETRY_REPORT.write_text(
        json.dumps(geometry_document, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with GEOMETRY_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(geometry_rows[0]))
        writer.writeheader()
        writer.writerows(geometry_rows)

    mass_document = {
        "schema": "zeroth01.physical_mount_v2_minimal.mass_properties.v1",
        "material_basis": "PETG 1270 kg/m3; sole prototype TPU 1200 kg/m3; CAD-solid nominal",
        "nominal_printed_mass_kg": sum(float(row["nominal_mass_kg"]) for row in mass_rows),
        "inertia_gate": "PASS" if inertia_overall else "FAIL",
        "parts": mass_parts,
        "forearm_retained_volume_ratio": {
            Path(str(row["replacement"])).stem: row["retained_volume_ratio"]
            for row in replacement_rows
        },
        "hardware_override_required": True,
    }
    MASS_REPORT.write_text(
        json.dumps(mass_document, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    with MASS_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(mass_rows[0]))
        writer.writeheader()
        writer.writerows(mass_rows)
    print(GEOMETRY_REPORT)
    print(MASS_REPORT)
    print(
        f"geometry={geometry_document['overall']} inertia={mass_document['inertia_gate']} "
        f"printed_mass_kg={mass_document['nominal_printed_mass_kg']:.6f}"
    )
    return 0 if geometry_document["overall"] == "PASS" and inertia_overall else 2


if __name__ == "__main__":
    raise SystemExit(main())
