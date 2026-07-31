"""Remove only the fixed claw finger integrated into each v1 forearm mesh.

The cut is made a small distance beyond the unchanged wrist joint origin, so
the elbow/forearm carrier and the circular wrist mounting interface remain.
The output is a capped, watertight STL suitable for the replacement URDF and
for import as a native SolidWorks mesh/surface part.
"""

from __future__ import annotations

import json
from pathlib import Path

from vtkmodules.vtkCommonDataModel import vtkPlane, vtkPlaneCollection
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkFiltersCore import (
    vtkCleanPolyData,
    vtkFeatureEdges,
    vtkPolyDataNormals,
    vtkReverseSense,
    vtkTriangleFilter,
)
from vtkmodules.vtkFiltersGeneral import vtkClipClosedSurface, vtkTransformPolyDataFilter
from vtkmodules.vtkFiltersModeling import vtkFillHolesFilter
from vtkmodules.vtkIOGeometry import vtkSTLReader, vtkSTLWriter


ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v1" / "meshes" / "skeleton"
OUTPUT_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements"
REPORT = ROOT / "reports" / "physical_mount_v2_minimal" / "forearm_claw_trim_gate.json"

# Source fixed-joint origins in each parent forearm frame.
LEFT_WRIST_AXIS_X_M = -0.066206
RIGHT_WRIST_AXIS_Y_M = 0.066206
CUT_ALLOWANCE_M = 0.0068


def _read(path: Path):
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    clean = vtkCleanPolyData()
    clean.SetInputConnection(reader.GetOutputPort())
    clean.Update()
    triangles = vtkTriangleFilter()
    triangles.SetInputConnection(clean.GetOutputPort())
    triangles.Update()
    normals = vtkPolyDataNormals()
    normals.SetInputConnection(triangles.GetOutputPort())
    normals.ConsistencyOn()
    normals.AutoOrientNormalsOn()
    normals.SplittingOff()
    normals.Update()
    return normals.GetOutput()


def _defect_edges(polydata) -> int:
    edges = vtkFeatureEdges()
    edges.SetInputData(polydata)
    edges.BoundaryEdgesOn()
    edges.NonManifoldEdgesOn()
    edges.FeatureEdgesOff()
    edges.ManifoldEdgesOff()
    edges.Update()
    return int(edges.GetOutput().GetNumberOfCells())


def _clip(source: Path, target: Path, origin, normal, expected_axis: int, expected_limit: float, keep_minimum: bool):
    plane = vtkPlane()
    plane.SetOrigin(*origin)
    plane.SetNormal(*normal)
    planes = vtkPlaneCollection()
    planes.AddItem(plane)
    clip = vtkClipClosedSurface()
    clip.SetInputData(_read(source))
    clip.SetClippingPlanes(planes)
    clip.GenerateFacesOn()
    clip.GenerateOutlineOff()
    clip.SetTolerance(1.0e-7)
    clip.Update()
    fill = vtkFillHolesFilter()
    fill.SetInputConnection(clip.GetOutputPort())
    fill.SetHoleSize(1.0)
    fill.Update()
    triangles = vtkTriangleFilter()
    triangles.SetInputConnection(fill.GetOutputPort())
    triangles.Update()
    clean = vtkCleanPolyData()
    clean.SetInputConnection(triangles.GetOutputPort())
    clean.SetTolerance(1.0e-6)
    clean.Update()
    result = clean.GetOutput()
    bounds = result.GetBounds()
    if result.GetNumberOfCells() <= 0:
        raise RuntimeError(f"clip produced no cells: {source}")
    observed = bounds[expected_axis * 2 if keep_minimum else expected_axis * 2 + 1]
    if abs(observed - expected_limit) > 2.0e-4:
        # vtkClipClosedSurface keeps the opposite half-space when the plane
        # normal is reversed.  Make this failure explicit rather than writing
        # a plausible-looking but wrong forearm.
        raise RuntimeError(
            f"wrong clip half-space for {source.name}: observed={observed} "
            f"expected={expected_limit} bounds={bounds}"
        )
    defects = _defect_edges(result)
    if defects != 0:
        raise RuntimeError(f"trimmed mesh is not watertight: {source.name} defects={defects}")
    target.parent.mkdir(parents=True, exist_ok=True)
    writer = vtkSTLWriter()
    writer.SetFileName(str(target))
    writer.SetInputData(result)
    writer.SetFileTypeToBinary()
    writer.Write()
    return {
        "source": source.relative_to(ROOT).as_posix(),
        "output": target.relative_to(ROOT).as_posix(),
        "source_bounds_m": list(_read(source).GetBounds()),
        "trimmed_bounds_m": list(bounds),
        "triangle_count": int(result.GetNumberOfCells()),
        "boundary_or_nonmanifold_edges": defects,
        "gate": "PASS",
    }


def _mirror_left_trim_to_right(left_target: Path, right_target: Path, source_right: Path):
    # Exact datum mapping between the two v1 parent-link frames, derived from
    # the fixed wrist origins:
    #   x_right = y_left, y_right = -x_left, z_right = -z_left.
    matrix = vtkMatrix4x4()
    matrix.Zero()
    matrix.SetElement(0, 1, 1.0)
    matrix.SetElement(1, 0, -1.0)
    matrix.SetElement(2, 2, -1.0)
    matrix.SetElement(3, 3, 1.0)
    transform = vtkTransform()
    transform.SetMatrix(matrix)
    apply = vtkTransformPolyDataFilter()
    apply.SetInputData(_read(left_target))
    apply.SetTransform(transform)
    apply.Update()
    reverse = vtkReverseSense()
    reverse.SetInputConnection(apply.GetOutputPort())
    reverse.ReverseCellsOn()
    reverse.ReverseNormalsOn()
    reverse.Update()
    clean = vtkCleanPolyData()
    clean.SetInputConnection(reverse.GetOutputPort())
    clean.SetTolerance(1.0e-7)
    clean.Update()
    result = clean.GetOutput()
    defects = _defect_edges(result)
    if defects != 0:
        raise RuntimeError(f"mirrored right forearm is not watertight: defects={defects}")
    bounds = result.GetBounds()
    if abs(bounds[3] - (RIGHT_WRIST_AXIS_Y_M + CUT_ALLOWANCE_M)) > 2.0e-4:
        raise RuntimeError(f"mirrored right cut datum mismatch: {bounds}")
    right_target.parent.mkdir(parents=True, exist_ok=True)
    writer = vtkSTLWriter()
    writer.SetFileName(str(right_target))
    writer.SetInputData(result)
    writer.SetFileTypeToBinary()
    writer.Write()
    source_poly = _read(source_right)
    return {
        "source": source_right.relative_to(ROOT).as_posix(),
        "output": right_target.relative_to(ROOT).as_posix(),
        "source_bounds_m": list(source_poly.GetBounds()),
        "trimmed_bounds_m": list(bounds),
        "triangle_count": int(result.GetNumberOfCells()),
        "boundary_or_nonmanifold_edges": defects,
        "source_topology_defects_repaired_by_exact_datum_mirror": _defect_edges(source_poly),
        "mirror_mapping": "x_right=y_left; y_right=-x_left; z_right=-z_left",
        "gate": "PASS",
    }


def main() -> int:
    left_cut = LEFT_WRIST_AXIS_X_M - CUT_ALLOWANCE_M
    right_cut = RIGHT_WRIST_AXIS_Y_M + CUT_ALLOWANCE_M
    left_target = OUTPUT_ROOT / "R_ARM_MIRROR_1_WRIST_TRIMMED.stl"
    right_target = OUTPUT_ROOT / "L_ARM_MIRROR_1_WRIST_TRIMMED.stl"
    rows = [
        _clip(
            SOURCE_ROOT / "R_ARM_MIRROR_1.stl",
            left_target,
            (left_cut, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            0,
            left_cut,
            True,
        ),
        _mirror_left_trim_to_right(
            left_target,
            right_target,
            SOURCE_ROOT / "L_ARM_MIRROR_1.stl",
        ),
    ]
    payload = {
        "schema": "zeroth01.physical_mount_v2_minimal.forearm_claw_trim_gate.v1",
        "method": "capped planar trim beyond unchanged fixed wrist joint origin",
        "wrist_axis_to_cut_allowance_mm": CUT_ALLOWANCE_M * 1000.0,
        "rows": rows,
        "overall": "PASS",
        "claim_boundary": (
            "Preserves the v1 forearm mesh up to the wrist service allowance and removes "
            "only the integrated fixed claw extension. Fastener strength remains a first-article gate."
        ),
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(REPORT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
