"""Scan axis-aligned payload envelopes against the released body STL.

This diagnostic deliberately operates on the original triangles.  The legacy
STEP conversion contains thousands of unsewn faces, so a successful OCC
boolean is not evidence that a real service pocket was cut.
"""

from __future__ import annotations

import json
from itertools import product
from pathlib import Path

import vtk


ROOT = Path(__file__).resolve().parents[2]
BODY_STL = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v1"
    / "skeleton"
    / "Z_BOT2_MASTER_BODY_SKELETON.stl"
)
OUT = ROOT / "reports" / "v4_original_minimal" / "payload_triangle_screening.json"


def body_polydata():
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(BODY_STL))
    reader.Update()
    clean = vtk.vtkCleanPolyData()
    clean.SetInputConnection(reader.GetOutputPort())
    clean.Update()
    return clean.GetOutput()


def contacts(body, size_mm, center_mm) -> int:
    cube = vtk.vtkCubeSource()
    cube.SetXLength(size_mm[0])
    cube.SetYLength(size_mm[1])
    cube.SetZLength(size_mm[2])
    cube.SetCenter(*center_mm)
    cube.Update()
    transform_body = vtk.vtkTransform()
    transform_payload = vtk.vtkTransform()
    collision = vtk.vtkCollisionDetectionFilter()
    collision.SetInputData(0, body)
    collision.SetTransform(0, transform_body)
    collision.SetInputData(1, cube.GetOutput())
    collision.SetTransform(1, transform_payload)
    collision.SetCollisionModeToAllContacts()
    collision.SetBoxTolerance(0.0)
    collision.SetCellTolerance(0.0)
    collision.GenerateScalarsOff()
    collision.Update()
    return int(collision.GetNumberOfContacts())


def enclosed_samples(body, size_mm, center_mm) -> int:
    points = vtk.vtkPoints()
    for fx, fy, fz in product((-0.4, -0.2, 0.0, 0.2, 0.4), repeat=3):
        points.InsertNextPoint(
            center_mm[0] + fx * size_mm[0],
            center_mm[1] + fy * size_mm[1],
            center_mm[2] + fz * size_mm[2],
        )
    samples = vtk.vtkPolyData()
    samples.SetPoints(points)
    enclosed = vtk.vtkSelectEnclosedPoints()
    enclosed.SetInputData(samples)
    enclosed.SetSurfaceData(body)
    enclosed.SetTolerance(1.0e-5)
    enclosed.CheckSurfaceOff()
    enclosed.Update()
    selected = enclosed.GetOutput().GetPointData().GetArray("SelectedPoints")
    return sum(int(selected.GetTuple1(index)) for index in range(selected.GetNumberOfTuples()))


def scan(body, size_mm, xs, ys, zs):
    rows = []
    for center in product(xs, ys, zs):
        count = contacts(body, size_mm, center)
        inside = enclosed_samples(body, size_mm, center)
        rows.append({"center_mm": center, "surface_contact_count": count, "enclosed_sample_count": inside})
    return sorted(rows, key=lambda row: (row["surface_contact_count"] > 0 or row["enclosed_sample_count"] > 0, row["surface_contact_count"], row["enclosed_sample_count"], sum(abs(v) for v in row["center_mm"])))


def main() -> int:
    body = body_polydata()
    tests = {
        "compute_pi_zero_2w_class": {
            "size_mm": (14.0, 74.0, 36.0),
            "rows": scan(body, (14.0, 74.0, 36.0), range(-40, 41, 10), (0,), range(-35, 1, 5)),
        },
        "thin_2s_battery": {
            "size_mm": (26.0, 79.0, 38.0),
            "rows": scan(body, (26.0, 79.0, 38.0), range(-40, 41, 10), (0,), range(-90, -34, 10)),
        },
        "torso_imu": {
            "size_mm": (12.0, 36.0, 29.0),
            "rows": scan(body, (12.0, 36.0, 29.0), range(-40, 41, 10), range(-20, 21, 10), range(-60, 1, 10)),
        },
    }
    payload = {
        "schema": "zeroth01.v4.payload_triangle_screening.v1",
        "source": str(BODY_STL),
        "body_triangle_count": int(body.GetNumberOfCells()),
        "method": "vtkCollisionDetectionFilter surface contacts plus 5x5x5 vtkSelectEnclosedPoints samples against released STL",
        "tests": {
            name: {
                "size_mm": item["size_mm"],
                "zero_contact_count": sum(row["surface_contact_count"] == 0 and row["enclosed_sample_count"] == 0 for row in item["rows"]),
                "best_20": item["rows"][:20],
            }
            for name, item in tests.items()
        },
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
