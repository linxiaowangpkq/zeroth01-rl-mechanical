"""Characterize each connected component of the released Zeroth-01 torso STL.

The visual torso is an open multi-component triangle mesh.  Treating it as a
single body hides which cosmetic insert or chassis component owns each free
edge.  This diagnostic splits the mesh first, then measures raw and VTK-capped
topology without modifying the released source file.
"""

from __future__ import annotations

import json
from pathlib import Path

import vtk


ROOT = Path(__file__).resolve().parents[2]
SOURCE = (
    ROOT.parents[1]
    / "reference"
    / "zeroth01"
    / "generated"
    / "cad"
    / "physical_mount_v1"
    / "skeleton"
    / "Z_BOT2_MASTER_BODY_SKELETON.stl"
)
REPORT = (
    ROOT
    / "reports"
    / "v5_original_16dof_solidworks_motion"
    / "torso_mesh_connected_regions.json"
)


def topology(poly: vtk.vtkPolyData) -> dict[str, int]:
    edges: dict[tuple[int, int], int] = {}
    ids = vtk.vtkIdList()
    poly.GetPolys().InitTraversal()
    while poly.GetPolys().GetNextCell(ids):
        vertices = [int(ids.GetId(index)) for index in range(ids.GetNumberOfIds())]
        for index, first in enumerate(vertices):
            second = vertices[(index + 1) % len(vertices)]
            edge = tuple(sorted((first, second)))
            edges[edge] = edges.get(edge, 0) + 1
    return {
        "boundary_edges": sum(count == 1 for count in edges.values()),
        "nonmanifold_edges": sum(count > 2 for count in edges.values()),
    }


def clean(poly: vtk.vtkPolyData) -> vtk.vtkPolyData:
    cleaner = vtk.vtkCleanPolyData()
    cleaner.SetInputData(poly)
    cleaner.PointMergingOn()
    cleaner.Update()
    output = vtk.vtkPolyData()
    output.DeepCopy(cleaner.GetOutput())
    return output


def region(source: vtk.vtkPolyData, index: int) -> vtk.vtkPolyData:
    connectivity = vtk.vtkPolyDataConnectivityFilter()
    connectivity.SetInputData(source)
    connectivity.SetExtractionModeToSpecifiedRegions()
    connectivity.AddSpecifiedRegion(index)
    connectivity.Update()
    return clean(connectivity.GetOutput())


def main() -> None:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(SOURCE))
    reader.Update()
    source = clean(reader.GetOutput())

    connectivity = vtk.vtkPolyDataConnectivityFilter()
    connectivity.SetInputData(source)
    connectivity.SetExtractionModeToAllRegions()
    connectivity.Update()
    count = int(connectivity.GetNumberOfExtractedRegions())

    rows = []
    for index in range(count):
        raw = region(source, index)
        raw_topology = topology(raw)

        fill = vtk.vtkFillHolesFilter()
        fill.SetInputData(raw)
        fill.SetHoleSize(1.0e9)
        fill.Update()
        capped = clean(fill.GetOutput())
        capped_topology = topology(capped)

        mass = vtk.vtkMassProperties()
        mass.SetInputData(capped)
        mass.Update()
        bounds = capped.GetBounds()
        rows.append(
            {
                "region": index,
                "raw_points": int(raw.GetNumberOfPoints()),
                "raw_faces": int(raw.GetNumberOfPolys()),
                **raw_topology,
                "vtk_capped_points": int(capped.GetNumberOfPoints()),
                "vtk_capped_faces": int(capped.GetNumberOfPolys()),
                "vtk_capped_boundary_edges": capped_topology["boundary_edges"],
                "vtk_capped_nonmanifold_edges": capped_topology["nonmanifold_edges"],
                "vtk_capped_volume_m3": float(mass.GetVolume()),
                "bounds_m": [float(value) for value in bounds],
                "vtk_cap_closed": capped_topology == {
                    "boundary_edges": 0,
                    "nonmanifold_edges": 0,
                },
            }
        )

    rows.sort(key=lambda row: (-row["raw_faces"], row["region"]))
    payload = {
        "schema": "zeroth01.v5.torso_mesh_connected_regions.v1",
        "source": SOURCE.relative_to(ROOT.parents[1]).as_posix(),
        "source_points": int(source.GetNumberOfPoints()),
        "source_faces": int(source.GetNumberOfPolys()),
        "region_count": count,
        "raw_boundary_edges": topology(source)["boundary_edges"],
        "raw_nonmanifold_edges": topology(source)["nonmanifold_edges"],
        "vtk_individually_capped_closed_region_count": sum(
            bool(row["vtk_cap_closed"]) for row in rows
        ),
        "regions": rows,
        "overall": "PASS" if all(row["vtk_cap_closed"] for row in rows) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "report": REPORT.relative_to(ROOT).as_posix(),
                "region_count": count,
                "largest_regions": rows[:8],
                "vtk_closed": payload["vtk_individually_capped_closed_region_count"],
                "overall": payload["overall"],
            },
            indent=2,
        ),
        flush=True,
    )


if __name__ == "__main__":
    main()
