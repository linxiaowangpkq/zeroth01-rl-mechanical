from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from vtkmodules.vtkCommonCore import vtkLookupTable
from vtkmodules.vtkFiltersCore import (
    vtkCleanPolyData,
    vtkPolyDataConnectivityFilter,
)
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkCamera,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderer,
    vtkWindowToImageFilter,
)

# Register the OpenGL backend when VTK is installed as split vtkmodules.
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401


def _bounds_size(bounds: tuple[float, ...]) -> tuple[float, float, float]:
    return (
        bounds[1] - bounds[0],
        bounds[3] - bounds[2],
        bounds[5] - bounds[4],
    )


def _bounds_center(bounds: tuple[float, ...]) -> tuple[float, float, float]:
    return (
        (bounds[0] + bounds[1]) / 2.0,
        (bounds[2] + bounds[3]) / 2.0,
        (bounds[4] + bounds[5]) / 2.0,
    )


def _to_mm_scale(bounds: tuple[float, ...]) -> float:
    return 1000.0 if max(_bounds_size(bounds)) < 2.0 else 1.0


def _used_geometry_stats(polydata: object) -> tuple[
    tuple[float, ...], int, int
]:
    point_ids: set[int] = set()
    cell_count = int(polydata.GetNumberOfCells())
    for cell_id in range(cell_count):
        cell = polydata.GetCell(cell_id)
        ids = cell.GetPointIds()
        for index in range(ids.GetNumberOfIds()):
            point_ids.add(int(ids.GetId(index)))
    if not point_ids:
        return tuple(float(value) for value in polydata.GetBounds()), 0, 0
    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    for point_id in point_ids:
        x_pos, y_pos, z_pos = polydata.GetPoint(point_id)
        xs.append(float(x_pos))
        ys.append(float(y_pos))
        zs.append(float(z_pos))
    return (
        (
            min(xs),
            max(xs),
            min(ys),
            max(ys),
            min(zs),
            max(zs),
        ),
        len(point_ids),
        cell_count,
    )


def analyze_mesh(path: Path) -> tuple[dict[str, object], object]:
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()

    clean = vtkCleanPolyData()
    clean.SetInputConnection(reader.GetOutputPort())
    clean.Update()

    connectivity = vtkPolyDataConnectivityFilter()
    connectivity.SetInputConnection(clean.GetOutputPort())
    connectivity.SetExtractionModeToAllRegions()
    connectivity.ColorRegionsOn()
    connectivity.Update()

    polydata = connectivity.GetOutput()
    bounds = tuple(float(value) for value in polydata.GetBounds())
    scale = _to_mm_scale(bounds)
    region_count = connectivity.GetNumberOfExtractedRegions()
    regions: list[dict[str, object]] = []

    for region_id in range(region_count):
        extract = vtkPolyDataConnectivityFilter()
        extract.SetInputConnection(clean.GetOutputPort())
        extract.SetExtractionModeToSpecifiedRegions()
        extract.AddSpecifiedRegion(region_id)
        extract.Update()
        region = extract.GetOutput()
        region_bounds, used_points, used_cells = _used_geometry_stats(region)
        regions.append(
            {
                "region_id": region_id,
                "points": used_points,
                "triangles": used_cells,
                "bounds_mm": [round(value * scale, 6) for value in region_bounds],
                "size_mm": [
                    round(value * scale, 6)
                    for value in _bounds_size(region_bounds)
                ],
            }
        )

    regions.sort(
        key=lambda row: (
            int(row["triangles"]),
            int(row["points"]),
        ),
        reverse=True,
    )
    report = {
        "source": str(path.resolve()),
        "source_unit_inference": "metre" if scale == 1000.0 else "millimetre",
        "points": int(polydata.GetNumberOfPoints()),
        "triangles": int(polydata.GetNumberOfCells()),
        "connected_region_count": int(region_count),
        "bounds_mm": [round(value * scale, 6) for value in bounds],
        "size_mm": [
            round(value * scale, 6) for value in _bounds_size(bounds)
        ],
        "regions": regions,
    }
    return report, polydata


def render_components(
    polydata: object,
    output: Path,
    *,
    width: int = 1400,
    height: int = 1000,
) -> None:
    bounds = tuple(float(value) for value in polydata.GetBounds())
    center = _bounds_center(bounds)
    size = _bounds_size(bounds)
    diagonal = math.sqrt(sum(value * value for value in size))

    point_scalars = polydata.GetPointData().GetScalars()
    cell_scalars = polydata.GetCellData().GetScalars()
    scalars = point_scalars or cell_scalars
    scalar_range = scalars.GetRange() if scalars else (0.0, 1.0)
    lut = vtkLookupTable()
    lut.SetHueRange(0.0, 0.85)
    lut.SetSaturationRange(0.75, 0.95)
    lut.SetValueRange(0.75, 1.0)
    lut.SetNumberOfTableValues(max(2, int(scalar_range[1]) + 1))
    lut.Build()

    mapper = vtkPolyDataMapper()
    mapper.SetInputData(polydata)
    mapper.SetLookupTable(lut)
    mapper.SetScalarRange(scalar_range)
    if point_scalars:
        mapper.SetScalarModeToUsePointData()
    else:
        mapper.SetScalarModeToUseCellData()
    mapper.ScalarVisibilityOn()

    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetEdgeColor(0.12, 0.14, 0.18)
    actor.GetProperty().SetLineWidth(0.4)

    renderer = vtkRenderer()
    renderer.SetBackground(0.97, 0.98, 1.0)
    renderer.AddActor(actor)

    camera = vtkCamera()
    camera.SetFocalPoint(*center)
    camera.SetPosition(
        center[0] + diagonal * 1.25,
        center[1] - diagonal * 1.45,
        center[2] + diagonal * 1.05,
    )
    camera.SetViewUp(0.0, 0.0, 1.0)
    camera.ParallelProjectionOn()
    renderer.SetActiveCamera(camera)
    renderer.ResetCamera()
    camera.Zoom(1.1)

    window = vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(width, height)
    window.SetMultiSamples(4)
    window.AddRenderer(renderer)
    window.Render()

    capture = vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetScale(1)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    capture.Update()

    output.parent.mkdir(parents=True, exist_ok=True)
    writer = vtkPNGWriter()
    writer.SetFileName(str(output))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    window.Finalize()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Audit an upstream Zeroth STL by connected region and render "
            "an offline component-colour snapshot."
        )
    )
    parser.add_argument("mesh", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()

    report, polydata = analyze_mesh(args.mesh)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = args.mesh.stem
    report_path = args.output_dir / f"{stem}_mesh_audit.json"
    snapshot_path = args.output_dir / f"{stem}_components.png"
    report_path.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    render_components(polydata, snapshot_path)
    print(report_path)
    print(snapshot_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
