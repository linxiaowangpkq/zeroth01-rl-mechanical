"""Render the two non-watertight released torso components in context."""

from __future__ import annotations

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
OUT = (
    ROOT
    / "reports"
    / "v5_original_16dof_solidworks_motion"
    / "snapshots"
    / "torso_problem_regions.png"
)
PROBLEM_REGIONS = (14, 18)


def actor(poly: vtk.vtkPolyData, color: tuple[float, float, float], opacity: float):
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    item = vtk.vtkActor()
    item.SetMapper(mapper)
    item.GetProperty().SetColor(*color)
    item.GetProperty().SetOpacity(opacity)
    return item


def main() -> None:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(SOURCE))
    reader.Update()
    source = reader.GetOutput()

    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.94, 0.96, 0.99)
    renderer.AddActor(actor(source, (0.78, 0.81, 0.86), 0.22))
    for index in PROBLEM_REGIONS:
        connectivity = vtk.vtkPolyDataConnectivityFilter()
        connectivity.SetInputData(source)
        connectivity.SetExtractionModeToSpecifiedRegions()
        connectivity.AddSpecifiedRegion(index)
        connectivity.Update()
        renderer.AddActor(actor(connectivity.GetOutput(), (0.92, 0.12, 0.06), 1.0))

    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(True)
    window.SetSize(1400, 1000)
    window.AddRenderer(renderer)
    renderer.ResetCamera()
    camera = renderer.GetActiveCamera()
    camera.Azimuth(28.0)
    camera.Elevation(23.0)
    camera.Zoom(1.35)
    renderer.ResetCameraClippingRange()
    window.Render()

    image = vtk.vtkWindowToImageFilter()
    image.SetInput(window)
    image.SetScale(1)
    image.ReadFrontBufferOff()
    image.Update()
    OUT.parent.mkdir(parents=True, exist_ok=True)
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(OUT))
    writer.SetInputConnection(image.GetOutputPort())
    writer.Write()
    print(OUT.relative_to(ROOT).as_posix(), flush=True)


if __name__ == "__main__":
    main()
