"""Render deterministic off-screen QA views of the coloured right-arm GLB."""

from __future__ import annotations

import json
import math
from pathlib import Path

import vtk


ROOT = Path(__file__).resolve().parents[2]
ARM = ROOT / "manufacturing" / "v5_bambu_first_article_release" / "arm_first_article"
SOURCE = ARM / "RIGHT_ARM_NEUTRAL_ASSEMBLY_REFERENCE.glb"
OUT = ARM / "validation" / "snapshots"
REPORT = ARM / "validation" / "render_gate.json"


def camera_for_view(bounds: tuple[float, ...], view: str):
    center = (
        (bounds[0] + bounds[1]) / 2.0,
        (bounds[2] + bounds[3]) / 2.0,
        (bounds[4] + bounds[5]) / 2.0,
    )
    size = (
        bounds[1] - bounds[0],
        bounds[3] - bounds[2],
        bounds[5] - bounds[4],
    )
    distance = max(size) * 2.8
    if view == "front":
        direction = (1.0, 0.0, 0.0)
    elif view == "right":
        direction = (0.0, -1.0, 0.0)
    else:
        direction = (1.0, -1.0, 0.65)
    norm = math.sqrt(sum(value * value for value in direction))
    direction = tuple(value / norm for value in direction)
    position = tuple(center[index] + direction[index] * distance for index in range(3))
    return center, position


def render(view: str, target: Path) -> dict[str, object]:
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1600, 1200)
    renderer = vtk.vtkRenderer()
    renderer.SetBackground(0.96, 0.97, 0.985)
    window.AddRenderer(renderer)

    importer = vtk.vtkGLTFImporter()
    importer.SetFileName(str(SOURCE))
    importer.SetRenderWindow(window)
    importer.Update()

    renderer = window.GetRenderers().GetFirstRenderer()
    renderer.SetBackground(0.96, 0.97, 0.985)
    bounds = tuple(float(value) for value in renderer.ComputeVisiblePropBounds())
    center, position = camera_for_view(bounds, view)
    camera = renderer.GetActiveCamera()
    camera.SetFocalPoint(*center)
    camera.SetPosition(*position)
    camera.SetViewUp(0.0, 0.0, 1.0)
    camera.ParallelProjectionOn()
    renderer.ResetCamera()
    camera.Zoom(1.08)
    renderer.ResetCameraClippingRange()
    window.Render()

    capture = vtk.vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGBA()
    capture.ReadFrontBufferOff()
    capture.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(target))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()
    return {
        "view": view,
        "path": target.relative_to(ROOT).as_posix(),
        "bytes": target.stat().st_size,
        # glTF/VTK scene units are metres; publish dimensions explicitly in
        # millimetres while leaving camera coordinates in metres.
        "bounds_mm": [round(value * 1000.0, 6) for value in bounds],
        "camera_position_m": list(position),
        "camera_target_m": list(center),
    }


def main() -> int:
    if not SOURCE.is_file():
        raise FileNotFoundError(SOURCE)
    OUT.mkdir(parents=True, exist_ok=True)
    rows = [render(view, OUT / f"right_arm_{view}.png") for view in ("iso", "front", "right")]
    payload = {
        "schema": "zeroth01.v5.right_arm_first_article.render_gate.v1",
        "source": SOURCE.relative_to(ROOT).as_posix(),
        "views": rows,
        "overall": "PASS" if all(row["bytes"] > 10_000 for row in rows) else "FAIL",
        "truth_boundary": "off-screen visual QA of the neutral digital subset; not a physical fit or powered-motion certificate",
    }
    REPORT.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
