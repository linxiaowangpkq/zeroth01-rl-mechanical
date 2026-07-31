"""Render the two original Zeroth wrist/end-link meshes in their local frames.

This is a diagnostic for replacing the claw links.  The coloured axes and the
semi-transparent proximal keep-out make the unchanged fixed-joint datum
visible before a replacement hand is generated.
"""

from __future__ import annotations

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingAnnotation import vtkAxesActor
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderer,
    vtkWindowToImageFilter,
)
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401


ROOT = Path(__file__).resolve().parents[2]
MESH_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v1" / "meshes" / "skeleton"
OUT_ROOT = ROOT / "snapshots" / "cad" / "physical_mount_v2_minimal" / "wrist_source"
URDF = ROOT / "generated" / "urdf" / "physical_mount_v1" / "zeroth01_physical_mount_v1.urdf"
V1_BUILD = ROOT / "cad" / "physical_mount_v1" / "build_physical_mount_v1.py"


def _mesh_actor(
    path: Path,
    color: tuple[float, float, float] = (0.82, 0.85, 0.9),
    opacity: float = 0.92,
) -> vtkActor:
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetEdgeColor(0.15, 0.18, 0.23)
    actor.GetProperty().SetLineWidth(0.35)
    return actor


def _matrix(transform) -> vtkMatrix4x4:
    rotation, translation = transform
    result = vtkMatrix4x4()
    result.Identity()
    for row in range(3):
        for column in range(3):
            result.SetElement(row, column, float(rotation[row][column]))
        result.SetElement(row, 3, float(translation[row]))
    return result


def _load_v1():
    spec = importlib.util.spec_from_file_location("minimal_wrist_v1", V1_BUILD)
    if spec is None or spec.loader is None:
        raise RuntimeError(V1_BUILD)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _axes_actor(length: float = 0.05) -> vtkAxesActor:
    actor = vtkAxesActor()
    actor.SetTotalLength(length, length, length)
    actor.SetShaftTypeToCylinder()
    actor.SetCylinderRadius(0.018)
    actor.SetConeRadius(0.08)
    actor.AxisLabelsOff()
    return actor


def _capture(window: vtkRenderWindow, path: Path) -> None:
    capture = vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    capture.Update()
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = vtkPNGWriter()
    writer.SetFileName(str(path))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()


def render(link: str) -> None:
    renderer = vtkRenderer()
    renderer.SetBackground(0.97, 0.98, 0.995)
    window = vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(900, 900)
    window.SetMultiSamples(8)
    window.AddRenderer(renderer)
    renderer.AddActor(_mesh_actor(MESH_ROOT / f"{link}.stl"))
    renderer.AddActor(_axes_actor())
    renderer.ResetCamera()
    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    bounds = renderer.ComputeVisiblePropBounds()
    center = tuple((bounds[i * 2] + bounds[i * 2 + 1]) / 2.0 for i in range(3))
    scale = max(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4])
    views = {
        "iso": ((2.0, -2.0, 1.4), (0.0, 0.0, 1.0)),
        "x": ((3.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        "y": ((0.0, 3.0, 0.0), (0.0, 0.0, 1.0)),
        "z": ((0.0, 0.0, 3.0), (0.0, 1.0, 0.0)),
    }
    for name, (direction, up) in views.items():
        camera.SetFocalPoint(*center)
        camera.SetPosition(*(center[i] + direction[i] * scale for i in range(3)))
        camera.SetViewUp(*up)
        camera.SetParallelScale(scale * 0.62)
        renderer.ResetCameraClippingRange()
        window.Render()
        _capture(window, OUT_ROOT / f"{link}_{name}.png")
    window.Finalize()


def render_mated() -> None:
    base = _load_v1()
    root = ET.parse(URDF).getroot()
    base_link, joints = base._load_kinematic_model(root)
    transforms = base._forward_kinematics(base_link, joints, {})
    pairs = (
        ("left", "R_ARM_MIRROR_1", "FINGER_1"),
        ("right", "L_ARM_MIRROR_1", "FINGER_1_2"),
    )
    for side, parent, child in pairs:
        renderer = vtkRenderer()
        renderer.SetBackground(0.97, 0.98, 0.995)
        window = vtkRenderWindow()
        window.SetOffScreenRendering(1)
        window.SetSize(1000, 900)
        window.SetMultiSamples(8)
        window.AddRenderer(renderer)
        parent_actor = _mesh_actor(
            MESH_ROOT / f"{parent}.stl", (0.70, 0.74, 0.80), 0.38
        )
        child_actor = _mesh_actor(
            MESH_ROOT / f"{child}.stl", (0.93, 0.95, 0.98), 1.0
        )
        parent_actor.SetUserMatrix(_matrix(transforms[parent]))
        child_actor.SetUserMatrix(_matrix(transforms[child]))
        renderer.AddActor(parent_actor)
        renderer.AddActor(child_actor)
        bounds = child_actor.GetBounds()
        center = tuple((bounds[i * 2] + bounds[i * 2 + 1]) / 2.0 for i in range(3))
        span = max(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4])
        camera = renderer.GetActiveCamera()
        camera.ParallelProjectionOn()
        camera.SetFocalPoint(*center)
        camera.SetPosition(center[0] + span * 3.0, center[1], center[2])
        camera.SetViewUp(0.0, 0.0, 1.0)
        camera.SetParallelScale(span * 0.68)
        renderer.ResetCameraClippingRange()
        window.Render()
        _capture(window, OUT_ROOT / f"{side}_mated_front.png")
        camera.SetPosition(
            center[0] + span * 2.2,
            center[1] - span * 2.0,
            center[2] + span * 1.2,
        )
        camera.SetViewUp(0.0, 0.0, 1.0)
        renderer.ResetCameraClippingRange()
        window.Render()
        _capture(window, OUT_ROOT / f"{side}_mated_iso.png")
        window.Finalize()


def main() -> int:
    for link in ("FINGER_1", "FINGER_1_2"):
        render(link)
    render_mated()
    print(OUT_ROOT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
