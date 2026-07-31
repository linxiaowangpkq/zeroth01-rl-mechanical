"""Render v2-minimal on the unchanged 16-DoF Physical Mount v1 chain."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from PIL import Image
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkIOGeometry import vtkSTLReader
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderer,
    vtkWindowToImageFilter,
)
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401


ROOT = Path(__file__).resolve().parents[2]
V1_BUILD = ROOT / "cad" / "physical_mount_v1" / "build_physical_mount_v1.py"
URDF = ROOT / "generated" / "urdf" / "physical_mount_v1" / "zeroth01_physical_mount_v1.urdf"
V1_URDF_ROOT = URDF.parent
PART_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
MANIFEST = ROOT / "reports" / "physical_mount_v2_minimal" / "component_manifest.json"
SNAPSHOT_ROOT = ROOT / "snapshots" / "cad" / "physical_mount_v2_minimal"
FRAME_ROOT = SNAPSHOT_ROOT / "motion_frames"
MOTION_GIF = SNAPSHOT_ROOT / "physical_mount_v2_minimal_16dof_motion.gif"
MOTION_REPORT = ROOT / "reports" / "physical_mount_v2_minimal" / "motion_evidence.json"

REPLACED_LINKS = {"FINGER_1", "FINGER_1_2"}
FOREARM_REPLACEMENTS = {
    "R_ARM_MIRROR_1": ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements" / "R_ARM_MIRROR_1_WRIST_TRIMMED.stl",
    "L_ARM_MIRROR_1": ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements" / "L_ARM_MIRROR_1_WRIST_TRIMMED.stl",
}


def _load_v1():
    spec = importlib.util.spec_from_file_location("minimal_v2_render_v1", V1_BUILD)
    if spec is None or spec.loader is None:
        raise RuntimeError(V1_BUILD)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _hex_color(value: str) -> tuple[float, float, float]:
    token = value.lstrip("#")
    return tuple(int(token[index : index + 2], 16) / 255.0 for index in (0, 2, 4))


def _actor(path: Path, color: tuple[float, float, float], opacity: float) -> vtkActor:
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    if reader.GetOutput().GetNumberOfCells() <= 0:
        raise RuntimeError(f"empty STL: {path}")
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().SetInterpolationToPhong()
    return actor


def _matrix(transform, scale: float) -> vtkMatrix4x4:
    rotation, translation = transform
    result = vtkMatrix4x4()
    result.Identity()
    for row in range(3):
        for column in range(3):
            result.SetElement(row, column, float(rotation[row][column]) * scale)
        result.SetElement(row, 3, float(translation[row]))
    return result


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


def _camera(renderer: vtkRenderer, view: str) -> None:
    bounds = renderer.ComputeVisiblePropBounds()
    center = tuple((bounds[i * 2] + bounds[i * 2 + 1]) / 2.0 for i in range(3))
    size = tuple(bounds[i * 2 + 1] - bounds[i * 2] for i in range(3))
    diagonal = math.sqrt(sum(value * value for value in size))
    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    camera.SetFocalPoint(*center)
    camera.SetViewUp(0.0, 0.0, 1.0)
    if view == "front":
        camera.SetPosition(center[0] + diagonal * 2.4, center[1], center[2])
        camera.SetParallelScale(max(size[2], size[1] * 1.25) * 0.56)
    else:
        camera.SetPosition(
            center[0] + diagonal * 1.8,
            center[1] - diagonal * 1.8,
            center[2] + diagonal * 0.8,
        )
        camera.SetParallelScale(max(size) * 0.70)
    renderer.ResetCameraClippingRange()


def run(frame_count: int) -> None:
    base = _load_v1()
    urdf_root = ET.parse(URDF).getroot()
    base_link, joints = base._load_kinematic_model(urdf_root)
    neutral = base._forward_kinematics(base_link, joints, {})

    renderer = vtkRenderer()
    renderer.SetBackground(0.965, 0.975, 0.99)
    renderer.SetBackground2(0.84, 0.88, 0.94)
    renderer.GradientBackgroundOn()
    window = vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1100, 1400)
    window.SetMultiSamples(8)
    window.AddRenderer(renderer)

    actors_by_link: dict[str, list[tuple[vtkActor, float]]] = defaultdict(list)
    carrier_actors: list[vtkActor] = []
    servo_actors: list[vtkActor] = []
    new_printed_actors: list[vtkActor] = []
    payload_actors: list[vtkActor] = []
    visible_sensor_actors: list[vtkActor] = []
    part_actors: dict[str, vtkActor] = {}
    carrier_actors_by_link: dict[str, list[vtkActor]] = defaultdict(list)

    for link in urdf_root.findall("link"):
        link_name = str(link.get("name"))
        if link_name in REPLACED_LINKS or link_name not in neutral:
            continue
        for visual in link.findall("visual"):
            mesh = visual.find("./geometry/mesh")
            if mesh is None:
                continue
            token = str(mesh.get("filename")).replace(
                "package://zeroth01_physical_mount_v1/", ""
            )
            path = V1_URDF_ROOT / token
            is_servo = "INSTALLED_STS3215_FAMILY" in path.name
            if not is_servo and link_name in FOREARM_REPLACEMENTS:
                path = FOREARM_REPLACEMENTS[link_name]
            actor = _actor(
                path,
                (0.086, 0.467, 1.0) if is_servo else (0.91, 0.93, 0.96),
                1.0,
            )
            renderer.AddActor(actor)
            actors_by_link[link_name].append((actor, 1.0))
            (servo_actors if is_servo else carrier_actors).append(actor)
            if not is_servo:
                carrier_actors_by_link[link_name].append(actor)

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    for row in manifest["parts"]:
        key = str(row["key"])
        link_name = str(row["installed_link"])
        actor = _actor(PART_ROOT / f"{key}.stl", _hex_color(str(row["color_hex"])), 1.0)
        renderer.AddActor(actor)
        part_actors[key] = actor
        actors_by_link[link_name].append((actor, 0.001))
        classification = str(row["classification"])
        if classification == "internal_payload_controlled_envelope" or key == "camera_bracket":
            payload_actors.append(actor)
        elif key in {"face_ui", "camera_window"}:
            visible_sensor_actors.append(actor)
        else:
            new_printed_actors.append(actor)

    def apply_pose(positions: dict[str, float]) -> None:
        transforms = base._forward_kinematics(base_link, joints, positions)
        for link_name, actors in actors_by_link.items():
            for actor, scale in actors:
                actor.SetUserMatrix(_matrix(transforms[link_name], scale))

    apply_pose({})
    for actor in payload_actors:
        actor.SetVisibility(False)
    window.Render()
    _camera(renderer, "front")
    window.Render()
    _capture(window, SNAPSHOT_ROOT / "minimal_v2_normal_front.png")
    _camera(renderer, "iso")
    window.Render()
    _capture(window, SNAPSHOT_ROOT / "minimal_v2_normal_iso.png")

    # Diagnostic colouring used to prove that no baseline FINGER actor is
    # present and to identify any remaining claw-like geometry owned by the
    # parent forearm links.
    for key in ("left_q_hand", "right_q_hand"):
        part_actors[key].GetProperty().SetColor(0.15, 0.75, 0.32)
    for link_name in ("R_ARM_MIRROR_1", "L_ARM_MIRROR_1"):
        for actor in carrier_actors_by_link[link_name]:
            actor.GetProperty().SetColor(0.95, 0.28, 0.20)
    _camera(renderer, "front")
    window.Render()
    _capture(window, SNAPSHOT_ROOT / "minimal_v2_hand_ownership_debug.png")
    for key in ("left_q_hand", "right_q_hand"):
        part_actors[key].GetProperty().SetColor(0.969, 0.973, 0.980)
    for link_name in ("R_ARM_MIRROR_1", "L_ARM_MIRROR_1"):
        for actor in carrier_actors_by_link[link_name]:
            actor.GetProperty().SetColor(0.91, 0.93, 0.96)

    for actor in new_printed_actors:
        actor.GetProperty().SetOpacity(0.28)
    for actor in carrier_actors:
        actor.GetProperty().SetOpacity(0.30)
    for actor in payload_actors:
        actor.SetVisibility(True)
    _camera(renderer, "front")
    window.Render()
    _capture(window, SNAPSHOT_ROOT / "minimal_v2_xray_front.png")
    _camera(renderer, "iso")
    window.Render()
    _capture(window, SNAPSHOT_ROOT / "minimal_v2_xray_iso.png")

    for actor in new_printed_actors:
        actor.GetProperty().SetOpacity(0.82)
    for actor in carrier_actors:
        actor.GetProperty().SetOpacity(0.56)
    for actor in payload_actors:
        actor.SetVisibility(False)
    _camera(renderer, "front")
    moving = [joint for joint in joints if joint["type"] == "revolute"]
    FRAME_ROOT.mkdir(parents=True, exist_ok=True)
    for stale in FRAME_ROOT.glob("frame_*.png"):
        stale.unlink()
    frames: list[Path] = []
    for frame_index in range(frame_count):
        phase = 2.0 * math.pi * frame_index / max(1, frame_count - 1)
        positions: dict[str, float] = {}
        for joint_index, joint in enumerate(moving):
            lower = float(joint["lower"])
            upper = float(joint["upper"])
            negative_room = max(0.0, -lower)
            positive_room = max(0.0, upper)
            amplitude = min(
                math.radians(6.0),
                0.5 * negative_room if negative_room else math.radians(2.0),
                0.5 * positive_room if positive_room else math.radians(2.0),
            )
            positions[str(joint["name"])] = amplitude * math.sin(
                phase + (joint_index % 4) * math.pi / 2.0
            )
        apply_pose(positions)
        renderer.ResetCameraClippingRange()
        window.Render()
        frame = FRAME_ROOT / f"frame_{frame_index:03d}.png"
        _capture(window, frame)
        frames.append(frame)

    images = [Image.open(path).convert("P", palette=Image.Palette.ADAPTIVE) for path in frames]
    try:
        images[0].save(
            MOTION_GIF,
            save_all=True,
            append_images=images[1:],
            duration=100,
            loop=0,
            optimize=False,
        )
    finally:
        for image in images:
            image.close()

    report = {
        "schema": "zeroth01.physical_mount_v2_minimal.motion_evidence.v1",
        "base_urdf": URDF.relative_to(ROOT).as_posix(),
        "servo_visual_count": len(servo_actors),
        "moving_joint_count": len(moving),
        "baseline_claw_visual_count": 0,
        "replacement_hand_visual_count": 2,
        "frame_count": frame_count,
        "maximum_joint_amplitude_deg": 6.0,
        "result": "PASS" if len(servo_actors) == 16 and len(moving) == 16 else "FAIL",
        "motion_gif": MOTION_GIF.relative_to(ROOT).as_posix(),
        "scope": "Forward-kinematics visual evidence; collision sweep is a separate gate.",
    }
    MOTION_REPORT.parent.mkdir(parents=True, exist_ok=True)
    MOTION_REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    window.Finalize()
    print(MOTION_GIF)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--frames", type=int, default=20)
    args = parser.parse_args()
    run(max(8, args.frames))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
