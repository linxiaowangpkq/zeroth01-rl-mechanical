"""Render coordinated 18DoF MuJoCo motion and collision-sweep evidence."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parents[2]
V3_RENDER = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "render_v3_motion.py"
MJCF = ROOT / "generated" / "mujoco" / "physical_mount_v4_original_minimal" / "zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml"
OUT = ROOT / "snapshots" / "motion" / "physical_mount_v4_original_minimal"
GIF = OUT / "zeroth01_v4_original_minimal_18dof_mujoco_motion.gif"
REPORT = ROOT / "reports" / "v4_original_minimal" / "coordinated_motion_evidence.json"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v3 = load(V3_RENDER, "zeroth01_v3_motion_helpers_for_v4")


def render_frame(model, data, index, frame_count, values):
    width, height = 900, 1120
    scale = 1500.0
    image = Image.new("RGB", (width, height), (239, 244, 250))
    draw = ImageDraw.Draw(image)
    for y in range(height):
        shade = int(247 - 18 * y / height)
        draw.line((0, y, width, y), fill=(shade, shade + 2, min(255, shade + 7)))

    def point(world):
        return (width / 2.0 - world[1] * scale, height - 92.0 - world[2] * scale)

    draw.line((45, height - 91, width - 45, height - 91), fill=(34, 41, 53), width=5)
    root_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "Z_BOT2_MASTER_BODY_SKELETON")
    root = data.xpos[root_id]
    root_rotation = data.xmat[root_id].reshape(3, 3)

    def body_local(local):
        return root + root_rotation.dot(np.asarray(local, dtype=float))

    torso_center = point(body_local((0.0, 0.0, -0.025)))
    draw.rounded_rectangle(
        (torso_center[0] - 78, torso_center[1] - 135, torso_center[0] + 78, torso_center[1] + 135),
        20,
        fill=(248, 249, 251),
        outline=(112, 128, 151),
        width=4,
    )

    # True v4 head location: released BODY-local frame, no neck, 90.75 mm
    # wide and 70.41 mm high after the hidden shoulder-servo clearance.
    head_center = point(body_local((0.0, 0.009189728, 0.03883662)))
    head_half_w = 0.090750004 * scale / 2.0
    head_half_h = 0.070409235 * scale / 2.0
    draw.rounded_rectangle(
        (head_center[0] - head_half_w, head_center[1] - head_half_h, head_center[0] + head_half_w, head_center[1] + head_half_h),
        18,
        fill=(250, 251, 252),
        outline=(93, 109, 132),
        width=4,
    )
    draw.rounded_rectangle(
        (head_center[0] - 54, head_center[1] - 39, head_center[0] + 54, head_center[1] + 39),
        13,
        fill=(12, 24, 36),
        outline=(55, 84, 110),
        width=2,
    )
    for eye_x in (-20, 20):
        draw.ellipse((head_center[0] + eye_x - 7, head_center[1] - 10, head_center[0] + eye_x + 7, head_center[1] + 10), fill=(82, 214, 255))
    draw.ellipse((head_center[0] - 26, head_center[1] - 31, head_center[0] - 17, head_center[1] - 22), fill=(0, 184, 217))
    draw.ellipse((head_center[0] + 20, head_center[1] - 29, head_center[0] + 25, head_center[1] - 24), fill=(0, 184, 217))

    physical = {
        "Z_BOT2_MASTER_SHOULDER2", "Z_BOT2_MASTER_SHOULDER2_2",
        "3215_1Flange", "3215_1Flange_2", "R_ARM_MIRROR_1", "L_ARM_MIRROR_1",
        "FINGER_1", "FINGER_1_2", "U_HIP_L", "U_HIP_R",
        "3215_BothFlange_5", "3215_BothFlange_6", "3215_BothFlange_9", "3215_BothFlange_10",
        "3215_BothFlange_13", "3215_BothFlange_14", "left_ankle_roll_carrier",
        "right_ankle_roll_carrier", "FOOT", "FOOT_2",
    }
    for body_id in range(1, model.nbody):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        if name not in physical:
            continue
        parent_id = int(model.body_parentid[body_id])
        parent_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, parent_id)
        if parent_name == "Z_BOT2_MASTER_BODY_SKELETON" or parent_name in physical:
            a, b = point(data.xpos[parent_id]), point(data.xpos[body_id])
            draw.line((a[0], a[1], b[0], b[1]), fill=(74, 87, 104), width=18)
            draw.line((a[0], a[1], b[0], b[1]), fill=(239, 242, 246), width=11)

    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        body_id = int(model.jnt_bodyid[joint_id])
        px, py = point(data.xpos[body_id])
        draw.rounded_rectangle((px - 20, py - 15, px + 20, py + 15), 5, fill=(22, 119, 255), outline=(5, 58, 135), width=2)
        draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=(235, 248, 255))

    for foot_name in ("FOOT", "FOOT_2"):
        foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, foot_name)
        px, py = point(data.xpos[foot_id])
        draw.rounded_rectangle((px - 52, py + 44, px + 52, py + 55), 5, fill=(17, 24, 35), outline=(5, 8, 14), width=2)

    draw.rounded_rectangle((28, 25, width - 28, 90), 16, fill=(16, 24, 40))
    draw.text((50, 42), "Zeroth-01 v4 · MuJoCo 18DoF collision sweep", font=v3.font(25, True), fill=(248, 251, 255))
    draw.text((40, height - 55), f"frame {index + 1:02d}/{frame_count}   hip pitch L/R {values['left_hip_pitch']:+.3f}/{values['right_hip_pitch']:+.3f} rad", font=v3.font(19), fill=(30, 44, 66))
    return image


def main() -> int:
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "official_standing")
    foot_ids = tuple(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in ("FOOT_collision", "FOOT_2_collision")
    )
    frame_count = 64
    frames = []
    penetrations = []
    corrections = []
    extrema = {}
    for index in range(frame_count):
        phase = 2.0 * math.pi * index / frame_count
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        values = v3.set_pose(model, data, phase)
        correction = v3.reground(model, data, foot_ids)
        corrections.append(abs(correction))
        for name, value in values.items():
            extrema.setdefault(name, [value, value])
            extrema[name][0] = min(extrema[name][0], value)
            extrema[name][1] = max(extrema[name][1], value)
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if contact.dist >= -0.0005:
                continue
            geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
            geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
            if "ground" in {geom1, geom2}:
                continue
            penetrations.append({"frame": index, "geom1": geom1, "geom2": geom2, "depth_m": float(-contact.dist)})
        frames.append(render_frame(model, data, index, frame_count, values))

    OUT.mkdir(parents=True, exist_ok=True)
    frames[0].save(OUT / "motion_frame_000.png")
    frames[frame_count // 4].save(OUT / "motion_frame_016.png")
    frames[0].save(GIF, save_all=True, append_images=frames[1:], duration=80, loop=0, optimize=False)
    payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.coordinated_motion_evidence.v1",
        "source_mjcf": MJCF.relative_to(ROOT).as_posix(),
        "frame_count": frame_count,
        "fps": 12.5,
        "joint_extrema_rad": extrema,
        "maximum_vertical_regrounding_correction_m": max(corrections),
        "non_ground_penetrations": penetrations,
        "gate": "PASS" if not penetrations else "FAIL",
        "gif": GIF.relative_to(ROOT).as_posix(),
        "truth_boundary": "Kinematic coordinated sweep with MuJoCo primitive collisions; not a powered balance or torque validation.",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"gate": payload["gate"], "frames": frame_count, "gif": str(GIF), "penetration_count": len(penetrations)}, indent=2))
    return 0 if not penetrations else 2


if __name__ == "__main__":
    raise SystemExit(main())
