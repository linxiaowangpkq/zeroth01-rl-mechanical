"""Generate actual MuJoCo-kinematic motion evidence and a GIF.

Every frame is evaluated by MuJoCo from the shipped MJCF, vertically
re-grounded through the free base, and checked for non-ground penetration
before it is rendered.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "generated" / "mujoco" / "physical_mount_v3_rl_fixed" / "zeroth01_physical_mount_v3_rl_fixed_18dof_mjx.xml"
OUT = ROOT / "snapshots" / "motion" / "physical_mount_v3_rl_fixed"
GIF = OUT / "zeroth01_v3_18dof_mujoco_motion.gif"
REPORT = ROOT / "reports" / "physical_mount_v3_rl_fixed" / "coordinated_motion_evidence.json"


def font(size, bold=False):
    name = "arialbd.ttf" if bold else "arial.ttf"
    path = Path("C:/Windows/Fonts") / name
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def joint_qadr(model, name):
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    if joint_id < 0:
        raise KeyError(name)
    return int(model.jnt_qposadr[joint_id])


def set_pose(model, data, phase):
    s, c = math.sin(phase), math.cos(phase)
    values = {
        "left_shoulder_yaw": 0.055 * c,
        "right_shoulder_yaw": 0.055 * c,
        "left_shoulder_pitch": -0.26 * s,
        "right_shoulder_pitch": 0.26 * s,
        "left_elbow_yaw": 0.12 * c,
        "right_elbow_yaw": -0.12 * c,
        "left_hip_yaw": -0.01 + 0.02 * s,
        "right_hip_yaw": 0.01 - 0.02 * s,
        "left_hip_roll": 0.040 + 0.015 * c,
        "right_hip_roll": -0.040 - 0.015 * c,
        "left_hip_pitch": 0.17 + 0.12 * s,
        "right_hip_pitch": -0.17 - 0.12 * s,
        "left_knee_pitch": -0.20 - 0.16 * max(0.0, -s),
        "right_knee_pitch": 0.20 + 0.16 * max(0.0, s),
        "left_ankle_pitch": -0.12 + 0.08 * s,
        "right_ankle_pitch": 0.12 - 0.08 * s,
        "left_ankle_roll": 0.075 * c,
        "right_ankle_roll": -0.075 * c,
    }
    for name, value in values.items():
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        low, high = model.jnt_range[joint_id]
        if value < low - 1.0e-12 or value > high + 1.0e-12:
            raise ValueError(f"{name}={value} outside [{low}, {high}]")
        data.qpos[joint_qadr(model, name)] = value
    return values


def reground(model, data, foot_ids):
    mujoco.mj_forward(model, data)
    bottoms = []
    for geom_id in foot_ids:
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        half = model.geom_size[geom_id, :3]
        bottoms.append(float(data.geom_xpos[geom_id, 2] - np.abs(rotation[2, :]).dot(half)))
    correction = -min(bottoms)
    data.qpos[2] += correction
    mujoco.mj_forward(model, data)
    return float(correction)


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
    torso_center = point(root + np.array((0.0, 0.0, -0.025)))
    torso_box = (torso_center[0] - 95, torso_center[1] - 155, torso_center[0] + 95, torso_center[1] + 155)
    draw.rounded_rectangle(torso_box, 28, fill=(248, 249, 251), outline=(112, 128, 151), width=4)
    head_center = point(root + np.array((0.0, 0.0, 0.105)))
    draw.rounded_rectangle((head_center[0] - 120, head_center[1] - 92, head_center[0] + 120, head_center[1] + 92), 62, fill=(247, 248, 250), outline=(100, 116, 138), width=4)
    draw.rounded_rectangle((head_center[0] - 94, head_center[1] - 62, head_center[0] + 94, head_center[1] + 58), 28, fill=(12, 24, 36), outline=(55, 84, 110), width=3)
    for eye_x in (-42, 42):
        draw.ellipse((head_center[0] + eye_x - 12, head_center[1] - 16, head_center[0] + eye_x + 12, head_center[1] + 18), fill=(82, 214, 255))
    draw.ellipse((head_center[0] - 106, head_center[1] - 112, head_center[0] - 58, head_center[1] - 66), fill=(247, 248, 250), outline=(100, 116, 138), width=3)
    draw.ellipse((head_center[0] + 58, head_center[1] - 112, head_center[0] + 106, head_center[1] - 66), fill=(247, 248, 250), outline=(100, 116, 138), width=3)

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

    joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        for joint_id in range(model.njnt)
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE
    ]
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        body_id = int(model.jnt_bodyid[joint_id])
        px, py = point(data.xpos[body_id])
        draw.rounded_rectangle((px - 20, py - 15, px + 20, py + 15), 5, fill=(22, 119, 255), outline=(5, 58, 135), width=2)
        draw.ellipse((px - 5, py - 5, px + 5, py + 5), fill=(235, 248, 255))

    for foot_name in ("FOOT", "FOOT_2"):
        foot_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, foot_name)
        px, py = point(data.xpos[foot_id])
        draw.rounded_rectangle((px - 52, py - 12, px + 52, py + 13), 7, fill=(17, 24, 35), outline=(5, 8, 14), width=2)

    draw.rounded_rectangle((28, 25, width - 28, 90), 16, fill=(16, 24, 40))
    draw.text((50, 42), "MuJoCo coordinated 18DoF motion — actual shipped MJCF", font=font(25, True), fill=(248, 251, 255))
    draw.text((40, height - 55), f"frame {index + 1:02d}/{frame_count}   hip pitch L/R {values['left_hip_pitch']:+.3f}/{values['right_hip_pitch']:+.3f} rad", font=font(19), fill=(30, 44, 66))
    return image


def main() -> int:
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "official_standing")
    foot_ids = tuple(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) for name in ("FOOT_collision", "FOOT_2_collision"))
    frame_count = 64
    frames = []
    penetrations = []
    corrections = []
    extrema = {}
    for index in range(frame_count):
        phase = 2.0 * math.pi * index / frame_count
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        values = set_pose(model, data, phase)
        correction = reground(model, data, foot_ids)
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
        "schema": "zeroth01.physical_mount_v3_rl_fixed.coordinated_motion_evidence.v1",
        "source_mjcf": MJCF.relative_to(ROOT).as_posix(),
        "frame_count": frame_count,
        "fps": 12.5,
        "joint_extrema_rad": extrema,
        "maximum_vertical_regrounding_correction_m": max(corrections),
        "non_ground_penetrations": penetrations,
        "gate": "PASS" if not penetrations else "FAIL",
        "gif": GIF.relative_to(ROOT).as_posix(),
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"gate": payload["gate"], "frames": frame_count, "gif": str(GIF), "penetration_count": len(penetrations)}, indent=2))
    return 0 if not penetrations else 2


if __name__ == "__main__":
    raise SystemExit(main())
