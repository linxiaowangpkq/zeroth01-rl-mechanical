"""Generate the v3 18DoF RL URDF from the released v2-minimal appearance.

The generator owns frames, axes, primitive collisions and nominal inertials.
The v2 URDF is read only as visual/CAD provenance.  Generated XML is not a
source of truth and must not be hand-edited.
"""

from __future__ import annotations

import copy
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
V2_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v2_minimal"
V2_URDF = V2_ROOT / "zeroth01_physical_mount_v2_minimal.urdf"
OUT_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v3_rl_fixed"
OUT_URDF = OUT_ROOT / "zeroth01_physical_mount_v3_rl_fixed_18dof.urdf"
ASSEMBLY_MANIFEST = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v3_rl_fixed"
    / "ZEROTH01_V3_RL_FIXED_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)

BODY = "Z_BOT2_MASTER_BODY_SKELETON"
LEFT_ANKLE_CARRIER = "left_ankle_roll_carrier"
RIGHT_ANKLE_CARRIER = "right_ankle_roll_carrier"
TARGET_TOTAL_MASS_KG = 3.095471828
STS3250_MASS_KG = 0.0745
CONTINUOUS_EFFORT_NM = 1.2552512
RATED_EFFORT_NM = 1.569064
MAX_VELOCITY_RAD_S = 3.0
ANKLE_ROLL_OFFSET_M = 0.050

# Replaced by the purchased StackChan head, and by the exposed load-bearing
# torso. These were visual-only in the v3 ledger: excluding them neither hides
# mass nor moves their nominal weight into another component.
REMOVED_BODY_VISUALS = {
    "minimal_chest_panel_visual",
    "minimal_head_front_visual",
    "minimal_head_back_visual",
    "minimal_visor_visual",
    "minimal_face_ui_visual",
    "minimal_camera_window_visual",
    "minimal_camera_bracket_visual",
}
TARGET_HIP_HALF_SPACING_M = 0.037
STACKCHAN_HEAD_POD = "m5stack_stackchan_k151_head_pod"
STACKCHAN_HEAD_ADAPTER = "stackchan_k151_torso_adapter"
STACKCHAN_HEAD_POD_MASS_KG = 0.187
STACKCHAN_HEAD_ADAPTER_MASS_KG = 0.018
# The 3 mm aluminium adapter sits directly on the torso top; the purchased
# StackChan base sits directly on the adapter, so the visible neck gap is 0 mm.
STACKCHAN_ADAPTER_CENTER_M = (0.015, 0.0, 0.0665)
STACKCHAN_HEAD_CENTER_M = (0.015, 0.0, 0.10325)
STACKCHAN_ADAPTER_SIZE_M = (0.066, 0.060, 0.003)
STACKCHAN_HEAD_SIZE_M = (0.0615, 0.054, 0.0705)

I3 = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def vec(text: str | None) -> tuple[float, float, float]:
    values = tuple(float(value) for value in (text or "0 0 0").split())
    if len(values) != 3:
        raise ValueError(text)
    return values


def mat_mul(a, b):
    return tuple(
        tuple(sum(a[r][k] * b[k][c] for k in range(3)) for c in range(3))
        for r in range(3)
    )


def mat_t(a):
    return tuple(tuple(a[c][r] for c in range(3)) for r in range(3))


def mat_vec(a, b):
    return tuple(sum(a[r][c] * b[c] for c in range(3)) for r in range(3))


def add(a, b):
    return tuple(a[i] + b[i] for i in range(3))


def sub(a, b):
    return tuple(a[i] - b[i] for i in range(3))


def rpy_matrix(values):
    roll, pitch, yaw = values
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return mat_mul(rz, mat_mul(ry, rx))


def matrix_rpy(r):
    pitch = math.asin(max(-1.0, min(1.0, -r[2][0])))
    if abs(math.cos(pitch)) > 1.0e-8:
        roll = math.atan2(r[2][1], r[2][2])
        yaw = math.atan2(r[1][0], r[0][0])
    else:
        roll = 0.0
        yaw = math.atan2(-r[0][1], r[1][1])
    return roll, pitch, yaw


def tf_mul(a, b):
    ar, at = a
    br, bt = b
    return mat_mul(ar, br), add(mat_vec(ar, bt), at)


def tf_inv(a):
    r, t = a
    rt = mat_t(r)
    return rt, tuple(-value for value in mat_vec(rt, t))


def old_fk(robot: ET.Element):
    links = {str(link.get("name")) for link in robot.findall("link")}
    children = {
        str(joint.find("child").get("link")) for joint in robot.findall("joint")
    }
    roots = sorted(links - children)
    if len(roots) != 1:
        raise ValueError(roots)
    transforms = {roots[0]: (I3, (0.0, 0.0, 0.0))}
    pending = list(robot.findall("joint"))
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = str(joint.find("parent").get("link"))
            if parent not in transforms:
                continue
            origin = joint.find("origin")
            local = (
                rpy_matrix(vec(origin.get("rpy") if origin is not None else None)),
                vec(origin.get("xyz") if origin is not None else None),
            )
            child = str(joint.find("child").get("link"))
            transforms[child] = tf_mul(transforms[parent], local)
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError("unresolved v2 tree")
    return transforms


def neutral_transforms(old_tf):
    """Return the CAD-matching neutral link frames for the v3 tree.

    All released v2 frames are kept verbatim.  Each ankle-pitch child frame is
    renamed to an intermediate carrier, and the original foot frame is moved
    50 mm down in the body/world Z direction without changing orientation.
    """

    result = dict(old_tf)
    for carrier, foot in (
        (LEFT_ANKLE_CARRIER, "FOOT"),
        (RIGHT_ANKLE_CARRIER, "FOOT_2"),
    ):
        rotation, position = old_tf[foot]
        result[carrier] = (rotation, position)
        result[foot] = (rotation, add(position, (0.0, 0.0, -ANKLE_ROLL_OFFSET_M)))
    return result


def relative_transform(parent_tf, child_tf):
    return tf_mul(tf_inv(parent_tf), child_tf)


def joint_kinematics(old_robot: ET.Element, neutral_tf):
    """Return exact parent/child transforms and joint-frame axes for v3."""

    old_joints = {
        str(joint.get("name")): joint for joint in old_robot.findall("joint")
    }
    limits_by_name = {name: limits for name, _, _, _, limits in JOINT_SPECS}
    result = []
    for name, parent, child, _, _ in JOINT_SPECS:
        if name in {"left_ankle_roll", "right_ankle_roll"}:
            desired_world_axis = (
                (1.0, 0.0, 0.0)
                if name == "left_ankle_roll"
                else (-1.0, 0.0, 0.0)
            )
            origin = relative_transform(neutral_tf[parent], neutral_tf[child])
            axis = mat_vec(mat_t(neutral_tf[parent][0]), desired_world_axis)
            limits = limits_by_name[name]
        else:
            source = old_joints[name]
            origin_element = source.find("origin")
            origin = (
                rpy_matrix(
                    vec(
                        origin_element.get("rpy")
                        if origin_element is not None
                        else None
                    )
                ),
                vec(
                    origin_element.get("xyz")
                    if origin_element is not None
                    else None
                ),
            )
            axis_element = source.find("axis")
            axis = vec(axis_element.get("xyz") if axis_element is not None else None)
            limit_element = source.find("limit")
            limits = (
                float(limit_element.get("lower")),
                float(limit_element.get("upper")),
            )
        result.append((name, parent, child, origin, axis, limits))
    return result


PAIR_LINKS = (
    ("Z_BOT2_MASTER_SHOULDER2", "Z_BOT2_MASTER_SHOULDER2_2"),
    ("3215_1Flange", "3215_1Flange_2"),
    ("R_ARM_MIRROR_1", "L_ARM_MIRROR_1"),
    ("FINGER_1", "FINGER_1_2"),
    ("U_HIP_L", "U_HIP_R"),
    ("3215_BothFlange_5", "3215_BothFlange_6"),
    ("3215_BothFlange_9", "3215_BothFlange_10"),
    ("3215_BothFlange_13", "3215_BothFlange_14"),
    ("FOOT", "FOOT_2"),
)

LEFT_LEG_LINKS = {
    "U_HIP_L",
    "3215_BothFlange_5",
    "3215_BothFlange_9",
    "3215_BothFlange_13",
    "FOOT",
}
RIGHT_LEG_LINKS = {
    "U_HIP_R",
    "3215_BothFlange_6",
    "3215_BothFlange_10",
    "3215_BothFlange_14",
    "FOOT_2",
}


def sym_positions(old):
    positions = {name: transform[1] for name, transform in old.items()}
    for left, right in PAIR_LINKS:
        lp, rp = positions[left], positions[right]
        # The released Zeroth source is X-forward, Y-left, Z-up.  The v2
        # assembly mirror plane is therefore Y=0 (not X=0).  Preserving this
        # convention is essential: reflecting X collapsed both shoulders and
        # both legs onto the centreline even though the cosmetic meshes still
        # looked separated.
        x = (lp[0] + rp[0]) / 2.0
        y = (abs(lp[1]) + abs(rp[1])) / 2.0
        z = (lp[2] + rp[2]) / 2.0
        positions[left] = (x, y, z)
        positions[right] = (x, -y, z)
    # The v2 source hip shafts average 42.838 mm from the centreline.  A
    # reversible 5.838 mm slotted adapter moves the complete leg chains
    # inboard so the 3.095 kg single-support moment remains below the
    # conservative STS3250 continuous torque rather than merely the rated
    # torque.  All downstream leg frames move as a rigid neutral-pose group.
    source_half_spacing = positions["U_HIP_L"][1]
    delta = TARGET_HIP_HALF_SPACING_M - source_half_spacing
    for name in LEFT_LEG_LINKS:
        x, y, z = positions[name]
        positions[name] = (x, y + delta, z)
    for name in RIGHT_LEG_LINKS:
        x, y, z = positions[name]
        positions[name] = (x, y - delta, z)
    return positions


def fmt(values):
    return " ".join(f"{float(value):.12g}" for value in values)


def set_origin(element: ET.Element, transform) -> None:
    r, t = transform
    origin = element.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        element.insert(0, origin)
    origin.set("xyz", fmt(t))
    origin.set("rpy", fmt(matrix_rpy(r)))


def add_material(visual: ET.Element, name: str, rgba: str) -> None:
    material = ET.SubElement(visual, "material", name=name)
    ET.SubElement(material, "color", rgba=rgba)


COLLISION = {
    # Central torso proxy follows the structural spine and deliberately does
    # not fill the v2 shoulder cut-outs with one oversized convex box.
    BODY: ((0.080, 0.105, 0.180), (0.0, 0.0, -0.025)),
    "Z_BOT2_MASTER_SHOULDER2": ((0.050, 0.045, 0.045), (0.0, 0.0, 0.0)),
    "Z_BOT2_MASTER_SHOULDER2_2": ((0.050, 0.045, 0.045), (0.0, 0.0, 0.0)),
    "3215_1Flange": ((0.045, 0.045, 0.105), (0.0, 0.0, -0.050)),
    "3215_1Flange_2": ((0.045, 0.045, 0.105), (0.0, 0.0, -0.050)),
    "R_ARM_MIRROR_1": ((0.040, 0.040, 0.100), (0.0, 0.0, -0.050)),
    "L_ARM_MIRROR_1": ((0.040, 0.040, 0.100), (0.0, 0.0, -0.050)),
    "FINGER_1": ((0.042, 0.042, 0.040), (0.0, 0.0, -0.018)),
    "FINGER_1_2": ((0.042, 0.042, 0.040), (0.0, 0.0, -0.018)),
    "U_HIP_L": ((0.050, 0.050, 0.045), (0.0, 0.0, 0.0)),
    "U_HIP_R": ((0.050, 0.050, 0.045), (0.0, 0.0, 0.0)),
    "3215_BothFlange_5": ((0.050, 0.050, 0.050), (0.0, 0.0, 0.0)),
    "3215_BothFlange_6": ((0.050, 0.050, 0.050), (0.0, 0.0, 0.0)),
    "3215_BothFlange_9": ((0.050, 0.050, 0.105), (0.0, 0.0, -0.050)),
    "3215_BothFlange_10": ((0.050, 0.050, 0.105), (0.0, 0.0, -0.050)),
    "3215_BothFlange_13": ((0.047, 0.047, 0.105), (0.0, 0.0, -0.050)),
    "3215_BothFlange_14": ((0.047, 0.047, 0.105), (0.0, 0.0, -0.050)),
    LEFT_ANKLE_CARRIER: ((0.052, 0.0365, 0.02472), (0.0, 0.0, 0.0)),
    RIGHT_ANKLE_CARRIER: ((0.052, 0.0365, 0.02472), (0.0, 0.0, 0.0)),
    "FOOT": ((0.110, 0.075, 0.009), (-0.010, 0.0, -0.0225)),
    "FOOT_2": ((0.110, 0.075, 0.009), (-0.010, 0.0, -0.0225)),
}


FIXED_MASSES = {
    # Aggregate link masses include the STS3250 body carried by that rigid
    # link.  Parent-side ownership follows the source mounting audit; this
    # prevents the earlier impossible case where a servo-owning link weighed
    # less than one 74.5 g actuator.
    "Z_BOT2_MASTER_SHOULDER2": 0.020,
    "Z_BOT2_MASTER_SHOULDER2_2": 0.020,
    "3215_1Flange": 0.149,
    "3215_1Flange_2": 0.149,
    "R_ARM_MIRROR_1": 0.030,
    "L_ARM_MIRROR_1": 0.030,
    "FINGER_1": 0.025,
    "FINGER_1_2": 0.025,
    "U_HIP_L": 0.020,
    "U_HIP_R": 0.020,
    "3215_BothFlange_5": 0.0995,
    "3215_BothFlange_6": 0.0995,
    "3215_BothFlange_9": 0.1745,
    "3215_BothFlange_10": 0.1745,
    "3215_BothFlange_13": 0.229,
    "3215_BothFlange_14": 0.229,
    LEFT_ANKLE_CARRIER: 0.0995,
    RIGHT_ANKLE_CARRIER: 0.0995,
    "FOOT": 0.070,
    "FOOT_2": 0.070,
    STACKCHAN_HEAD_POD: STACKCHAN_HEAD_POD_MASS_KG,
    STACKCHAN_HEAD_ADAPTER: STACKCHAN_HEAD_ADAPTER_MASS_KG,
    "torso_imu_module": 0.010,
    "compute_module": 0.100,
    "battery_pack": 0.200,
    "IMU_2": 0.004,
}


def body_mass() -> float:
    return TARGET_TOTAL_MASS_KG - sum(FIXED_MASSES.values())


def add_inertial(link: ET.Element, mass: float, size, center) -> None:
    sx, sy, sz = size
    ixx = mass * (sy * sy + sz * sz) / 12.0
    iyy = mass * (sx * sx + sz * sz) / 12.0
    izz = mass * (sx * sx + sy * sy) / 12.0
    inertial = ET.SubElement(link, "inertial")
    ET.SubElement(inertial, "origin", xyz=fmt(center), rpy="0 0 0")
    ET.SubElement(inertial, "mass", value=f"{mass:.12g}")
    ET.SubElement(
        inertial,
        "inertia",
        ixx=f"{ixx:.12g}",
        iyy=f"{iyy:.12g}",
        izz=f"{izz:.12g}",
        ixy="0",
        ixz="0",
        iyz="0",
    )


def add_box_visual_collision(
    link: ET.Element,
    name: str,
    size,
    rgba: str,
    *,
    collision: bool = True,
) -> None:
    visual = ET.SubElement(link, "visual", name=f"{name}_visual")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", size=fmt(size))
    add_material(visual, f"{name}_material", rgba)
    if collision:
        element = ET.SubElement(link, "collision", name=f"{name}_collision")
        ET.SubElement(element, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(element, "geometry")
        ET.SubElement(geometry, "box", size=fmt(size))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _unit(values):
    length = math.sqrt(sum(value * value for value in values))
    if length <= 1.0e-12:
        raise ValueError(values)
    return tuple(value / length for value in values)


def _z_to_vector_rpy(delta):
    z_axis = _unit(delta)
    helper = (1.0, 0.0, 0.0) if abs(z_axis[0]) < 0.9 else (0.0, 1.0, 0.0)
    x_axis = _unit(_cross(helper, z_axis))
    y_axis = _cross(z_axis, x_axis)
    rotation = tuple(
        (x_axis[row], y_axis[row], z_axis[row]) for row in range(3)
    )
    return matrix_rpy(rotation)


def collision_inertia_spec(link_name: str, neutral_tf):
    if link_name in {BODY, "FOOT", "FOOT_2"}:
        # The released body and foot frames are rotated relative to base XYZ.
        # Express each desired world-aligned proxy in its real link frame.
        world_size = COLLISION[link_name][0]
        world_center = COLLISION[link_name][1]
        rotation_t = mat_t(neutral_tf[link_name][0])
        local_size = tuple(
            sum(abs(rotation_t[row][column]) * world_size[column] for column in range(3))
            for row in range(3)
        )
        return local_size, mat_vec(rotation_t, world_center)
    radius = LINK_RADIUS_M.get(link_name, 0.014)
    child = SEGMENT_CHILD.get(link_name)
    if child is None:
        size = (2.0 * radius,) * 3
        return size, (0.0, 0.0, 0.0)
    delta = relative_transform(neutral_tf[link_name], neutral_tf[child])[1]
    length = math.sqrt(sum(value * value for value in delta))
    if length <= 0.004:
        size = (2.0 * radius,) * 3
        return size, (0.0, 0.0, 0.0)
    size = tuple(abs(value) + 2.0 * radius for value in delta)
    return size, tuple(value / 2.0 for value in delta)


def primitive_collision(link: ET.Element, link_name: str, neutral_tf) -> None:
    if link_name in {BODY, "FOOT", "FOOT_2"}:
        size, center = collision_inertia_spec(link_name, neutral_tf)
        collision = ET.SubElement(link, "collision", name=f"{link_name}_primitive_collision")
        ET.SubElement(collision, "origin", xyz=fmt(center), rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "box", size=fmt(size))
        return

    radius = LINK_RADIUS_M.get(link_name, 0.014)
    child = SEGMENT_CHILD.get(link_name)
    delta = (
        relative_transform(neutral_tf[link_name], neutral_tf[child])[1]
        if child
        else (0.0, 0.0, 0.0)
    )
    length = math.sqrt(sum(value * value for value in delta))
    collision = ET.SubElement(link, "collision", name=f"{link_name}_primitive_collision")
    geometry = ET.SubElement(collision, "geometry")
    if length <= 0.004:
        ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
        ET.SubElement(geometry, "sphere", radius=f"{radius:.12g}")
    else:
        ET.SubElement(
            collision,
            "origin",
            xyz=fmt(tuple(value / 2.0 for value in delta)),
            rpy=fmt(_z_to_vector_rpy(delta)),
        )
        ET.SubElement(geometry, "cylinder", radius=f"{radius:.12g}", length=f"{length:.12g}")


JOINT_SPECS = (
    # Clean world-aligned neutral frames: X forward/roll, Y left/pitch,
    # Z up/yaw.  Opposite bilateral axis signs make one symmetric command
    # produce mirrored motion while retaining identical numeric limits.
    ("left_shoulder_yaw", BODY, "Z_BOT2_MASTER_SHOULDER2", (0.0, 0.0, 1.0), (-0.12, 0.12)),
    ("right_shoulder_yaw", BODY, "Z_BOT2_MASTER_SHOULDER2_2", (0.0, 0.0, -1.0), (-0.12, 0.12)),
    ("left_shoulder_pitch", "Z_BOT2_MASTER_SHOULDER2", "3215_1Flange", (0.0, 1.0, 0.0), (-0.47123889, 0.47123889)),
    ("right_shoulder_pitch", "Z_BOT2_MASTER_SHOULDER2_2", "3215_1Flange_2", (0.0, 1.0, 0.0), (-0.47123889, 0.47123889)),
    ("left_elbow_yaw", "3215_1Flange", "R_ARM_MIRROR_1", (1.0, 0.0, 0.0), (-0.24, 0.24)),
    ("right_elbow_yaw", "3215_1Flange_2", "L_ARM_MIRROR_1", (-1.0, 0.0, 0.0), (-0.24, 0.24)),
    ("left_hip_yaw", BODY, "U_HIP_L", (0.0, 0.0, 1.0), (-0.12, 0.12)),
    ("right_hip_yaw", BODY, "U_HIP_R", (0.0, 0.0, -1.0), (-0.12, 0.12)),
    ("left_hip_roll", "U_HIP_L", "3215_BothFlange_5", (1.0, 0.0, 0.0), (-0.02, 0.14)),
    ("right_hip_roll", "U_HIP_R", "3215_BothFlange_6", (-1.0, 0.0, 0.0), (-0.02, 0.14)),
    ("left_hip_pitch", "3215_BothFlange_5", "3215_BothFlange_9", (0.0, 1.0, 0.0), (-0.22, 0.22)),
    ("right_hip_pitch", "3215_BothFlange_6", "3215_BothFlange_10", (0.0, 1.0, 0.0), (-0.22, 0.22)),
    ("left_knee_pitch", "3215_BothFlange_9", "3215_BothFlange_13", (0.0, 1.0, 0.0), (0.0, 0.83285928)),
    ("right_knee_pitch", "3215_BothFlange_10", "3215_BothFlange_14", (0.0, 1.0, 0.0), (0.0, 0.83285928)),
    ("left_ankle_pitch", "3215_BothFlange_13", LEFT_ANKLE_CARRIER, (0.0, 1.0, 0.0), (-0.33207964, 0.33207964)),
    ("right_ankle_pitch", "3215_BothFlange_14", RIGHT_ANKLE_CARRIER, (0.0, 1.0, 0.0), (-0.33207964, 0.33207964)),
    ("left_ankle_roll", LEFT_ANKLE_CARRIER, "FOOT", (1.0, 0.0, 0.0), (-0.25, 0.25)),
    ("right_ankle_roll", RIGHT_ANKLE_CARRIER, "FOOT_2", (-1.0, 0.0, 0.0), (-0.25, 0.25)),
)

FIXED_LINKS = (
    ("left_hand_fixed_joint", "R_ARM_MIRROR_1", "FINGER_1"),
    ("right_hand_fixed_joint", "L_ARM_MIRROR_1", "FINGER_1_2"),
)

# The collision shape for a load-bearing member follows the actual vector to
# its downstream joint, instead of assuming every member points along local
# -Z.  This is shared by URDF inertial approximations and the MJX generator.
SEGMENT_CHILD = {
    "Z_BOT2_MASTER_SHOULDER2": "3215_1Flange",
    "Z_BOT2_MASTER_SHOULDER2_2": "3215_1Flange_2",
    "3215_1Flange": "R_ARM_MIRROR_1",
    "3215_1Flange_2": "L_ARM_MIRROR_1",
    "R_ARM_MIRROR_1": "FINGER_1",
    "L_ARM_MIRROR_1": "FINGER_1_2",
    "U_HIP_L": "3215_BothFlange_5",
    "U_HIP_R": "3215_BothFlange_6",
    "3215_BothFlange_5": "3215_BothFlange_9",
    "3215_BothFlange_6": "3215_BothFlange_10",
    "3215_BothFlange_9": "3215_BothFlange_13",
    "3215_BothFlange_10": "3215_BothFlange_14",
    "3215_BothFlange_13": LEFT_ANKLE_CARRIER,
    "3215_BothFlange_14": RIGHT_ANKLE_CARRIER,
    LEFT_ANKLE_CARRIER: "FOOT",
    RIGHT_ANKLE_CARRIER: "FOOT_2",
}

LINK_RADIUS_M = {
    "Z_BOT2_MASTER_SHOULDER2": 0.011,
    "Z_BOT2_MASTER_SHOULDER2_2": 0.011,
    "3215_1Flange": 0.016,
    "3215_1Flange_2": 0.016,
    "R_ARM_MIRROR_1": 0.014,
    "L_ARM_MIRROR_1": 0.014,
    "FINGER_1": 0.018,
    "FINGER_1_2": 0.018,
    "U_HIP_L": 0.012,
    "U_HIP_R": 0.012,
    "3215_BothFlange_5": 0.015,
    "3215_BothFlange_6": 0.015,
    "3215_BothFlange_9": 0.018,
    "3215_BothFlange_10": 0.018,
    "3215_BothFlange_13": 0.017,
    "3215_BothFlange_14": 0.017,
    LEFT_ANKLE_CARRIER: 0.014,
    RIGHT_ANKLE_CARRIER: 0.014,
}


def sts3250_manifest_components() -> dict[str, dict[str, object]]:
    payload = json.loads(ASSEMBLY_MANIFEST.read_text(encoding="utf-8"))
    components: dict[str, dict[str, object]] = {}
    for component in payload["components"]:
        if component.get("role") != "dimension_controlled_sts3250":
            continue
        servo_id = str(component["component_id"]).split("_", 1)[0]
        components[servo_id] = component
    if len(components) != 18:
        raise RuntimeError(f"expected 18 STS3250 manifest components, got {len(components)}")
    return components


def component_world_transform(component: dict[str, object]):
    rows = component["transform_local_mm_to_world_mm"]
    rotation = tuple(tuple(float(rows[r][c]) for c in range(3)) for r in range(3))
    translation = tuple(float(rows[r][3]) * 0.001 for r in range(3))
    return rotation, translation


def copy_visuals(old_robot, old_link_name, new_link, neutral_tf, sts_components):
    old_link = next(
        link for link in old_robot.findall("link") if link.get("name") == old_link_name
    )
    for visual in old_link.findall("visual"):
        if old_link_name == BODY and visual.get("name") in REMOVED_BODY_VISUALS:
            continue
        copied = copy.deepcopy(visual)
        mesh = copied.find("./geometry/mesh")
        if mesh is not None:
            filename = str(mesh.get("filename"))
            servo_id = str(copied.get("name", "")).split("_", 1)[0]
            if servo_id in sts_components:
                component = sts_components[servo_id]
                if component["owner_link"] != old_link_name:
                    raise RuntimeError(
                        f"{servo_id} owner mismatch: {component['owner_link']} != {old_link_name}"
                    )
                local_tf = relative_transform(
                    neutral_tf[old_link_name],
                    component_world_transform(component),
                )
                origin = copied.find("origin")
                if origin is None:
                    origin = ET.SubElement(copied, "origin")
                origin.set("xyz", fmt(local_tf[1]))
                origin.set("rpy", fmt(matrix_rpy(local_tf[0])))
                mesh.set("filename", "meshes/v3/sts3250_dimension_controlled.stl")
                mesh.set("scale", "0.001 0.001 0.001")
            elif "left_sole.stl" in filename:
                mesh.set("filename", "meshes/v3/left_sole_lightweighted.stl")
            elif "right_sole.stl" in filename:
                mesh.set("filename", "meshes/v3/right_sole_lightweighted.stl")
        new_link.append(copied)


def ankle_servo_rotation(world_axis):
    """Match the roll-servo orientation used by the SolidWorks manifest."""

    z_axis = _unit(world_axis)
    x_axis = (0.0, 0.0, -1.0)
    y_axis = _cross(z_axis, x_axis)
    return tuple((x_axis[row], y_axis[row], z_axis[row]) for row in range(3))


def add_ankle_visual(link: ET.Element, side: str, neutral_tf) -> None:
    carrier = LEFT_ANKLE_CARRIER if side == "left" else RIGHT_ANKLE_CARRIER
    foot = "FOOT" if side == "left" else "FOOT_2"
    world_axis = (1.0, 0.0, 0.0) if side == "left" else (-1.0, 0.0, 0.0)
    part_world = (ankle_servo_rotation(world_axis), neutral_tf[foot][1])
    part_local = relative_transform(neutral_tf[carrier], part_world)
    visual = ET.SubElement(link, "visual", name=f"{side}_ankle_roll_carrier_visual")
    ET.SubElement(
        visual,
        "origin",
        xyz=fmt(part_local[1]),
        rpy=fmt(matrix_rpy(part_local[0])),
    )
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        filename=f"meshes/v3/{side}_ankle_roll_carrier.stl",
        scale="0.001 0.001 0.001",
    )
    add_material(visual, f"{side}_ankle_roll_carrier_white", "0.969 0.973 0.980 1")

    servo = ET.SubElement(link, "visual", name=f"{side}_ankle_roll_blue_servo_visual")
    ET.SubElement(
        servo,
        "origin",
        xyz=fmt(part_local[1]),
        rpy=fmt(matrix_rpy(part_local[0])),
    )
    geometry = ET.SubElement(servo, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        filename="meshes/v3/sts3250_dimension_controlled.stl",
        scale="0.001 0.001 0.001",
    )
    add_material(servo, f"{side}_ankle_roll_servo_blue", "0.086 0.467 1 1")


def add_joint(
    robot,
    name,
    parent,
    child,
    origin,
    axis=None,
    limits=None,
    kind="revolute",
    rpy=(0.0, 0.0, 0.0),
):
    joint = ET.SubElement(robot, "joint", name=name, type=kind)
    ET.SubElement(joint, "parent", link=parent)
    ET.SubElement(joint, "child", link=child)
    ET.SubElement(joint, "origin", xyz=fmt(origin), rpy=fmt(rpy))
    if axis is not None:
        ET.SubElement(joint, "axis", xyz=fmt(axis))
    if limits is not None:
        ET.SubElement(
            joint,
            "limit",
            lower=f"{limits[0]:.12g}",
            upper=f"{limits[1]:.12g}",
            effort=f"{CONTINUOUS_EFFORT_NM:.12g}",
            velocity=f"{MAX_VELOCITY_RAD_S:.12g}",
        )


def gen_urdf() -> ET.Element:
    old_robot = ET.parse(V2_URDF).getroot()
    old_tf = old_fk(old_robot)
    neutral_tf = neutral_transforms(old_tf)
    sts_components = sts3250_manifest_components()

    robot = ET.Element("robot", name="zeroth01_physical_mount_v3_rl_fixed_18dof")
    robot.append(ET.Comment(
        "v3 uses v2-minimal visual provenance, explicit mirrored kinematics, "
        "primitive collisions and nominal—not measured—as-built inertials."
    ))
    ET.SubElement(robot, "link", name="base_link")
    body_link = ET.SubElement(robot, "link", name=BODY)
    copy_visuals(old_robot, BODY, body_link, neutral_tf, sts_components)
    primitive_collision(body_link, BODY, neutral_tf)
    add_inertial(body_link, body_mass(), *collision_inertia_spec(BODY, neutral_tf))
    add_joint(
        robot,
        "base_to_body",
        "base_link",
        BODY,
        neutral_tf[BODY][1],
        kind="fixed",
        rpy=matrix_rpy(neutral_tf[BODY][0]),
    )

    physical_links = set(COLLISION) - {BODY}
    for name in sorted(physical_links):
        link = ET.SubElement(robot, "link", name=name)
        if name in {LEFT_ANKLE_CARRIER, RIGHT_ANKLE_CARRIER}:
            add_ankle_visual(
                link,
                "left" if name == LEFT_ANKLE_CARRIER else "right",
                neutral_tf,
            )
        else:
            copy_visuals(old_robot, name, link, neutral_tf, sts_components)
        primitive_collision(link, name, neutral_tf)
        add_inertial(
            link,
            FIXED_MASSES[name],
            *collision_inertia_spec(name, neutral_tf),
        )

    for name, parent, child, origin, axis, limits in joint_kinematics(
        old_robot, neutral_tf
    ):
        add_joint(
            robot,
            name,
            parent,
            child,
            origin[1],
            axis,
            limits,
            rpy=matrix_rpy(origin[0]),
        )

    for name, parent, child in FIXED_LINKS:
        origin = relative_transform(neutral_tf[parent], neutral_tf[child])
        add_joint(
            robot,
            name,
            parent,
            child,
            origin[1],
            kind="fixed",
            rpy=matrix_rpy(origin[0]),
        )

    old_links = {str(link.get("name")): link for link in old_robot.findall("link")}
    fixed_payloads = (
        ("IMU_2", BODY),
        ("torso_imu_module", BODY),
        ("compute_module", BODY),
        ("battery_pack", BODY),
    )
    for name, parent in fixed_payloads:
        link = ET.SubElement(robot, "link", name=name)
        for visual in old_links[name].findall("visual"):
            link.append(copy.deepcopy(visual))
        mass = FIXED_MASSES[name]
        if name == "compute_module":
            size = (0.105, 0.020, 0.070)
        elif name == "battery_pack":
            size = (0.075, 0.038, 0.038)
        else:
            size = (0.032, 0.025, 0.008)
        add_inertial(link, mass, size, (0.0, 0.0, 0.0))
        origin = relative_transform(neutral_tf[parent], old_tf[name])
        add_joint(
            robot,
            f"{name}_fixed_joint",
            parent,
            name,
            origin[1],
            kind="fixed",
            rpy=matrix_rpy(origin[0]),
        )

    adapter = ET.SubElement(robot, "link", name=STACKCHAN_HEAD_ADAPTER)
    add_box_visual_collision(
        adapter,
        STACKCHAN_HEAD_ADAPTER,
        STACKCHAN_ADAPTER_SIZE_M,
        "0.75 0.78 0.82 1",
    )
    add_inertial(
        adapter,
        FIXED_MASSES[STACKCHAN_HEAD_ADAPTER],
        STACKCHAN_ADAPTER_SIZE_M,
        (0.0, 0.0, 0.0),
    )
    adapter_world = (I3, STACKCHAN_ADAPTER_CENTER_M)
    adapter_local = relative_transform(neutral_tf[BODY], adapter_world)
    add_joint(
        robot,
        "stackchan_head_adapter_fixed_joint",
        BODY,
        STACKCHAN_HEAD_ADAPTER,
        adapter_local[1],
        kind="fixed",
        rpy=matrix_rpy(adapter_local[0]),
    )

    head = ET.SubElement(robot, "link", name=STACKCHAN_HEAD_POD)
    add_box_visual_collision(
        head,
        STACKCHAN_HEAD_POD,
        STACKCHAN_HEAD_SIZE_M,
        "0.97 0.98 0.99 1",
    )
    # A black front glass reference makes the purchased expression display
    # visible without pretending the supplier's internal B-Rep is ours.
    face = ET.SubElement(head, "visual", name="stackchan_front_glass_visual")
    ET.SubElement(face, "origin", xyz="0.031 0 -0.006", rpy="0 0 0")
    geometry = ET.SubElement(face, "geometry")
    ET.SubElement(geometry, "box", size="0.001 0.044 0.040")
    add_material(face, "stackchan_front_glass", "0.02 0.03 0.04 1")
    add_inertial(
        head,
        FIXED_MASSES[STACKCHAN_HEAD_POD],
        STACKCHAN_HEAD_SIZE_M,
        (0.0, 0.0, 0.0),
    )
    add_joint(
        robot,
        "stackchan_head_pod_fixed_joint",
        STACKCHAN_HEAD_ADAPTER,
        STACKCHAN_HEAD_POD,
        sub(STACKCHAN_HEAD_CENTER_M, STACKCHAN_ADAPTER_CENTER_M),
        kind="fixed",
    )

    for side, foot in (("left", "FOOT"), ("right", "FOOT_2")):
        foot_rotation_t = mat_t(neutral_tf[foot][0])
        for corner, world_offset in (
            ("front_medial", (0.045, -0.030, -0.027)),
            ("front_lateral", (0.045, 0.030, -0.027)),
            ("rear_medial", (-0.055, -0.030, -0.027)),
            ("rear_lateral", (-0.055, 0.030, -0.027)),
        ):
            xyz = mat_vec(foot_rotation_t, world_offset)
            frame = f"{side}_sole_{corner}_contact"
            ET.SubElement(robot, "link", name=frame)
            add_joint(robot, f"{frame}_joint", foot, frame, xyz, kind="fixed")

    sensor_frames = (
        ("camera_optical_frame", STACKCHAN_HEAD_POD, (0.03075, 0.0, -0.020), "0 1.57079632679 0"),
        ("left_microphone_frame", STACKCHAN_HEAD_POD, (0.03075, 0.015, -0.020), "0 0 0"),
        ("right_microphone_frame", STACKCHAN_HEAD_POD, (0.03075, -0.015, -0.020), "0 0 0"),
        ("head_speaker_frame", STACKCHAN_HEAD_POD, (-0.03075, 0.0, 0.0), "0 0 0"),
        ("head_imu_frame", STACKCHAN_HEAD_POD, (0.0, 0.0, 0.0), "0 0 0"),
    )
    for name, parent, xyz, rpy in sensor_frames:
        ET.SubElement(robot, "link", name=name)
        add_joint(robot, f"{name}_joint", parent, name, xyz, kind="fixed")
        robot.findall("joint")[-1].find("origin").set("rpy", rpy)

    ET.indent(robot, space="  ")
    return robot


def copy_meshes(robot: ET.Element) -> None:
    mesh_root = OUT_ROOT / "meshes"
    if mesh_root.is_dir():
        shutil.rmtree(mesh_root)
    v3_source = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed" / "parts"
    filenames = sorted(
        {str(mesh.get("filename")) for mesh in robot.findall(".//mesh")}
    )
    for filename in filenames:
        relative = Path(filename)
        if not relative.parts or relative.parts[0] != "meshes":
            raise RuntimeError(f"non-portable mesh reference: {filename}")
        source = (
            v3_source / relative.name
            if len(relative.parts) >= 2 and relative.parts[1] == "v3"
            else V2_ROOT / relative
        )
        if not source.is_file():
            raise FileNotFoundError(source)
        target = OUT_ROOT / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def main() -> int:
    robot = gen_urdf()
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    copy_meshes(robot)
    ET.ElementTree(robot).write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    masses = [
        float(mass.get("value"))
        for mass in robot.findall("./link/inertial/mass")
    ]
    movable = [joint for joint in robot.findall("joint") if joint.get("type") == "revolute"]
    total = sum(masses)
    if len(movable) != 18:
        raise RuntimeError(f"expected 18 movable joints, got {len(movable)}")
    if abs(total - TARGET_TOTAL_MASS_KG) > 1.0e-9:
        raise RuntimeError((total, TARGET_TOTAL_MASS_KG))
    print(OUT_URDF)
    print(f"movable_joints={len(movable)} total_mass_kg={total:.9f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
