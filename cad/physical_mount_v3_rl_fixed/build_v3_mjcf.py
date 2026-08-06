"""Generate the primitive-only MuJoCo/MJX source from the v3 URDF ledger."""

from __future__ import annotations

import importlib.util
import math
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "generated" / "mujoco" / "physical_mount_v3_rl_fixed"
MJCF = OUT / "zeroth01_physical_mount_v3_rl_fixed_18dof_mjx.xml"


def load_urdf_module():
    path = Path(__file__).with_name("build_v3_urdf.py")
    spec = importlib.util.spec_from_file_location("zeroth_v3_urdf", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def fmt(values):
    return " ".join(f"{float(value):.12g}" for value in values)


def matrix_quat(matrix):
    """Convert a proper rotation matrix to MuJoCo w-x-y-z quaternion."""

    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2][1] - matrix[1][2]) / scale
        y = (matrix[0][2] - matrix[2][0]) / scale
        z = (matrix[1][0] - matrix[0][1]) / scale
    elif matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        w = (matrix[2][1] - matrix[1][2]) / scale
        x = 0.25 * scale
        y = (matrix[0][1] + matrix[1][0]) / scale
        z = (matrix[0][2] + matrix[2][0]) / scale
    elif matrix[1][1] > matrix[2][2]:
        scale = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        w = (matrix[0][2] - matrix[2][0]) / scale
        x = (matrix[0][1] + matrix[1][0]) / scale
        y = 0.25 * scale
        z = (matrix[1][2] + matrix[2][1]) / scale
    else:
        scale = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
        w = (matrix[1][0] - matrix[0][1]) / scale
        x = (matrix[0][2] + matrix[2][0]) / scale
        y = (matrix[1][2] + matrix[2][1]) / scale
        z = 0.25 * scale
    length = math.sqrt(w * w + x * x + y * y + z * z)
    return (w / length, x / length, y / length, z / length)


def inertia_box(mass, size):
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def add_link_collision(parent, m, link_name, neutral_tf, rgba):
    if link_name in {m.BODY, "FOOT", "FOOT_2"}:
        size, center = m.collision_inertia_spec(link_name, neutral_tf)
        ET.SubElement(
            parent,
            "geom",
            name=f"{link_name}_collision" if link_name != m.BODY else "torso_collision",
            type="box",
            size=fmt(tuple(value / 2.0 for value in size)),
            pos=fmt(center),
            rgba=rgba,
            friction="1.2 0.02 0.001" if link_name in {"FOOT", "FOOT_2"} else "1.0 0.02 0.001",
        )
        return
    radius = m.LINK_RADIUS_M.get(link_name, 0.014)
    child = m.SEGMENT_CHILD.get(link_name)
    delta = (
        m.relative_transform(neutral_tf[link_name], neutral_tf[child])[1]
        if child
        else (0.0, 0.0, 0.0)
    )
    length = math.sqrt(sum(value * value for value in delta))
    kwargs = {
        "name": f"{link_name}_collision",
        "rgba": rgba,
    }
    if length <= 0.004:
        ET.SubElement(parent, "geom", type="sphere", size=f"{radius:.12g}", **kwargs)
    else:
        ET.SubElement(
            parent,
            "geom",
            type="capsule",
            fromto=fmt((0.0, 0.0, 0.0) + delta),
            size=f"{radius:.12g}",
            **kwargs,
        )


def main() -> int:
    m = load_urdf_module()
    old_robot = ET.parse(m.V2_URDF).getroot()
    old_tf = m.old_fk(old_robot)
    neutral_tf = m.neutral_transforms(old_tf)
    sole_bottom = min(
        neutral_tf[name][1][2]
        + m.COLLISION[name][1][2]
        - m.COLLISION[name][0][2] / 2.0
        for name in ("FOOT", "FOOT_2")
    )
    standing_height = -sole_bottom

    root = ET.Element("mujoco", model="zeroth01_physical_mount_v3_rl_fixed_18dof_mjx")
    ET.SubElement(root, "compiler", angle="radian", coordinate="local", autolimits="true")
    ET.SubElement(root, "option", timestep="0.002", gravity="0 0 -9.81", integrator="implicitfast")
    ET.SubElement(
        root,
        "size",
        njmax="2000",
        nconmax="400",
    )
    default = ET.SubElement(root, "default")
    ET.SubElement(default, "joint", damping="0.035", armature="0.002", frictionloss="0.01")
    ET.SubElement(default, "geom", margin="0.001", condim="3", friction="1.0 0.02 0.001")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "headlight", diffuse="0.75 0.75 0.75", ambient="0.35 0.35 0.35")
    ET.SubElement(visual, "rgba", haze="0.15 0.18 0.22 1")

    world = ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -1 1.2", dir="0 0 -1", directional="true")
    ET.SubElement(
        world,
        "geom",
        name="ground",
        type="plane",
        size="2 2 0.02",
        rgba="0.82 0.84 0.88 1",
        friction="1.2 0.02 0.001",
    )
    body = ET.SubElement(
        world,
        "body",
        name=m.BODY,
        pos=fmt(neutral_tf[m.BODY][1]),
        quat=fmt(matrix_quat(neutral_tf[m.BODY][0])),
    )
    ET.SubElement(body, "freejoint", name="floating_base")
    torso_size, torso_center = m.collision_inertia_spec(m.BODY, neutral_tf)
    torso_mass = m.body_mass()
    ET.SubElement(
        body,
        "inertial",
        pos=fmt(torso_center),
        mass=f"{torso_mass:.12g}",
        diaginertia=fmt(inertia_box(torso_mass, torso_size)),
    )
    add_link_collision(body, m, m.BODY, neutral_tf, "0.93 0.94 0.96 1")
    ET.SubElement(body, "site", name="imu_site", pos="0 0 0.02", size="0.006", rgba="0.2 1 0.2 1")

    body_nodes = {m.BODY: body}
    joint_order = []
    pending = list(m.joint_kinematics(old_robot, neutral_tf))
    while pending:
        progressed = False
        for spec in pending[:]:
            name, parent, child, origin, axis, limits = spec
            if parent not in body_nodes:
                continue
            child_body = ET.SubElement(
                body_nodes[parent],
                "body",
                name=child,
                pos=fmt(origin[1]),
                quat=fmt(matrix_quat(origin[0])),
            )
            ET.SubElement(
                child_body,
                "joint",
                name=name,
                type="hinge",
                axis=fmt(axis),
                range=fmt(limits),
                limited="true",
            )
            size, center = m.collision_inertia_spec(child, neutral_tf)
            mass = m.FIXED_MASSES[child]
            ET.SubElement(
                child_body,
                "inertial",
                pos=fmt(center),
                mass=f"{mass:.12g}",
                diaginertia=fmt(inertia_box(mass, size)),
            )
            rgba = "0.92 0.93 0.95 1"
            if child in {m.LEFT_ANKLE_CARRIER, m.RIGHT_ANKLE_CARRIER}:
                rgba = "0.086 0.467 1 1"
            if child in {"FOOT", "FOOT_2"}:
                rgba = "0.12 0.14 0.17 1"
            add_link_collision(child_body, m, child, neutral_tf, rgba)
            if child in {"FOOT", "FOOT_2"}:
                side = "left" if child == "FOOT" else "right"
                foot_rotation_t = m.mat_t(neutral_tf[child][0])
                for corner, world_offset in (
                    ("front_medial", (0.045, -0.030, -0.027)),
                    ("front_lateral", (0.045, 0.030, -0.027)),
                    ("rear_medial", (-0.055, -0.030, -0.027)),
                    ("rear_lateral", (-0.055, 0.030, -0.027)),
                ):
                    xyz = m.mat_vec(foot_rotation_t, world_offset)
                    ET.SubElement(
                        child_body,
                        "site",
                        name=f"{side}_sole_{corner}",
                        pos=fmt(xyz),
                        size="0.004",
                        rgba="0 1 0.8 1",
                    )
            body_nodes[child] = child_body
            joint_order.append(name)
            pending.remove(spec)
            progressed = True
        if not progressed:
            raise RuntimeError(f"unresolved kinematic bodies: {pending}")

    for _, parent, child in m.FIXED_LINKS:
        origin = m.relative_transform(neutral_tf[parent], neutral_tf[child])
        node = ET.SubElement(
            body_nodes[parent],
            "body",
            name=child,
            pos=fmt(origin[1]),
            quat=fmt(matrix_quat(origin[0])),
        )
        size, center = m.collision_inertia_spec(child, neutral_tf)
        mass = m.FIXED_MASSES[child]
        ET.SubElement(node, "inertial", pos=fmt(center), mass=f"{mass:.12g}", diaginertia=fmt(inertia_box(mass, size)))
        add_link_collision(node, m, child, neutral_tf, "0.94 0.95 0.97 1")
        body_nodes[child] = node

    payload_size = {
        "IMU_2": (0.032, 0.025, 0.008),
        "torso_imu_module": (0.032, 0.025, 0.008),
        "compute_module": (0.105, 0.020, 0.070),
        "battery_pack": (0.075, 0.038, 0.038),
    }
    payload_rgba = {
        "compute_module": "1 0.57 0 1",
        "battery_pack": "0.84 0 0.98 1",
        "torso_imu_module": "0.39 0.87 0.09 1",
        "IMU_2": "0.39 0.87 0.09 1",
    }
    for name, size in payload_size.items():
        origin = m.relative_transform(neutral_tf[m.BODY], old_tf[name])
        node = ET.SubElement(
            body,
            "body",
            name=name,
            pos=fmt(origin[1]),
            quat=fmt(matrix_quat(origin[0])),
        )
        mass = m.FIXED_MASSES[name]
        ET.SubElement(node, "inertial", pos="0 0 0", mass=f"{mass:.12g}", diaginertia=fmt(inertia_box(mass, size)))
        ET.SubElement(node, "geom", name=f"{name}_visual", type="box", size=fmt(tuple(value / 2.0 for value in size)), contype="0", conaffinity="0", rgba=payload_rgba[name])

    adapter_world = (m.I3, m.STACKCHAN_ADAPTER_CENTER_M)
    adapter_origin = m.relative_transform(neutral_tf[m.BODY], adapter_world)
    adapter = ET.SubElement(
        body,
        "body",
        name=m.STACKCHAN_HEAD_ADAPTER,
        pos=fmt(adapter_origin[1]),
        quat=fmt(matrix_quat(adapter_origin[0])),
    )
    adapter_mass = m.FIXED_MASSES[m.STACKCHAN_HEAD_ADAPTER]
    ET.SubElement(
        adapter,
        "inertial",
        pos="0 0 0",
        mass=f"{adapter_mass:.12g}",
        diaginertia=fmt(inertia_box(adapter_mass, m.STACKCHAN_ADAPTER_SIZE_M)),
    )
    ET.SubElement(
        adapter,
        "geom",
        name=f"{m.STACKCHAN_HEAD_ADAPTER}_collision",
        type="box",
        size=fmt(tuple(value / 2.0 for value in m.STACKCHAN_ADAPTER_SIZE_M)),
        rgba="0.75 0.78 0.82 1",
    )

    head = ET.SubElement(
        adapter,
        "body",
        name=m.STACKCHAN_HEAD_POD,
        pos=fmt(m.sub(m.STACKCHAN_HEAD_CENTER_M, m.STACKCHAN_ADAPTER_CENTER_M)),
    )
    head_mass = m.FIXED_MASSES[m.STACKCHAN_HEAD_POD]
    ET.SubElement(
        head,
        "inertial",
        pos="0 0 0",
        mass=f"{head_mass:.12g}",
        diaginertia=fmt(inertia_box(head_mass, m.STACKCHAN_HEAD_SIZE_M)),
    )
    ET.SubElement(
        head,
        "geom",
        name=f"{m.STACKCHAN_HEAD_POD}_collision",
        type="box",
        size=fmt(tuple(value / 2.0 for value in m.STACKCHAN_HEAD_SIZE_M)),
        rgba="0.97 0.98 0.99 1",
    )
    ET.SubElement(
        head,
        "site",
        name="stackchan_camera_site",
        pos="0.03075 0 -0.020",
        size="0.004",
        rgba="0 0.8 1 1",
    )

    actuator = ET.SubElement(root, "actuator")
    for name in joint_order:
        ET.SubElement(
            actuator,
            "motor",
            name=f"{name}_motor",
            joint=name,
            gear="1",
            ctrllimited="true",
            ctrlrange=fmt((-m.CONTINUOUS_EFFORT_NM, m.CONTINUOUS_EFFORT_NM)),
        )

    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "gyro", name="imu_gyro", site="imu_site")
    ET.SubElement(sensor, "accelerometer", name="imu_accelerometer", site="imu_site")
    for side in ("left", "right"):
        for corner in ("front_medial", "front_lateral", "rear_medial", "rear_lateral"):
            ET.SubElement(sensor, "touch", name=f"{side}_{corner}_touch", site=f"{side}_sole_{corner}")

    keyframe = ET.SubElement(root, "keyframe")
    body_position = m.add(neutral_tf[m.BODY][1], (0.0, 0.0, standing_height))
    qpos = body_position + matrix_quat(neutral_tf[m.BODY][0]) + (0.0,) * len(joint_order)
    ET.SubElement(keyframe, "key", name="official_standing", qpos=fmt(qpos))
    # A symmetric crouch is useful as a reset diversification pose.  Joint
    # order follows the generated tree and is also written to the handoff.
    crouch = list(qpos)
    for joint_name, value in (
        ("left_hip_pitch", 0.18),
        ("right_hip_pitch", -0.18),
        ("left_knee_pitch", -0.36),
        ("right_knee_pitch", 0.36),
        ("left_ankle_pitch", -0.18),
        ("right_ankle_pitch", 0.18),
    ):
        crouch[7 + joint_order.index(joint_name)] = value
    ET.SubElement(keyframe, "key", name="symmetric_crouch", qpos=fmt(crouch))

    ET.indent(root, space="  ")
    OUT.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(MJCF, encoding="utf-8", xml_declaration=True)
    print(MJCF)
    print(f"standing_height_m={standing_height:.9f} joints={len(joint_order)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
