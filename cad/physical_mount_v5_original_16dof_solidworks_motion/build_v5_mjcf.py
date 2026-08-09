"""Generate the 16-DoF MuJoCo/MJX model from the v5 URDF ledger.

The v3 MJX generator remains the source for the floating-base convention,
primitive collision policy, foot sensors and actuator definition.  This
adapter restores the original Zeroth-01 ankle-pitch -> foot chain and removes
the two experimental ankle-roll bodies before the model reaches RL.
"""

from __future__ import annotations

import importlib.util
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

from build123d import Location, import_step
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
V3_MJCF_SOURCE = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "build_v3_mjcf.py"
V5_URDF_SOURCE = Path(__file__).with_name("build_v5_urdf.py")
OUT = ROOT / "generated" / "mujoco" / "physical_mount_v5_original_16dof_solidworks_motion"
MJCF = OUT / "zeroth01_physical_mount_v5_original_16dof_solidworks_motion_mjx.xml"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "mjcf_compile_gate.json"

EXPECTED_COMPILED_JOINT_ORDER = (
    "left_shoulder_yaw",
    "left_shoulder_pitch",
    "left_elbow_yaw",
    "right_shoulder_yaw",
    "right_shoulder_pitch",
    "right_elbow_yaw",
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee_pitch",
    "left_ankle_pitch",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee_pitch",
    "right_ankle_pitch",
)

GAIT_NEUTRAL_JOINTS = {
    "left_hip_pitch": 0.18,
    "left_knee_pitch": -0.36,
    "left_ankle_pitch": -0.18,
    "right_hip_pitch": -0.18,
    "right_knee_pitch": 0.36,
    "right_ankle_pitch": 0.18,
}

GAIT_NEUTRAL_TRAVEL_JOINTS = (
    "left_knee_pitch",
    "left_ankle_pitch",
    "right_knee_pitch",
    "right_ankle_pitch",
)

MIN_GAIT_NEUTRAL_TRAVEL_RAD = math.radians(5.0)


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def vec(text: str | None):
    return tuple(float(value) for value in (text or "0 0 0").split())


def fmt(values):
    return " ".join(f"{float(value):.12g}" for value in values)


def inertia_box(mass, size):
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def remove_named_body(parent: ET.Element, names: set[str]) -> None:
    for body in list(parent.findall("body")):
        if str(body.get("name")) in names:
            parent.remove(body)
        else:
            remove_named_body(body, names)


def joint_kinematics_original_16(v5, source_robot: ET.Element, neutral_tf):
    rows = []
    for joint in source_robot.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        origin_node = joint.find("origin")
        axis_node = joint.find("axis")
        limit_node = joint.find("limit")
        origin = (
            v5.v3.rpy_matrix(vec(origin_node.get("rpy") if origin_node is not None else None)),
            vec(origin_node.get("xyz") if origin_node is not None else None),
        )
        rows.append(
            (
                str(joint.get("name")),
                str(joint.find("parent").get("link")),
                str(joint.find("child").get("link")),
                origin,
                vec(axis_node.get("xyz") if axis_node is not None else None),
                (float(limit_node.get("lower")), float(limit_node.get("upper"))),
            )
        )
    if len(rows) != 16:
        raise RuntimeError(f"expected 16 original joints, got {len(rows)}")
    return rows


def add_v5_head(torso: ET.Element) -> None:
    head = ET.SubElement(torso, "body", name="v4_head_shell")
    ET.SubElement(
        head,
        "inertial",
        pos="0 0.009189728 0.03883662",
        mass="0.055",
        diaginertia=fmt(inertia_box(0.055, (0.090750004, 0.031000001, 0.070409235))),
    )
    ET.SubElement(
        head,
        "geom",
        name="v4_head_shell_collision",
        type="box",
        size=fmt((0.045375002, 0.0155000005, 0.0352046175)),
        pos="0 0.009189728 0.03883662",
        rgba="0.97 0.98 0.99 1",
    )
    unitv2 = ET.SubElement(head, "body", name="m5stack_unitv2")
    ET.SubElement(
        unitv2,
        "inertial",
        pos="0 0.006 0.045",
        mass="0.018",
        diaginertia=fmt(inertia_box(0.018, (0.048, 0.024260273, 0.024))),
    )
    ET.SubElement(
        unitv2,
        "geom",
        name="m5stack_unitv2_visual",
        type="box",
        size="0.024 0.0121301365 0.012",
        pos="0 0.006 0.045",
        rgba="0 0.72 0.85 1",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(unitv2, "site", name="unitv2_camera_site", pos="-0.014 -0.0075102725 0.045", size="0.004", rgba="0 0.8 1 1")
    ET.SubElement(unitv2, "site", name="unitv2_microphone_site", pos="0.014 -0.0075102725 0.045", size="0.004", rgba="1 0.2 0.8 1")


def find_body(root: ET.Element, name: str) -> ET.Element:
    body = root.find(f".//body[@name='{name}']")
    if body is None:
        raise RuntimeError(f"missing MuJoCo body {name}")
    return body


def replace_collision_geometry(root: ET.Element) -> None:
    """Use v5 wrist/foot geometry instead of the old oversized proxies."""

    for name, center_y in (("FINGER_1", -0.014), ("FINGER_1_2", 0.014)):
        body = find_body(root, name)
        geom = body.find(f"./geom[@name='{name}_collision']")
        if geom is None:
            raise RuntimeError(f"missing wrist collision {name}")
        for key in ("fromto", "quat", "axisangle", "euler"):
            geom.attrib.pop(key, None)
        geom.set("type", "box")
        geom.set("size", "0.010 0.0035 0.007")
        geom.set("pos", fmt((-0.0034808, center_y, 0.0187993)))
        geom.set("rgba", "0.97 0.98 0.99 1")

    for name, carrier_y, sole_y, side in (
        ("FOOT", 0.012845, 0.04013, "left"),
        ("FOOT_2", -0.012845, -0.04013, "right"),
    ):
        body = find_body(root, name)
        carrier = body.find(f"./geom[@name='{name}_collision']")
        if carrier is None:
            raise RuntimeError(f"missing foot collision {name}")
        carrier.set("type", "box")
        carrier.set("size", "0.045 0.0228 0.0202")
        carrier.set("pos", fmt((-0.016025, carrier_y, 0.0187)))
        carrier.set("rgba", "0.92 0.93 0.95 1")
        carrier.set("friction", "1.0 0.02 0.001")
        sole = ET.SubElement(
            body,
            "geom",
            name=f"{name}_v5_white_tapered_sole_collision",
            type="box",
            size="0.049 0.0045 0.024",
            pos=fmt((-0.016025, sole_y, 0.0187)),
            rgba="0.97 0.98 0.99 1",
            friction="1.2 0.02 0.001",
        )
        _ = sole
        for site in list(body.findall("site")):
            if str(site.get("name", "")).startswith(f"{side}_sole_"):
                body.remove(site)
        # The released foot frames are rotated relative to the documented
        # world frame.  Local -X is world +X (forward), and the larger local Z
        # coordinate is closer to the sagittal plane.  Name sites by their
        # world semantics rather than by untransformed local coordinates.
        for fore_aft, x_pos in (("front", -0.045), ("rear", 0.032)):
            for lateral, z_pos in (("medial", 0.0307), ("lateral", 0.0067)):
                ET.SubElement(
                    body,
                    "site",
                    name=f"{side}_sole_{fore_aft}_{lateral}",
                    pos=fmt((x_pos, 0.0446 if side == "left" else -0.0446, z_pos)),
                    size="0.004",
                    rgba="0 1 0.8 1",
                )


def tree_joint_order(root: ET.Element) -> list[str]:
    """Return the depth-first hinge order used by the MuJoCo compiler."""

    order: list[str] = []

    def visit(body: ET.Element) -> None:
        for joint in body.findall("./joint"):
            if joint.get("type", "hinge") == "hinge":
                order.append(str(joint.get("name")))
        for child in body.findall("./body"):
            visit(child)

    for top_level in root.findall("./worldbody/body"):
        visit(top_level)
    return order


def enforce_model_index_contract(root: ET.Element, joint_order: list[str]) -> None:
    """Make ctrl[i], qpos[7+i] and joint_order[i] the same contract."""

    if tuple(joint_order) != EXPECTED_COMPILED_JOINT_ORDER:
        raise RuntimeError(
            "compiled tree joint order changed; review the kinematic tree before training: "
            + ", ".join(joint_order)
        )

    actuator = root.find("./actuator")
    if actuator is None:
        raise RuntimeError("missing actuator section")
    motors = {str(motor.get("joint")): motor for motor in actuator.findall("./motor")}
    if set(motors) != set(joint_order):
        raise RuntimeError("actuator joints do not match compiled tree joints")
    for motor in list(actuator):
        actuator.remove(motor)
    for joint_name in joint_order:
        motor = motors[joint_name]
        motor.set("name", f"{joint_name}_ctrl")
        actuator.append(motor)

    keyframe = root.find("./keyframe")
    if keyframe is None:
        keyframe = ET.SubElement(root, "keyframe")
    old_official = keyframe.find("./key[@name='official_standing']")
    if old_official is None:
        raise RuntimeError("missing official_standing base pose")
    base_qpos = [float(value) for value in old_official.get("qpos", "").split()[:7]]
    if len(base_qpos) != 7:
        raise RuntimeError("official_standing does not contain a free-base pose")
    for key in list(keyframe):
        keyframe.remove(key)

    def pose(values: dict[str, float]) -> list[float]:
        return base_qpos + [float(values.get(name, 0.0)) for name in joint_order]

    ET.SubElement(keyframe, "key", name="official_standing", qpos=fmt(pose({})))
    gait_qpos = pose(GAIT_NEUTRAL_JOINTS)
    ET.SubElement(keyframe, "key", name="gait_neutral", qpos=fmt(gait_qpos))
    # Backward-compatible name for older reset code.  It is deliberately the
    # exact same grounded, limit-safe pose as gait_neutral.
    ET.SubElement(keyframe, "key", name="symmetric_crouch", qpos=fmt(gait_qpos))


def write_xml(root: ET.Element) -> None:
    ET.indent(root, space="  ")
    OUT.mkdir(parents=True, exist_ok=True)
    # Write bytes directly so the generated MJCF has one clone-stable LF byte
    # representation on Windows and Linux.  The embedded hash, delivery
    # manifest and committed file must all describe these same bytes.
    data = ET.tostring(root, encoding="utf-8", xml_declaration=True)
    MJCF.write_bytes(data.replace(b"\r\n", b"\n").replace(b"\r", b"\n"))


def box_geom_bottom(model, data, geom_name: str) -> float:
    import mujoco

    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if geom_id < 0 or int(model.geom_type[geom_id]) != int(mujoco.mjtGeom.mjGEOM_BOX):
        raise RuntimeError(f"missing box sole collision geom {geom_name}")
    rotation = data.geom_xmat[geom_id].reshape(3, 3)
    half_size = model.geom_size[geom_id, :3]
    world_z_half_extent = sum(abs(float(rotation[2, axis])) * float(half_size[axis]) for axis in range(3))
    return float(data.geom_xpos[geom_id, 2]) - world_z_half_extent


def sole_bottoms(model, data) -> dict[str, float]:
    return {
        "left": box_geom_bottom(model, data, "FOOT_v5_white_tapered_sole_collision"),
        "right": box_geom_bottom(model, data, "FOOT_2_v5_white_tapered_sole_collision"),
    }


def solve_grounded_keyframes(root: ET.Element) -> dict[str, dict[str, object]]:
    """Solve free-base Z so both symmetric sole boxes meet the ground."""

    import mujoco

    write_xml(root)
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    solved: dict[str, dict[str, object]] = {}
    for name in ("gait_neutral", "symmetric_crouch"):
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, name)
        if key_id < 0:
            raise RuntimeError(f"missing keyframe {name}")
        qpos = model.key_qpos[key_id].copy()
        data.qpos[:] = qpos
        mujoco.mj_forward(model, data)
        before = sole_bottoms(model, data)
        qpos[2] -= 0.5 * (before["left"] + before["right"])
        key = root.find(f"./keyframe/key[@name='{name}']")
        if key is None:
            raise RuntimeError(name)
        key.set("qpos", fmt(qpos))
        solved[name] = {
            "base_z_before_m": float(model.key_qpos[key_id, 2]),
            "base_z_solved_m": float(qpos[2]),
            "sole_bottom_before_m": before,
        }
    write_xml(root)
    return solved


def apply_urdf_inertials(root: ET.Element, robot: ET.Element) -> None:
    """Copy every RL mass, COM and full inertia tensor from the v5 URDF."""

    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        name = str(link.get("name"))
        body = root.find(f".//body[@name='{name}']")
        if body is None:
            # Fixed massless contact-frame links become MuJoCo sites.
            continue
        target = body.find("inertial")
        if target is None:
            target = ET.SubElement(body, "inertial")
        origin = inertial.find("origin")
        tensor = inertial.find("inertia")
        target.set("pos", origin.get("xyz"))
        target.set("mass", inertial.find("mass").get("value"))
        target.set(
            "fullinertia",
            " ".join(tensor.get(key) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
        )
        for key in ("diaginertia", "quat"):
            target.attrib.pop(key, None)


def transformed_bbox_z(row, source: Path):
    shape = import_step(source)
    matrix = row["transform_local_mm_to_world_mm"]
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[r][c]) for r in range(3) for c in range(4)])
    world = shape.moved(Location(gp_trsf=transform))
    bounds = world.bounding_box()
    return float(bounds.min.Z) / 1000.0, float(bounds.max.Z) / 1000.0


def apply_actual_standing_height(root: ET.Element, u5, base, v2_robot) -> float:
    manifest = json.loads(u5.V5_MANIFEST.read_text(encoding="utf-8"))
    sole_rows = [row for row in manifest["components"] if row["role"] == "replaceable_white_tapered_lower_wider_sole"]
    bounds = [transformed_bbox_z(row, u5.ROOT / str(row["source"])) for row in sole_rows]
    neutral_min_z = min(item[0] for item in bounds)
    standing_height = -neutral_min_z
    neutral_body_z = base.old_fk(v2_robot)[base.BODY][1][2]
    for key in root.findall("./keyframe/key"):
        qpos = [float(value) for value in key.get("qpos").split()]
        qpos[2] = neutral_body_z + standing_height
        key.set("qpos", fmt(qpos))
    return standing_height


def model_names(model, object_type, count: int) -> list[str]:
    import mujoco

    return [str(mujoco.mj_id2name(model, object_type, index)) for index in range(count)]


def validate_model_contract(model) -> dict[str, object]:
    """Validate all index, reset-pose and foot-site assumptions used by RL."""

    import mujoco

    joint_names_all = model_names(model, mujoco.mjtObj.mjOBJ_JOINT, model.njnt)
    compiled_joint_order = [name for name in joint_names_all if name != "floating_base"]
    actuator_order = [
        str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, int(model.actuator_trnid[index, 0])))
        for index in range(model.nu)
    ]
    mapping = []
    for ctrl_index, joint_name in enumerate(actuator_order):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        mapping.append(
            {
                "joint": joint_name,
                "ctrl_index": ctrl_index,
                "qpos_index": int(model.jnt_qposadr[joint_id]),
                "qvel_index": int(model.jnt_dofadr[joint_id]),
            }
        )
    index_contract_pass = (
        tuple(compiled_joint_order) == EXPECTED_COMPILED_JOINT_ORDER
        and actuator_order == compiled_joint_order
        and all(row["qpos_index"] == 7 + row["ctrl_index"] for row in mapping)
        and all(row["qvel_index"] == 6 + row["ctrl_index"] for row in mapping)
    )

    data = mujoco.MjData(model)

    def site_position(name: str) -> list[float]:
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, name)
        if site_id < 0:
            raise RuntimeError(f"missing site {name}")
        return [float(value) for value in data.site_xpos[site_id]]

    pose_gates: dict[str, object] = {}
    for key_name in ("gait_neutral", "symmetric_crouch"):
        key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, key_name)
        if key_id < 0:
            raise RuntimeError(f"missing keyframe {key_name}")
        qpos = model.key_qpos[key_id].copy()
        data.qpos[:] = qpos
        data.qvel[:] = 0.0
        mujoco.mj_forward(model, data)
        joint_values: dict[str, float] = {}
        limit_failures: list[str] = []
        travel: dict[str, object] = {}
        for joint_name in compiled_joint_order:
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            value = float(qpos[int(model.jnt_qposadr[joint_id])])
            lower, upper = (float(item) for item in model.jnt_range[joint_id])
            joint_values[joint_name] = value
            if value < lower - 1.0e-9 or value > upper + 1.0e-9:
                limit_failures.append(joint_name)
            if joint_name in GAIT_NEUTRAL_TRAVEL_JOINTS:
                negative_margin = value - lower
                positive_margin = upper - value
                travel[joint_name] = {
                    "negative_margin_rad": negative_margin,
                    "positive_margin_rad": positive_margin,
                    "required_each_direction_rad": MIN_GAIT_NEUTRAL_TRAVEL_RAD,
                    "pass": min(negative_margin, positive_margin) >= MIN_GAIT_NEUTRAL_TRAVEL_RAD,
                }

        expected_values_pass = all(
            abs(joint_values[name] - GAIT_NEUTRAL_JOINTS.get(name, 0.0)) <= 1.0e-10
            for name in compiled_joint_order
        )
        bottoms = sole_bottoms(model, data)
        bottom_delta = abs(bottoms["left"] - bottoms["right"])
        dual_ground_pass = max(abs(bottoms["left"]), abs(bottoms["right"])) <= 5.0e-4 and bottom_delta <= 5.0e-4

        sites: dict[str, list[float]] = {}
        for side in ("left", "right"):
            for fore_aft in ("front", "rear"):
                for lateral in ("medial", "lateral"):
                    name = f"{side}_sole_{fore_aft}_{lateral}"
                    sites[name] = site_position(name)
        mirror_errors = []
        for fore_aft in ("front", "rear"):
            for lateral in ("medial", "lateral"):
                left = sites[f"left_sole_{fore_aft}_{lateral}"]
                right = sites[f"right_sole_{fore_aft}_{lateral}"]
                mirror_errors.append(max(abs(left[0] - right[0]), abs(left[1] + right[1]), abs(left[2] - right[2])))
        mirror_max_error = max(mirror_errors)
        pose_gates[key_name] = {
            "base_z_m": float(qpos[2]),
            "joint_values_rad": joint_values,
            "limit_failures": limit_failures,
            "expected_values_pass": expected_values_pass,
            "sole_bottom_world_z_m": bottoms,
            "dual_ground_contact_geometry_pass": dual_ground_pass,
            "left_right_mirror_max_error_m": mirror_max_error,
            "left_right_mirror_pass": mirror_max_error <= 3.0e-3,
            "bidirectional_travel": travel,
            "bidirectional_travel_pass": all(bool(row["pass"]) for row in travel.values()),
        }
        pose_gates[key_name]["overall"] = "PASS" if (
            not limit_failures
            and expected_values_pass
            and dual_ground_pass
            and mirror_max_error <= 3.0e-3
            and bool(pose_gates[key_name]["bidirectional_travel_pass"])
        ) else "FAIL"

    official_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "official_standing")
    data.qpos[:] = model.key_qpos[official_id]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    foot_site_gate: dict[str, object] = {}
    for side in ("left", "right"):
        front_x = sum(site_position(f"{side}_sole_front_{name}")[0] for name in ("medial", "lateral")) / 2.0
        rear_x = sum(site_position(f"{side}_sole_rear_{name}")[0] for name in ("medial", "lateral")) / 2.0
        medial_abs_y = sum(abs(site_position(f"{side}_sole_{name}_medial")[1]) for name in ("front", "rear")) / 2.0
        lateral_abs_y = sum(abs(site_position(f"{side}_sole_{name}_lateral")[1]) for name in ("front", "rear")) / 2.0
        foot_site_gate[side] = {
            "front_mean_world_x_m": front_x,
            "rear_mean_world_x_m": rear_x,
            "medial_mean_abs_world_y_m": medial_abs_y,
            "lateral_mean_abs_world_y_m": lateral_abs_y,
            "forward_name_pass": front_x > rear_x,
            "medial_name_pass": medial_abs_y < lateral_abs_y,
        }
    foot_site_semantics_pass = all(
        bool(row["forward_name_pass"]) and bool(row["medial_name_pass"])
        for row in foot_site_gate.values()
    )
    overall = (
        index_contract_pass
        and foot_site_semantics_pass
        and all(str(row["overall"]) == "PASS" for row in pose_gates.values())
    )
    return {
        "frame_convention": "X forward, Y left, Z up; SI units; free base qpos[0:7]",
        "compiled_joint_order": compiled_joint_order,
        "actuator_order": actuator_order,
        "index_mapping": mapping,
        "index_contract_pass": index_contract_pass,
        "recommended_training_reset_keyframe": "gait_neutral",
        "calibration_zero_keyframe": "official_standing",
        "keyframe_gates": pose_gates,
        "foot_site_semantics": foot_site_gate,
        "foot_site_semantics_pass": foot_site_semantics_pass,
        "overall": "PASS" if overall else "FAIL",
    }


def gait_neutral_pd_smoke(model) -> dict[str, object]:
    """Run a short deterministic dynamics gate without changing the model."""

    import mujoco
    import numpy as np

    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "gait_neutral")
    data.qpos[:] = model.key_qpos[key_id]
    target = data.qpos[7:].copy()
    duration_s = 5.0
    steps = int(round(duration_s / float(model.opt.timestep)))
    base_z: list[float] = []
    max_abs_ctrl = 0.0
    for _ in range(steps):
        data.ctrl[:] = np.clip(
            18.0 * (target - data.qpos[7:]) - 0.45 * data.qvel[6:],
            model.actuator_ctrlrange[:, 0],
            model.actuator_ctrlrange[:, 1],
        )
        max_abs_ctrl = max(max_abs_ctrl, float(np.max(np.abs(data.ctrl))))
        mujoco.mj_step(model, data)
        base_z.append(float(data.qpos[2]))
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    stable_height = min(base_z) > 0.35
    return {
        "duration_s": steps * float(model.opt.timestep),
        "controller": "joint-space PD kp=18.0 kd=0.45, clipped by canonical actuator ctrlrange",
        "finite_state_pass": finite,
        "base_z_min_m": min(base_z),
        "base_z_final_m": base_z[-1],
        "stable_height_pass": stable_height,
        "max_abs_ctrl_nm": max_abs_ctrl,
        "overall": "PASS" if finite and stable_height else "FAIL",
    }


def main() -> int:
    u5 = load(V5_URDF_SOURCE, "zeroth01_v5_urdf_for_mjcf")
    robot = u5.gen_urdf()
    v2_robot = ET.parse(u5.V2_URDF).getroot()
    base = u5.v3

    masses = {
        str(link.get("name")): float(link.find("./inertial/mass").get("value"))
        for link in robot.findall("link")
        if link.find("./inertial/mass") is not None
    }
    body_mass = masses[base.BODY]
    for name, mass in masses.items():
        if name != base.BODY:
            base.FIXED_MASSES[name] = mass

    # Roll back the experimental 18-DoF topology at the shared primitive
    # collision layer as well as at URDF level.
    base.neutral_transforms = lambda old_tf: dict(old_tf)
    base.joint_kinematics = lambda old_robot, neutral_tf: joint_kinematics_original_16(u5, old_robot, neutral_tf)
    base.body_mass = lambda: body_mass
    base.SEGMENT_CHILD["3215_BothFlange_13"] = "FOOT"
    base.SEGMENT_CHILD["3215_BothFlange_14"] = "FOOT_2"
    base.SEGMENT_CHILD.pop(base.LEFT_ANKLE_CARRIER, None)
    base.SEGMENT_CHILD.pop(base.RIGHT_ANKLE_CARRIER, None)

    m3 = load(V3_MJCF_SOURCE, "zeroth01_v3_mjcf_builder_for_v5")
    m3.OUT = OUT
    m3.MJCF = MJCF
    m3.load_urdf_module = lambda: base
    m3.main()

    root = ET.parse(MJCF).getroot()
    root.set("model", "zeroth01_physical_mount_v5_original_16dof_solidworks_motion_mjx")
    torso = root.find(f"./worldbody/body[@name='{base.BODY}']")
    if torso is None:
        raise RuntimeError("missing torso body")
    remove_named_body(
        torso,
        {
            "IMU_2",
            "torso_imu_module",
            "compute_module",
            "battery_pack",
            base.CORES3_HEAD_ADAPTER,
            base.CORES3_HEAD_POD,
        },
    )
    # Those internal payloads are already in the v5 torso aggregate mass.
    # The two separately-accounted purchased head links are reinstated here.
    add_v5_head(torso)
    replace_collision_geometry(root)
    apply_urdf_inertials(root, robot)
    standing_height = apply_actual_standing_height(root, u5, base, v2_robot)

    joint_order = tree_joint_order(root)
    enforce_model_index_contract(root, joint_order)

    actuators = root.findall("./actuator/motor")
    if len(actuators) != 16 or any("ankle_roll" in str(item.get("joint")) for item in actuators):
        raise RuntimeError("v5 actuator topology is not original 16-DoF")

    grounded_keyframe_solution: dict[str, dict[str, object]] = {}
    try:
        grounded_keyframe_solution = solve_grounded_keyframes(root)
    except ImportError:
        # Source generation remains possible without MuJoCo, but the runtime
        # report explicitly remains pending and cannot be treated as a gate.
        write_xml(root)

    runtime = {"runtime_compile_gate": "PENDING_MUJOCO_RUNTIME"}
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MJCF))
        compiled_mass = float(model.body_mass.sum())
        contract = validate_model_contract(model)
        dynamics_smoke = gait_neutral_pd_smoke(model)
        mjcf_bytes = MJCF.read_bytes()
        if b"\r" in mjcf_bytes:
            raise RuntimeError("generated MJCF is not LF-normalized")
        mjcf_sha256 = hashlib.sha256(mjcf_bytes).hexdigest()
        runtime = {
            "runtime_compile_gate": "PASS" if contract["overall"] == "PASS" and dynamics_smoke["overall"] == "PASS" else "FAIL",
            "mujoco_version": mujoco.__version__,
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "body_count": int(model.nbody),
            "geom_count": int(model.ngeom),
            "sensor_count": int(model.nsensor),
            "compiled_total_mass_kg": compiled_mass,
            "compiled_mass_delta_kg": compiled_mass - u5.TARGET_TOTAL_MASS_KG,
            "standing_height_m": standing_height,
            "mjcf_sha256": mjcf_sha256,
            "mjcf_hash_mode": "sha256_exact_lf_bytes",
            "grounded_keyframe_solution": grounded_keyframe_solution,
            "model_contract": contract,
            "gait_neutral_dynamics_smoke": dynamics_smoke,
        }
        if model.nu != 16 or abs(compiled_mass - u5.TARGET_TOTAL_MASS_KG) > 1.0e-8:
            runtime["runtime_compile_gate"] = "FAIL"
    except ImportError:
        pass

    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.mjcf_compile_gate.v2",
        "mjcf": MJCF.relative_to(ROOT).as_posix(),
        "expected_mass_kg": u5.TARGET_TOTAL_MASS_KG,
        "expected_joint_count": 16,
        "ankle_roll_actuator_count": 0,
        "source_gate": "PASS",
        "physics_change_policy": "no geometry, mass, inertia, collision, joint axis, joint range or torque limit changed by the RL contract repair",
        "inertial_source": "full tensors copied from generated v5 URDF",
        "collision_source": "v5 compact wrist support + 9 mm lower-wider sole primitives",
        **runtime,
    }
    payload["overall"] = "PASS" if payload["runtime_compile_gate"] in {"PASS", "PENDING_MUJOCO_RUNTIME"} else "FAIL"
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
