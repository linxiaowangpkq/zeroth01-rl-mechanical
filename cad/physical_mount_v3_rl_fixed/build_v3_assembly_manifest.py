"""Build the external-part assembly manifest used by CAD review and SolidWorks.

The full source-carrier assembly remains external-part based so the original
v2/v1 B-Reps are not re-serialized into one multi-million-facet STEP file.
"""

from __future__ import annotations

import importlib.util
import json
import math
import struct
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed" / "ZEROTH01_V3_RL_FIXED_18DOF_FULL_ASSEMBLY_MANIFEST.json"
V1_SKELETON = ROOT / "generated" / "cad" / "physical_mount_v1" / "step" / "skeleton"
V2_PARTS = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
V2_REPLACEMENTS = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements"
V2_MANIFEST = ROOT / "reports" / "physical_mount_v2_minimal" / "component_manifest.json"
V3_PARTS = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed" / "parts"
ACTUATOR_LAYOUT = ROOT / "generated" / "config" / "physical_mount_v3_rl_fixed_actuator_layout.json"
V2_SERVO_MESHES = ROOT / "generated" / "urdf" / "physical_mount_v2_minimal" / "meshes" / "servos"
STS3250_LOCAL_BBOX_CENTER_MM = np.array((-10.11, 0.0, -20.75), dtype=float)

WHITE = "#F7F8FA"
BLUE = "#1677FF"
BLACK = "#101820"


def binary_stl_bounds(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read bounds without pulling the visualization-only trimesh package."""

    payload = path.read_bytes()
    if len(payload) < 84:
        raise ValueError(f"STL is too short: {path}")
    triangle_count = struct.unpack_from("<I", payload, 80)[0]
    if len(payload) != 84 + 50 * triangle_count:
        raise ValueError(f"only binary STL is supported: {path}")
    minimum = np.full(3, np.inf, dtype=float)
    maximum = np.full(3, -np.inf, dtype=float)
    offset = 84
    for _ in range(triangle_count):
        values = struct.unpack_from("<12fH", payload, offset)
        vertices = np.asarray(values[3:12], dtype=float).reshape(3, 3)
        minimum = np.minimum(minimum, vertices.min(axis=0))
        maximum = np.maximum(maximum, vertices.max(axis=0))
        offset += 50
    return minimum, maximum


def load_module(filename, name):
    path = Path(__file__).with_name(filename)
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def matrix4(rotation, translation_mm):
    return [
        [rotation[0][0], rotation[0][1], rotation[0][2], translation_mm[0]],
        [rotation[1][0], rotation[1][1], rotation[1][2], translation_mm[1]],
        [rotation[2][0], rotation[2][1], rotation[2][2], translation_mm[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def unit(values):
    length = math.sqrt(sum(value * value for value in values))
    return tuple(value / length for value in values)


def axis_rotation(axis):
    z_axis = unit(axis)
    helper = (1.0, 0.0, 0.0) if abs(z_axis[0]) < 0.9 else (0.0, 1.0, 0.0)
    x_axis = unit(cross(helper, z_axis))
    y_axis = cross(z_axis, x_axis)
    return tuple((x_axis[row], y_axis[row], z_axis[row]) for row in range(3))


def ankle_servo_rotation(axis):
    """Mirror cases: local +X is outboard and local +Y is world up."""

    z_axis = unit(axis)
    axis_sign = 1.0 if z_axis[0] >= 0.0 else -1.0
    x_axis = (0.0, axis_sign, 0.0)
    y_axis = cross(z_axis, x_axis)
    return tuple((x_axis[row], y_axis[row], z_axis[row]) for row in range(3))


def fitted_servo_rotations(old_robot, old_tf):
    """Inherit the full installed orientation of the 16 released servos.

    The v2 STL is already expressed in its owner-link frame.  Its centroid
    offset from the joint shaft selects one of the eight proper signed-axis
    rotations for the dimension-controlled STS3250.  This preserves rotation
    about the shaft, which axis-only alignment lost.
    """

    visuals = {}
    for link in old_robot.findall("link"):
        owner = str(link.get("name"))
        for visual in link.findall("visual"):
            name = str(visual.get("name", ""))
            if not name.endswith("_blue_servo_visual"):
                continue
            joint = name.split("_", 1)[1].removesuffix("_blue_servo_visual")
            mesh = visual.find("./geometry/mesh")
            visuals[joint] = (owner, V2_SERVO_MESHES / Path(str(mesh.get("filename"))).name)

    u = load_module("build_v3_urdf.py", "zeroth_v3_fit")
    old_joints = {str(joint.get("name")): joint for joint in old_robot.findall("joint")}
    result = {}
    for joint_name, _, child, _, _ in u.JOINT_SPECS:
        if joint_name not in visuals:
            continue
        old_joint = old_joints[joint_name]
        old_parent = str(old_joint.find("parent").get("link"))
        origin = old_joint.find("origin")
        joint_rotation = np.asarray(
            u.tf_mul(
                old_tf[old_parent],
                (
                    u.rpy_matrix(u.vec(origin.get("rpy") if origin is not None else None)),
                    u.vec(origin.get("xyz") if origin is not None else None),
                ),
            )[0],
            dtype=float,
        )
        axis_local = np.asarray(u.vec(old_joint.find("axis").get("xyz")), dtype=float)
        axis_vector = joint_rotation @ axis_local
        axis_vector = axis_vector / np.linalg.norm(axis_vector)
        owner, mesh_path = visuals[joint_name]
        mesh_minimum, mesh_maximum = binary_stl_bounds(mesh_path)
        local_center_m = (mesh_minimum + mesh_maximum) / 2.0
        owner_rotation = np.asarray(old_tf[owner][0], dtype=float)
        owner_position = np.asarray(old_tf[owner][1], dtype=float)
        source_center_m = owner_rotation @ local_center_m + owner_position
        old_child = str(old_joint.find("child").get("link"))
        shaft_center_m = np.asarray(old_tf[old_child][1], dtype=float)
        source_offset_mm = (source_center_m - shaft_center_m) * 1000.0

        candidates = []
        for sign in (1.0, -1.0):
            z_axis = sign * axis_vector
            helpers = [np.asarray(value, dtype=float) for value in np.eye(3)]
            helper = min(helpers, key=lambda value: abs(float(np.dot(value, z_axis))))
            base_x = helper - np.dot(helper, z_axis) * z_axis
            base_x = base_x / np.linalg.norm(base_x)
            base_y = np.cross(z_axis, base_x)
            for x_axis in (base_x, -base_x, base_y, -base_y):
                y_axis = np.cross(z_axis, x_axis)
                rotation = np.column_stack((x_axis, y_axis, z_axis))
                residual = float(np.linalg.norm(rotation @ STS3250_LOCAL_BBOX_CENTER_MM - source_offset_mm))
                candidates.append((residual, rotation))
        residual, rotation = min(candidates, key=lambda item: item[0])
        result[joint_name] = (tuple(tuple(float(value) for value in row) for row in rotation), residual)
    return result


def row(root, component_id, role, source, transform, color, owner, notes=""):
    source = Path(source)
    if not source.is_file():
        raise FileNotFoundError(source)
    return {
        "component_id": component_id,
        "role": role,
        "source": source.relative_to(root).as_posix(),
        "owner_link": owner,
        "transform_local_mm_to_world_mm": transform,
        "color_hex": color,
        "notes": notes,
    }


def main() -> int:
    u = load_module("build_v3_urdf.py", "zeroth_v3_manifest_urdf")
    old_robot = ET.parse(u.V2_URDF).getroot()
    old_tf = u.old_fk(old_robot)
    fitted_servo_pose = fitted_servo_rotations(old_robot, old_tf)
    # Mechanical v3 is a minimal extension of the released v2 assembly. Keep
    # every original 6D link transform exactly; numerical re-symmetrisation is
    # not allowed to move a real carrier away from its installed servo.
    neutral_tf = u.neutral_transforms(old_tf)
    positions = {name: transform[1] for name, transform in neutral_tf.items()}
    pos_mm = {name: tuple(value * 1000.0 for value in xyz) for name, xyz in positions.items()}
    components = []
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))

    carrier_links = [u.BODY]
    carrier_links.extend(spec[2] for spec in u.JOINT_SPECS if spec[2] not in {u.LEFT_ANKLE_CARRIER, u.RIGHT_ANKLE_CARRIER})
    for link_name in dict.fromkeys(carrier_links):
        if link_name in {"FINGER_1", "FINGER_1_2"}:
            continue
        if link_name == "R_ARM_MIRROR_1":
            source = V2_REPLACEMENTS / "R_ARM_MIRROR_1_WRIST_TRIMMED.step"
        elif link_name == "L_ARM_MIRROR_1":
            source = V2_REPLACEMENTS / "L_ARM_MIRROR_1_WRIST_TRIMMED.step"
        elif link_name == u.BODY:
            source = V3_PARTS / "body_skeleton_top_trimmed_45mm.step"
        else:
            source = V1_SKELETON / f"{link_name}.step"
        components.append(row(
            ROOT,
            f"CARRIER_{link_name}",
            "source_load_bearing_carrier",
            source,
            matrix4(old_tf[link_name][0], pos_mm[link_name]),
            WHITE,
            link_name,
            (
                "source-derived body with only the obsolete upper head plate removed above Z=45 mm; released shoulder/hip interfaces and complete neutral 6D transform retained"
                if link_name == u.BODY
                else "v2/v1 released carrier; complete neutral 6D transform retained without numerical re-symmetrisation"
            ),
        ))

    # Q-hands replace the complete old claw links.
    for component_id, link_name, filename in (
        ("LEFT_Q_HAND", "FINGER_1", "left_q_hand.step"),
        ("RIGHT_Q_HAND", "FINGER_1_2", "right_q_hand.step"),
    ):
        components.append(row(ROOT, component_id, "fixed_q_hand", V2_PARTS / filename, matrix4(old_tf[link_name][0], pos_mm[link_name]), WHITE, link_name, "old claw is not present"))

    # v2 cosmetic/electronic parts stay attached to their source owner.  The
    # old dense soles are replaced below.
    v2_rows = json.loads(V2_MANIFEST.read_text(encoding="utf-8"))["parts"]
    replaced_head_keys = {
        "head_front",
        "head_back",
        "visor",
        "face_ui",
        "camera_window",
        "camera_bracket",
        "display_module",
        "camera_module",
        "tof_module",
    }
    removed_service_volume_keys = replaced_head_keys | {"chest_panel"}
    for item in v2_rows:
        key = str(item["key"])
        if key in {"left_q_hand", "right_q_hand", "left_sole", "right_sole"} | removed_service_volume_keys:
            continue
        owner = str(item["installed_link"])
        components.append(row(
            ROOT,
            f"V2_{key.upper()}",
            str(item["classification"]),
            V2_PARTS / f"{key}.step",
            matrix4(old_tf[owner][0], pos_mm[owner]),
            str(item["color_hex"]),
            owner,
        ))

    components.append(row(
        ROOT,
        "M5STACK_CORES3_K128_PURCHASED_HEAD_MODULE",
        "purchased_interaction_head_module",
        V3_PARTS / "m5stack_cores3_k128_purchased_envelope.step",
        matrix4(identity, tuple(value * 1000.0 for value in u.CORES3_HEAD_CENTER_M)),
        WHITE,
        u.CORES3_HEAD_POD,
        "off-the-shelf SKU K128 CoreS3 main unit, official 54 x 54 x 15.5 mm; the full 72.7 g retail-set mass is conservatively assigned here; camera, dual microphones, 1 W speaker, touch display and IMU included",
    ))
    components.append(row(
        ROOT,
        "CORES3_INTERNAL_TORSO_CRADLE_2MM_6061",
        "reversible_purchased_head_torso_adapter",
        V3_PARTS / "cores3_internal_torso_cradle_2mm_6061.step",
        matrix4(identity, tuple(value * 1000.0 for value in u.CORES3_ADAPTER_CENTER_M)),
        "#BFC7D1",
        u.CORES3_HEAD_ADAPTER,
        "hidden 2 mm 6061 U-cradle with four M3 torso-side holes; perimeter lips retain CoreS3; first-article slot/nut-plate fit is still required",
    ))
    for component_id, role, filename, feature_color, notes in (
        (
            "CORES3_FACE_GLASS",
            "purchased_head_face_reference",
            "m5stack_cores3_face_glass_reference.step",
            BLACK,
            "front glass/display feature of the purchased module",
        ),
        (
            "CORES3_CAMERA_WINDOW",
            "purchased_head_sensor_window_reference",
            "m5stack_cores3_camera_reference.step",
            "#00B8D9",
            "GC0308 camera location; optical frame is published in URDF",
        ),
        (
            "CORES3_SCREEN_EXPRESSION",
            "purchased_head_screen_ui_reference",
            "m5stack_cores3_expression_reference.step",
            "#00B8D9",
            "replaceable screen pixels, not a manufactured solid",
        ),
    ):
        components.append(row(
            ROOT,
            component_id,
            role,
            V3_PARTS / filename,
            matrix4(identity, tuple(value * 1000.0 for value in u.CORES3_HEAD_CENTER_M)),
            feature_color,
            u.CORES3_HEAD_POD,
            notes,
        ))

    # The 18 identical blue dimension-controlled references are separate
    # components at their shaft frames.
    actuator_ids = {
        str(item["joint"]): str(item["id"])
        for item in json.loads(ACTUATOR_LAYOUT.read_text(encoding="utf-8"))["actuators"]
    }
    source_servo_owner = {}
    for link in old_robot.findall("link"):
        for visual in link.findall("visual"):
            name = str(visual.get("name", ""))
            if name.endswith("_blue_servo_visual"):
                source_servo_owner[
                    name.split("_", 1)[1].removesuffix("_blue_servo_visual")
                ] = str(link.get("name"))
    for joint_name, parent, child, axis, _ in u.JOINT_SPECS:
        servo_id = actuator_ids[joint_name]
        if joint_name in fitted_servo_pose:
            servo_rotation, fit_residual_mm = fitted_servo_pose[joint_name]
            pose_note = f"full 6D installation inherited from released v2 servo; centroid-fit residual {fit_residual_mm:.3f} mm"
        else:
            servo_rotation = ankle_servo_rotation(axis)
            fit_residual_mm = 0.0
            pose_note = "new mirrored 30 mm ankle-roll installation; local +Z output axis maps to body +/-X, local +Y is world up and the 45.22 mm case direction is lateral"
        owner_link = source_servo_owner.get(joint_name, parent)
        components.append(row(
            ROOT,
            f"{servo_id}_STS3250_{joint_name}",
            "dimension_controlled_sts3250",
            V3_PARTS / "sts3250_dimension_controlled.step",
            matrix4(servo_rotation, pos_mm[child]),
            BLUE,
            owner_link,
            f"shaft origin coincides with URDF joint frame; {pose_note}",
        ))

    for side, carrier, foot, axis in (
        ("left", u.LEFT_ANKLE_CARRIER, "FOOT", (1.0, 0.0, 0.0)),
        ("right", u.RIGHT_ANKLE_CARRIER, "FOOT_2", (-1.0, 0.0, 0.0)),
    ):
        roll_rotation = ankle_servo_rotation(axis)
        components.append(row(ROOT, f"{side.upper()}_ANKLE_ROLL_CARRIER", "new_ankle_roll_parent_carrier", V3_PARTS / f"{side}_ankle_roll_carrier.step", matrix4(roll_rotation, pos_mm[foot]), WHITE, carrier, "cage surrounds roll servo; upper anchor reaches released ankle-pitch output"))
        components.append(row(ROOT, f"{side.upper()}_ANKLE_ROLL_HORN", "new_ankle_roll_child_horn_adapter", V3_PARTS / f"{side}_ankle_roll_horn_adapter.step", matrix4(roll_rotation, pos_mm[foot]), BLUE, foot))
        components.append(row(ROOT, f"{side.upper()}_7MM_LIGHTWEIGHT_SOLE", "replaceable_7mm_perimeter_rib_sole", V3_PARTS / f"{side}_sole_lightweighted.step", matrix4(old_tf[foot][0], pos_mm[foot]), BLACK, foot))

    payload = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.external_part_assembly.v1",
        "units": "mm",
        "frame": "X forward, Y left, Z up",
        "component_count": len(components),
        "movable_joint_count": len(u.JOINT_SPECS),
        "blue_sts3250_count": sum(item["role"] == "dimension_controlled_sts3250" for item in components),
        "old_claw_count": 0,
        "purchased_head_module": "M5Stack CoreS3 K128",
        "removed_custom_head_component_count": len(replaced_head_keys),
        "removed_interfering_chest_panel_count": 1,
        "removed_obsolete_upper_head_plate_count": 1,
        "components": components,
        "truth_boundary": "source carrier installation geometry + controlled STS3250 envelope + official-size purchased CoreS3 reference; purchased STS3250/cradle first article and as-built mass properties remain HOLD",
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(OUT)
    print(json.dumps({key: payload[key] for key in ("component_count", "movable_joint_count", "blue_sts3250_count", "old_claw_count")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
