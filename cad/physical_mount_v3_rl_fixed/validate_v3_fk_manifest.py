"""Validate that URDF neutral frames and the SolidWorks manifest are identical."""

from __future__ import annotations

import importlib.util
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = Path(__file__).with_name("build_v3_urdf.py")
URDF = (
    ROOT
    / "generated"
    / "urdf"
    / "physical_mount_v3_rl_fixed"
    / "zeroth01_physical_mount_v3_rl_fixed_18dof.urdf"
)
MANIFEST = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v3_rl_fixed"
    / "ZEROTH01_V3_RL_FIXED_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)
REPORT = (
    ROOT
    / "reports"
    / "physical_mount_v3_rl_fixed"
    / "fk_manifest_gate.json"
)


def load_module():
    spec = importlib.util.spec_from_file_location("zeroth_v3_fk_gate", SOURCE)
    if spec is None or spec.loader is None:
        raise RuntimeError(SOURCE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def max_matrix_delta(a, b):
    return max(abs(a[row][column] - b[row][column]) for row in range(3) for column in range(3))


def norm(values):
    return math.sqrt(sum(value * value for value in values))


def manifest_transform(row):
    matrix = row["transform_local_mm_to_world_mm"]
    return (
        tuple(tuple(float(matrix[i][j]) for j in range(3)) for i in range(3)),
        tuple(float(matrix[i][3]) / 1000.0 for i in range(3)),
    )


def joint_values(joint):
    origin = joint.find("origin")
    axis = joint.find("axis")
    limit = joint.find("limit")
    return {
        "xyz": tuple(float(value) for value in origin.get("xyz", "0 0 0").split()),
        "rpy": tuple(float(value) for value in origin.get("rpy", "0 0 0").split()),
        "axis": tuple(float(value) for value in axis.get("xyz", "0 0 0").split()),
        "limit": (
            float(limit.get("lower", "0")),
            float(limit.get("upper", "0")),
        ),
    }


def main() -> int:
    u = load_module()
    v2_robot = ET.parse(u.V2_URDF).getroot()
    v3_robot = ET.parse(URDF).getroot()
    v2_tf = u.old_fk(v2_robot)
    expected_tf = u.neutral_transforms(v2_tf)
    actual_tf = u.old_fk(v3_robot)

    frame_rows = []
    frame_names = set(u.COLLISION) | {
        "FINGER_1",
        "FINGER_1_2",
        "IMU_2",
        "torso_imu_module",
        "compute_module",
        "battery_pack",
    }
    for name in sorted(frame_names):
        expected_rotation, expected_position = expected_tf[name]
        actual_rotation, actual_position = actual_tf[name]
        frame_rows.append(
            {
                "link": name,
                "translation_residual_mm": norm(u.sub(actual_position, expected_position)) * 1000.0,
                "rotation_max_abs": max_matrix_delta(actual_rotation, expected_rotation),
            }
        )

    v2_joints = {str(joint.get("name")): joint for joint in v2_robot.findall("joint")}
    v3_joints = {str(joint.get("name")): joint for joint in v3_robot.findall("joint")}
    original_joint_rows = []
    for name, _, _, _, _ in u.JOINT_SPECS:
        if name.endswith("ankle_roll"):
            continue
        source = joint_values(v2_joints[name])
        target = joint_values(v3_joints[name])
        translation_residual = max(
            abs(source["xyz"][index] - target["xyz"][index])
            for index in range(3)
        )
        axis_residual = max(
            abs(source["axis"][index] - target["axis"][index])
            for index in range(3)
        )
        limit_residual = max(
            abs(source["limit"][index] - target["limit"][index])
            for index in range(2)
        )
        rotation_residual = max_matrix_delta(
            u.rpy_matrix(source["rpy"]), u.rpy_matrix(target["rpy"])
        )
        residual = max(
            translation_residual,
            axis_residual,
            rotation_residual,
            limit_residual,
        )
        original_joint_rows.append(
            {
                "joint": name,
                "max_numeric_residual": residual,
                "child": str(v3_joints[name].find("child").get("link")),
            }
        )

    roll_rows = []
    for name, carrier, foot, desired_axis in (
        ("left_ankle_roll", u.LEFT_ANKLE_CARRIER, "FOOT", (1.0, 0.0, 0.0)),
        ("right_ankle_roll", u.RIGHT_ANKLE_CARRIER, "FOOT_2", (-1.0, 0.0, 0.0)),
    ):
        joint = v3_joints[name]
        values = joint_values(joint)
        origin_rotation = u.rpy_matrix(values["rpy"])
        joint_world_rotation = u.mat_mul(actual_tf[carrier][0], origin_rotation)
        axis_world = u.mat_vec(joint_world_rotation, values["axis"])
        world_delta = u.sub(actual_tf[foot][1], actual_tf[carrier][1])
        roll_rows.append(
            {
                "joint": name,
                "parent": str(joint.find("parent").get("link")),
                "child": str(joint.find("child").get("link")),
                "vertical_offset_mm": world_delta[2] * 1000.0,
                "lateral_offset_mm": math.hypot(world_delta[0], world_delta[1]) * 1000.0,
                "world_axis_residual": norm(u.sub(axis_world, desired_axis)),
            }
        )

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    manifest_by_id = {row["component_id"]: row for row in manifest["components"]}
    manifest_rows = []
    for component_id, link in (
        ("CARRIER_Z_BOT2_MASTER_BODY_SKELETON", u.BODY),
        ("CARRIER_3215_BothFlange_13", "3215_BothFlange_13"),
        ("CARRIER_3215_BothFlange_14", "3215_BothFlange_14"),
        ("LEFT_7MM_LIGHTWEIGHT_SOLE", "FOOT"),
        ("RIGHT_7MM_LIGHTWEIGHT_SOLE", "FOOT_2"),
    ):
        component_tf = manifest_transform(manifest_by_id[component_id])
        link_tf = actual_tf[link]
        manifest_rows.append(
            {
                "component": component_id,
                "link": link,
                "translation_residual_mm": norm(u.sub(component_tf[1], link_tf[1])) * 1000.0,
                "rotation_max_abs": max_matrix_delta(component_tf[0], link_tf[0]),
            }
        )

    servo_visuals = {}
    for link in v3_robot.findall("link"):
        link_name = str(link.get("name"))
        for visual in link.findall("visual"):
            visual_name = str(visual.get("name", ""))
            if visual_name.startswith("S") and "_blue_servo_visual" in visual_name:
                servo_id = visual_name.split("_", 1)[0]
            elif visual_name == "left_ankle_roll_blue_servo_visual":
                servo_id = "S18"
            elif visual_name == "right_ankle_roll_blue_servo_visual":
                servo_id = "S17"
            else:
                continue
            origin = visual.find("origin")
            local_tf = (
                u.rpy_matrix(u.vec(origin.get("rpy") if origin is not None else None)),
                u.vec(origin.get("xyz") if origin is not None else None),
            )
            mesh = visual.find("./geometry/mesh")
            servo_visuals[servo_id] = {
                "owner_link": link_name,
                "world_tf": u.tf_mul(actual_tf[link_name], local_tf),
                "mesh": str(mesh.get("filename")) if mesh is not None else "",
            }

    servo_manifest_rows = []
    manifest_servos = {
        str(row["component_id"]).split("_", 1)[0]: row
        for row in manifest["components"]
        if row.get("role") == "dimension_controlled_sts3250"
    }
    for servo_id in sorted(manifest_servos):
        component = manifest_servos[servo_id]
        visual = servo_visuals.get(servo_id)
        if visual is None:
            servo_manifest_rows.append(
                {
                    "id": servo_id,
                    "owner_link": None,
                    "translation_residual_mm": float("inf"),
                    "rotation_max_abs": float("inf"),
                    "mesh": None,
                    "gate": "FAIL_MISSING_VISUAL",
                }
            )
            continue
        component_tf = manifest_transform(component)
        visual_tf = visual["world_tf"]
        translation_residual_mm = norm(u.sub(component_tf[1], visual_tf[1])) * 1000.0
        rotation_residual = max_matrix_delta(component_tf[0], visual_tf[0])
        mesh_ok = visual["mesh"] == "meshes/v3/sts3250_dimension_controlled.stl"
        owner_ok = visual["owner_link"] == component["owner_link"]
        servo_manifest_rows.append(
            {
                "id": servo_id,
                "owner_link": visual["owner_link"],
                "translation_residual_mm": translation_residual_mm,
                "rotation_max_abs": rotation_residual,
                "mesh": visual["mesh"],
                "gate": (
                    "PASS"
                    if translation_residual_mm <= 1.0e-5
                    and rotation_residual <= 1.0e-8
                    and mesh_ok
                    and owner_ok
                    else "FAIL"
                ),
            }
        )

    head_parent = str(v3_joints["cores3_head_module_fixed_joint"].find("parent").get("link"))
    masses = [float(item.get("value")) for item in v3_robot.findall("./link/inertial/mass")]
    failures = []
    if any(row["translation_residual_mm"] > 1.0e-5 or row["rotation_max_abs"] > 1.0e-8 for row in frame_rows):
        failures.append("neutral_link_frame_mismatch")
    if any(row["max_numeric_residual"] > 1.0e-9 for row in original_joint_rows):
        failures.append("released_v2_joint_frame_changed")
    if any(
        abs(row["vertical_offset_mm"] + 1000.0 * u.ANKLE_ROLL_OFFSET_M) > 1.0e-5
        or row["lateral_offset_mm"] > 1.0e-5
        or row["world_axis_residual"] > 1.0e-8
        for row in roll_rows
    ):
        failures.append("ankle_roll_geometry_mismatch")
    if any(row["translation_residual_mm"] > 1.0e-5 or row["rotation_max_abs"] > 1.0e-8 for row in manifest_rows):
        failures.append("solidworks_manifest_frame_mismatch")
    if len(servo_visuals) != 18 or any(row["gate"] != "PASS" for row in servo_manifest_rows):
        failures.append("urdf_sts3250_visual_manifest_mismatch")
    if head_parent != u.CORES3_HEAD_ADAPTER:
        failures.append("head_bypasses_adapter")
    if abs(sum(masses) - u.TARGET_TOTAL_MASS_KG) > 1.0e-9:
        failures.append("mass_mismatch")

    payload = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.fk_manifest_gate.v1",
        "urdf": str(URDF),
        "manifest": str(MANIFEST),
        "movable_joint_count": sum(joint.get("type") == "revolute" for joint in v3_robot.findall("joint")),
        "total_mass_kg": sum(masses),
        "head_parent_link": head_parent,
        "neutral_frame_rows": frame_rows,
        "released_v2_joint_rows": original_joint_rows,
        "ankle_roll_rows": roll_rows,
        "manifest_frame_rows": manifest_rows,
        "sts3250_visual_manifest_rows": servo_manifest_rows,
        "failures": failures,
        "overall": "PASS" if not failures else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
