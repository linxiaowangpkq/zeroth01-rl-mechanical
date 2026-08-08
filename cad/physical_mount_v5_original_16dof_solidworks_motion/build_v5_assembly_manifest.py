"""Build the v5 full-body external-part assembly manifest."""

from __future__ import annotations

import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
V3_DIR = ROOT / "cad" / "physical_mount_v3_rl_fixed"
V5_PARTS = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "parts"
V5_SOLID_CARRIERS = V5_PARTS / "source_carriers_solid"
V5_CLEARANCED_CARRIERS = V5_PARTS / "source_carriers_sts3250_clearanced"
V5_WRIST_MOUNT_CARRIERS = V5_PARTS / "source_carriers_sts3250_wrist_mount"
V2_URDF = ROOT / "generated" / "urdf" / "physical_mount_v2_minimal" / "zeroth01_physical_mount_v2_minimal.urdf"
SERVO_MANIFEST = ROOT / "reports" / "physical_mount_v1" / "servo_component_manifest.json"
ACTUATORS_V1 = ROOT / "generated" / "config" / "physical_mount_v1_actuators.json"
OUT = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"

WHITE = "#F7F8FA"
BLUE = "#1677FF"
BLACK = "#101820"
CYAN = "#00B8D9"
ORANGE = "#FF9100"
MAGENTA = "#D500F9"
GREEN = "#64DD17"
GREY = "#BFC7D1"
STEEL = "#586069"
BODY = "Z_BOT2_MASTER_BODY_SKELETON"

CHILD_STANDOFF_MM = {
    "left_shoulder_yaw": 1.95,
    "right_shoulder_yaw": 1.95,
    "left_shoulder_pitch": 1.0,
    "right_shoulder_pitch": 1.0,
    "left_elbow_yaw": 3.0,
    "right_elbow_yaw": 3.0,
    "left_hip_yaw": 1.95,
    "right_hip_yaw": 1.95,
    "left_hip_roll": 1.0,
    "right_hip_roll": 1.0,
    "left_hip_pitch": 1.0,
    "right_hip_pitch": 1.0,
    "left_knee_pitch": 1.0,
    "right_knee_pitch": 1.0,
    "left_ankle_pitch": 12.95,
    "right_ankle_pitch": 12.95,
}
SERVO_AXIAL_SHIM_MM = {"left_hip_yaw": 4.0, "right_hip_yaw": 4.0}


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


u = load(V3_DIR / "build_v3_urdf.py", "zeroth_v3_urdf_helpers_for_v5")
m = load(V3_DIR / "build_v3_assembly_manifest.py", "zeroth_v3_manifest_helpers_for_v5")


def joint_specs(robot: ET.Element) -> list[dict[str, object]]:
    configured = json.loads(ACTUATORS_V1.read_text(encoding="utf-8"))["servos"]
    joints = {str(joint.get("name")): joint for joint in robot.findall("joint")}
    rows = []
    for actuator in configured:
        name = str(actuator["joint"])
        joint = joints[name]
        limit = joint.find("limit")
        rows.append(
            {
                "id": str(actuator["id"]),
                "name": name,
                "parent": str(joint.find("parent").get("link")),
                "child": str(joint.find("child").get("link")),
                "axis": tuple(float(value) for value in joint.find("axis").get("xyz").split()),
                "limits": (float(limit.get("lower")), float(limit.get("upper"))),
            }
        )
    if len(rows) != 16:
        raise RuntimeError(f"expected 16 configured servos, got {len(rows)}")
    return rows


def source_carrier(link_name: str) -> Path:
    if link_name == BODY:
        clearanced_body = V5_CLEARANCED_CARRIERS / "torso_mounted_solid.step"
        if clearanced_body.is_file():
            return clearanced_body
        solid_body = V5_SOLID_CARRIERS / f"{BODY}_SOLID.step"
        if not solid_body.is_file():
            raise FileNotFoundError(f"positive-volume fixed torso missing: {solid_body}")
        return solid_body
    mount_clearanced = V5_CLEARANCED_CARRIERS / f"{link_name}_MOUNT_CLEARANCED.step"
    if mount_clearanced.is_file():
        return mount_clearanced
    wrist_mount = V5_WRIST_MOUNT_CARRIERS / f"{link_name}_STS3250_WRIST_MOUNT.step"
    if wrist_mount.is_file():
        return wrist_mount
    clearanced = V5_CLEARANCED_CARRIERS / f"{link_name}_STS3250_CLEARANCED.step"
    if clearanced.is_file():
        return clearanced
    solid = V5_SOLID_CARRIERS / f"{link_name}_SOLID.step"
    if not solid.is_file():
        raise FileNotFoundError(f"motion-solid carrier missing for {link_name}: {solid}")
    return solid
def main() -> int:
    robot = ET.parse(V2_URDF).getroot()
    old_tf = u.old_fk(robot)
    pos_mm = {
        name: tuple(value * 1000.0 for value in transform[1])
        for name, transform in old_tf.items()
    }
    specs = joint_specs(robot)
    fitted = m.fitted_servo_rotations(robot, old_tf)
    components: list[dict[str, object]] = []

    carrier_links = [BODY]
    carrier_links.extend(str(spec[side]) for spec in specs for side in ("parent", "child"))
    for link_name in dict.fromkeys(carrier_links):
        components.append(
            m.row(
                ROOT,
                f"CARRIER_{link_name}",
                "source_load_bearing_carrier",
                source_carrier(link_name),
                m.matrix4(old_tf[link_name][0], pos_mm[link_name]),
                WHITE,
                link_name,
                (
                    "fixed released Zeroth-01 torso reference; intentionally not promoted to a moving body"
                    if link_name == BODY
                    else "released Zeroth-01 16DoF carrier sewn into a positive-volume motion solid at the exact neutral transform"
                ),
            )
        )

    for component_id, filename, finger, owner in (
        ("LEFT_FIXED_WRIST_SUPPORT", "left_fixed_wrist_support.step", "FINGER_1", "R_ARM_MIRROR_1"),
        ("RIGHT_FIXED_WRIST_SUPPORT", "right_fixed_wrist_support.step", "FINGER_1_2", "L_ARM_MIRROR_1"),
    ):
        components.append(
            m.row(
                ROOT,
                component_id,
                "fixed_compact_wrist_support",
                V5_PARTS / filename,
                m.matrix4(old_tf[finger][0], pos_mm[finger]),
                WHITE,
                owner,
                "small rounded support bumper outside the released forearm; two M3 through-bolts provide retention and anti-rotation; not an actuator and not a gripper",
            )
        )

    for component_id, filename, foot_owner in (
        ("LEFT_TAPERED_SOLE_FLARE", "left_tapered_sole_flare.step", "FOOT"),
        ("RIGHT_TAPERED_SOLE_FLARE", "right_tapered_sole_flare.step", "FOOT_2"),
    ):
        components.append(
            m.row(
                ROOT,
                component_id,
                "replaceable_white_tapered_lower_wider_sole",
                V5_PARTS / filename,
                m.matrix4(old_tf[foot_owner][0], pos_mm[foot_owner]),
                WHITE,
                foot_owner,
                "9.0 mm replaceable sole; top stays within released foot outline, ground face is 8 mm longer and wider; four M3 clearance holes; ground face matches the released RL contact plane",
            )
        )

    body_rotation = old_tf[BODY][0]
    body_tf = m.matrix4(body_rotation, pos_mm[BODY])
    for component_id, role, filename, color, note in (
        ("V5_HEAD_FRONT", "printable_head_front_shell", "head_front_5mm_each_side.step", WHITE, "+5 mm each side rounded two-piece head"),
        ("V5_HEAD_REAR", "printable_head_rear_shell", "head_rear_5mm_each_side.step", WHITE, "rear shell with cable service exit"),
        ("V5_HEAD_VISOR", "simple_camera_microphone_visor", "head_simple_visor.step", BLACK, "simple original-style black face"),
        ("V5_UNITV2", "purchased_internal_interaction_module", "m5stack_unitv2_purchased_envelope.step", CYAN, "48 x 18.5 x 24 mm camera/microphone module"),
        ("V5_UNITV2_CRADLE", "removable_internal_service_mount", "unitv2_removable_cradle.step", GREY, "removable M2.5 cradle"),
        ("V5_HEAD_TORSO_NUT_PLATE", "direct_head_torso_mount", "direct_head_torso_nut_plate.step", GREY, "four M3 direct mount; no neck"),
        ("V5_COMPUTE", "internal_payload_controlled_envelope", "compute_envelope.step", ORANGE, "compute reservation inside torso"),
        ("V5_COMPUTE_TRAY", "removable_internal_service_mount", "compute_removable_tray.step", GREY, "removable internal compute tray"),
        ("V5_BATTERY", "internal_payload_controlled_envelope", "battery_envelope.step", MAGENTA, "battery/BMS reservation inside torso"),
        ("V5_BATTERY_CAGE", "removable_internal_service_mount", "battery_service_cage.step", GREY, "internal battery cage"),
        ("V5_TORSO_IMU", "internal_payload_controlled_envelope", "torso_imu_envelope.step", GREEN, "IMU reservation"),
        ("V5_HARNESS_GUIDES", "harness_strain_relief", "harness_strain_relief_guides.step", GREY, "internal strain-relief guides"),
    ):
        components.append(m.row(ROOT, component_id, role, V5_PARTS / filename, body_tf, color, BODY, note))

    source_servo_owner: dict[str, str] = {}
    for link in robot.findall("link"):
        owner = str(link.get("name"))
        for visual in link.findall("visual"):
            name = str(visual.get("name", ""))
            if name.endswith("_blue_servo_visual"):
                source_servo_owner[name.split("_", 1)[1].removesuffix("_blue_servo_visual")] = owner

    for spec in specs:
        joint_name = str(spec["name"])
        servo_id = str(spec["id"])
        parent = str(spec["parent"])
        child = str(spec["child"])
        servo_position = pos_mm[child]
        servo_rotation, residual = fitted[joint_name]
        axial_shim = SERVO_AXIAL_SHIM_MM.get(joint_name, 0.0)
        installed_position = tuple(
            servo_position[index] + servo_rotation[index][2] * axial_shim
            for index in range(3)
        )
        housing_owner = source_servo_owner.get(joint_name, parent)
        output_owner = child if housing_owner == parent else parent
        installed_tf = m.matrix4(servo_rotation, installed_position)
        joint_tf = m.matrix4(servo_rotation, servo_position)
        components.append(
            m.row(
                ROOT,
                f"{servo_id}_STS3250_{joint_name}",
                "purchased_exact_sts3250",
                V5_PARTS / "sts3250_step_parts_exact_shaft_frame.step",
                installed_tf,
                BLUE,
                housing_owner,
                f"purchased-exact STEP; housing fixed to {housing_owner}; fitted 6D residual {residual:.3f} mm",
            )
        )
        components.append(
            m.row(
                ROOT,
                f"{servo_id}_PARENT_THRUST_RING_{joint_name}",
                "sts3250_parent_axis_and_thrust_mate",
                V5_PARTS / "sts3250_parent_thrust_ring_2mm.step",
                installed_tf,
                GREY,
                housing_owner,
                "parent-fixed annulus; 0.225 mm radial clearance around output bridge; supplies actual SolidWorks concentric and thrust-face mates",
            )
        )
        components.append(
            m.row(
                ROOT,
                f"{servo_id}_PCD14_OUTPUT_BRIDGE_{joint_name}",
                "sts3250_pcd14_output_bridge_to_child",
                V5_PARTS / "sts3250_pcd14_output_bridge_2p05mm.step",
                installed_tf,
                BLUE,
                output_owner,
                f"rotating side, locked to {output_owner}; four M3 on PCD14 and centre M3x6",
            )
        )
        # The released carrier already contains the structural PCD14 horn.
        # For 14 joints the 2.05 mm output bridge directly contacts that horn;
        # the old extra full-disc standoffs duplicated it and caused the large
        # blue/grey collisions seen by the user.  Only the two 4 mm hip-yaw
        # case offsets need a 1.95 mm four-sleeve gap spacer.
        if joint_name in SERVO_AXIAL_SHIM_MM:
            components.append(
                m.row(
                    ROOT,
                    f"{servo_id}_2XM2X8_CASE_SCREWS_{joint_name}",
                    "purchased_2xm2x8_case_fasteners",
                    V5_PARTS / "sts3250_hip_yaw_2xm2x8_case_screws.step",
                    joint_tf,
                    STEEL,
                    housing_owner,
                    "two real M2x8 shanks pass through the torso-integrated lateral-rib bosses and engage the official STS3250 negative-X rear-cover holes; low-profile heads remain service-accessible",
                )
            )
            components.append(
                m.row(
                    ROOT,
                    f"{servo_id}_PCD14_FOUR_SLEEVE_SPACER_{joint_name}",
                    "sts3250_pcd14_four_m3_spacer_sleeves_to_released_horn",
                    V5_PARTS / "sts3250_pcd14_four_sleeve_spacer_1p65mm_z0p30.step",
                    joint_tf,
                    BLUE,
                    output_owner,
                    "four standard OD4x1.65 metal PCD14 sleeves seat 0.30 mm above the joint datum and end at the 1.95 mm bridge face; exact STS face clearance is 0.05 mm; M3x8 screws are specified in the fastener map",
                )
            )

    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.assembly_manifest.v1",
        "component_count": len(components),
        "movable_joint_count": 16,
        "blue_sts3250_count": 16,
        "ankle_roll_actuator_count": 0,
        "source_shin_count": 2,
        "source_foot_count": 2,
        "tapered_lower_wider_sole_count": 2,
        "black_sole_count": 0,
        "claw_count": 0,
        "large_hand_count": 0,
        "fixed_wrist_support_count": 2,
        "redundant_full_disc_child_standoff_count": 0,
        "hip_yaw_four_sleeve_spacer_count": 2,
        "hip_yaw_case_fastener_bridge_count": 0,
        "hip_yaw_integrated_case_mount_count": 2,
        "hip_yaw_case_screw_set_count": 2,
        "motion_solid_carrier_count": 16,
        "fixed_reference_carrier_count": 1,
        "sts3250_catalog_id": "feetech_sts3250",
        "sts3250_catalog_sha256": "cf46f17da455e1f158114791bb31404c24d925e8a758bbd6189f8ee815a571bf",
        "joint_specs": specs,
        "servo_axial_shims_mm": SERVO_AXIAL_SHIM_MM,
        "components": components,
        "truth_boundary": "original Zeroth-01 16-actuator topology plus exact STS3250, positive-volume fixed torso with integrated hip-yaw case mounts, explicit parent thrust ring and child PCD14 output stack; native SolidWorks mate/Motion gate is required separately",
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({key: payload[key] for key in ("component_count", "movable_joint_count", "blue_sts3250_count", "ankle_roll_actuator_count")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
