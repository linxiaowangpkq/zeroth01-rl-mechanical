"""Build the full original-minimal v4 external-part assembly manifest."""

from __future__ import annotations

import importlib.util
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
V3_DIR = ROOT / "cad" / "physical_mount_v3_rl_fixed"
V1_STL = ROOT / "generated" / "cad" / "physical_mount_v1" / "skeleton"
V2_REPLACEMENTS = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements"
V3_PARTS = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed" / "parts"
V4_PARTS = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "parts"
ACTUATOR_LAYOUT = ROOT / "generated" / "config" / "physical_mount_v3_rl_fixed_actuator_layout.json"
OUT = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v4_original_minimal"
    / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)

WHITE = "#F7F8FA"
BLUE = "#1677FF"
BLACK = "#101820"
CYAN = "#00B8D9"
ORANGE = "#FF9100"
MAGENTA = "#D500F9"
GREEN = "#64DD17"
GREY = "#BFC7D1"
SHIN_SHORTEN_M = 0.018
ANKLE_ROLL_DIRECT_OFFSET_M = 0.0265


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    m = load(V3_DIR / "build_v3_assembly_manifest.py", "zeroth_v3_manifest_helpers")
    u = load(V3_DIR / "build_v3_urdf.py", "zeroth_v3_urdf_for_v4")
    old_robot = ET.parse(u.V2_URDF).getroot()
    old_tf = u.old_fk(old_robot)
    neutral_tf = u.neutral_transforms(old_tf)
    # Preserve the released terminal ankle interfaces while shortening only
    # the straight middle span of each lower-leg carrier.  The ankle-pitch
    # shaft therefore rises by 18 mm.  A robust direct STS3250 roll stage then
    # places the foot 26.5 mm below that shaft.  This is the smallest verified
    # change that keeps the complete robot below 500 mm without a fragile
    # remote belt drive.
    for carrier, foot in (
        (u.LEFT_ANKLE_CARRIER, "FOOT"),
        (u.RIGHT_ANKLE_CARRIER, "FOOT_2"),
    ):
        rotation, released_position = old_tf[foot]
        ankle_pitch_position = u.add(released_position, (0.0, 0.0, SHIN_SHORTEN_M))
        neutral_tf[carrier] = (rotation, ankle_pitch_position)
        neutral_tf[foot] = (
            rotation,
            u.add(ankle_pitch_position, (0.0, 0.0, -ANKLE_ROLL_DIRECT_OFFSET_M)),
        )
    positions = {name: transform[1] for name, transform in neutral_tf.items()}
    pos_mm = {
        name: tuple(value * 1000.0 for value in xyz)
        for name, xyz in positions.items()
    }
    fitted_servo_pose = m.fitted_servo_rotations(old_robot, old_tf)
    identity = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    components = []

    carrier_links = [u.BODY]
    carrier_links.extend(
        spec[2]
        for spec in u.JOINT_SPECS
        if spec[2] not in {u.LEFT_ANKLE_CARRIER, u.RIGHT_ANKLE_CARRIER, "FINGER_1", "FINGER_1_2"}
    )
    for link_name in dict.fromkeys(carrier_links):
        if link_name == u.BODY:
            source = V4_PARTS / "body_original_head_interface_trimmed_2p5mm.step"
            note = "official v1 body with old head removed at the measured seam and only a 2.5 mm planar top-interface trim; legacy 6368-face carrier otherwise unchanged"
        elif link_name == "3215_BothFlange_13":
            source = V4_PARTS / "left_source_shin_shortened_18mm.step"
            note = "official left lower-leg carrier shortened only in its straight middle span by 18 mm; both terminal STS3250 interfaces retained"
        elif link_name == "3215_BothFlange_14":
            source = V4_PARTS / "right_source_shin_shortened_18mm.step"
            note = "official right lower-leg carrier shortened only in its straight middle span by 18 mm; both terminal STS3250 interfaces retained"
        elif link_name == "R_ARM_MIRROR_1":
            source = V2_REPLACEMENTS / "R_ARM_MIRROR_1_WRIST_TRIMMED.step"
            note = "released v2 source forearm with only the old claw root trimmed for compact fixed palm"
        elif link_name == "L_ARM_MIRROR_1":
            source = V2_REPLACEMENTS / "L_ARM_MIRROR_1_WRIST_TRIMMED.step"
            note = "released v2 source forearm with only the old claw root trimmed for compact fixed palm"
        else:
            source = V1_STL / f"{link_name}.stl"
            note = "unchanged released v1 source carrier; exact neutral transform retained"
        components.append(
            m.row(
                ROOT,
                f"CARRIER_{link_name}",
                "source_load_bearing_carrier",
                source,
                m.matrix4(old_tf[link_name][0], pos_mm[link_name]),
                WHITE,
                link_name,
                note,
            )
        )

    for component_id, owner, filename in (
        ("LEFT_SMALL_LIGHT_PALM", "FINGER_1", "left_small_light_palm.step"),
        ("RIGHT_SMALL_LIGHT_PALM", "FINGER_1_2", "right_small_light_palm.step"),
    ):
        components.append(
            m.row(
                ROOT,
                component_id,
                "compact_fixed_palm_twin_ear_m3",
                V4_PARTS / filename,
                m.matrix4(old_tf[owner][0], tuple(value * 1000.0 for value in old_tf[owner][1])),
                WHITE,
                owner,
                "30 x 36 x 24 mm light palm body; source claw absent; fixed wrist datum and M3 crossbolt retained",
            )
        )

    v4_body_parts = (
        ("V4_HEAD_FRONT", "printable_head_front_shell", "head_front_5mm_each_side.step", WHITE, u.BODY, "nominal source envelope +5 mm left/right/top/bottom with 5 mm edge radius; hidden underside has exact shoulder-servo clearance pockets"),
        ("V4_HEAD_REAR", "printable_head_rear_shell", "head_rear_5mm_each_side.step", WHITE, u.BODY, "two-piece 2.2 mm wall shell with USB-C cable exit"),
        ("V4_HEAD_VISOR", "simple_camera_microphone_visor", "head_simple_visor.step", BLACK, u.BODY, "simple original-style black face; camera and acoustic ports only"),
        ("V4_UNITV2", "purchased_internal_interaction_module", "m5stack_unitv2_purchased_envelope.step", CYAN, u.BODY, "M5Stack UnitV2 official 48 x 18.5 x 24 mm, 18 g, GC2145 camera and microphone"),
        ("V4_UNITV2_CRADLE", "removable_internal_service_mount", "unitv2_removable_cradle.step", GREY, u.BODY, "M2.5 removable U-cradle"),
        ("V4_HEAD_TORSO_NUT_PLATE", "direct_head_torso_mount", "direct_head_torso_nut_plate.step", GREY, u.BODY, "four M3 direct torso fasteners; no neck; drill jig is a manufacturing tool and is not installed"),
        ("V4_COMPUTE_ENVELOPE", "internal_payload_controlled_envelope", "compute_envelope.step", ORANGE, u.BODY, "70 x 12 x 32 mm Raspberry Pi Zero 2 W-class compute volume, recessed and hidden in normal assembly"),
        ("V4_COMPUTE_TRAY", "removable_internal_service_mount", "compute_removable_tray.step", GREY, u.BODY, "M3 removable tray with side rails"),
        ("V4_BATTERY_ENVELOPE", "internal_payload_controlled_envelope", "battery_envelope.step", MAGENTA, u.BODY, "75 x 22 x 34 mm thin 2S battery service volume, recessed and hidden in normal assembly"),
        ("V4_BATTERY_CAGE", "removable_internal_service_mount", "battery_service_cage.step", GREY, u.BODY, "strap ribs and front service opening"),
        ("V4_TORSO_IMU_ENVELOPE", "internal_payload_controlled_envelope", "torso_imu_envelope.step", GREEN, u.BODY, "32 x 25 x 8 mm IMU volume on rigid torso, not the head"),
        ("V4_REAR_SERVICE_POD", "reversible_white_rear_service_pod", "rear_service_pod.step", WHITE, u.BODY, "32 mm deep open-front white pod; four M3 standoffs, integral IMU shelf and four cable guides; original torso is not cut"),
    )
    for component_id, role, filename, color, owner, note in v4_body_parts:
        components.append(
            m.row(
                ROOT,
                component_id,
                role,
                V4_PARTS / filename,
                # V4 head and service-pod geometry is authored in the released
                # Zeroth-01 BODY local frame.  The released BODY mesh itself is
                # installed with a ~90 degree yaw, so body-attached parts must
                # inherit that rotation instead of using the assembly identity.
                m.matrix4(old_tf[u.BODY][0], (0.0, 0.0, 0.0)),
                color,
                owner,
                note,
            )
        )

    actuator_ids = {
        str(item["joint"]): str(item["id"])
        for item in json.loads(ACTUATOR_LAYOUT.read_text(encoding="utf-8"))["actuators"]
    }
    source_servo_owner = {}
    hip_yaw_spacer_poses = {}
    for link in old_robot.findall("link"):
        for visual in link.findall("visual"):
            name = str(visual.get("name", ""))
            if name.endswith("_blue_servo_visual"):
                source_servo_owner[
                    name.split("_", 1)[1].removesuffix("_blue_servo_visual")
                ] = str(link.get("name"))
    for joint_name, parent, child, axis, _ in u.JOINT_SPECS:
        servo_id = actuator_ids[joint_name]
        servo_position = pos_mm[child]
        if joint_name in fitted_servo_pose:
            servo_rotation, residual = fitted_servo_pose[joint_name]
            note = f"released full 6D installation retained; centroid fit residual {residual:.3f} mm"
            if joint_name in {"left_hip_yaw", "right_hip_yaw"}:
                clocking_deg = -30.0 if joint_name == "left_hip_yaw" else 30.0
                servo_rotation = u.mat_mul(
                    u.rpy_matrix((0.0, 0.0, math.radians(clocking_deg))),
                    servo_rotation,
                )
                servo_position = (
                    servo_position[0],
                    servo_position[1],
                    servo_position[2] + 2.0,
                )
                hip_yaw_spacer_poses[joint_name] = (servo_rotation, pos_mm[child], child)
                note = (
                    f"SolidWorks-screened symmetric re-clocking {clocking_deg:+.0f} deg and "
                    "+2 mm along the common yaw axis; axis-line kinematics unchanged"
                )
        else:
            servo_rotation = m.ankle_servo_rotation(axis)
            note = (
                "v4 mirrored direct-drive ankle; 26.5 mm pitch-to-roll shaft "
                "spacing gives nominal STS3250 case clearance"
            )
        components.append(
            m.row(
                ROOT,
                f"{servo_id}_STS3250_{joint_name}",
                "dimension_controlled_sts3250",
                V3_PARTS / "sts3250_dimension_controlled.step",
                m.matrix4(servo_rotation, servo_position),
                BLUE,
                source_servo_owner.get(joint_name, parent),
                f"shaft origin matches URDF joint frame; {note}",
            )
        )

    for joint_name, (rotation, position, owner) in hip_yaw_spacer_poses.items():
        components.append(
            m.row(
                ROOT,
                f"{joint_name.upper()}_PCD14_2MM_OUTPUT_SPACER",
                "hip_yaw_2mm_output_axis_spacer",
                V4_PARTS / "hip_yaw_horn_spacer_2mm.step",
                m.matrix4(rotation, position),
                BLUE,
                owner,
                "2 mm PCD14 four-M3 spacer bridges the axial servo shift without moving the revolute axis line",
            )
        )

    for side, carrier, foot, axis in (
        ("left", u.LEFT_ANKLE_CARRIER, "FOOT", (1.0, 0.0, 0.0)),
        ("right", u.RIGHT_ANKLE_CARRIER, "FOOT_2", (-1.0, 0.0, 0.0)),
    ):
        rotation = m.ankle_servo_rotation(axis)
        components.append(m.row(ROOT, f"{side.upper()}_ANKLE_ROLL_CARRIER", "direct_ankle_roll_parent_carrier", V4_PARTS / f"{side}_direct_ankle_carrier_26p5mm.step", m.matrix4(rotation, pos_mm[foot]), WHITE, carrier, "direct double-ear cage; 26.5 mm pitch-to-roll spacing and mirrored STS3250 installation"))
        components.append(m.row(ROOT, f"{side.upper()}_ANKLE_ROLL_HORN", "direct_ankle_roll_child_horn_adapter", V3_PARTS / f"{side}_ankle_roll_horn_adapter.step", m.matrix4(rotation, pos_mm[foot]), BLUE, foot, "PCD14 four-M3 output horn adapter retained from verified v3 direct-drive ankle"))
        components.append(m.row(ROOT, f"{side.upper()}_7MM_LIGHTWEIGHT_SOLE", "replaceable_7mm_perimeter_rib_sole", V3_PARTS / f"{side}_sole_lightweighted.step", m.matrix4(old_tf[foot][0], pos_mm[foot]), BLACK, foot, "retained 7 mm lightened sole"))

    payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.external_part_assembly.v1",
        "units": "mm",
        "frame": "released Zeroth source link frames; RL convention X forward, Y left, Z up",
        "component_count": len(components),
        "movable_joint_count": len(u.JOINT_SPECS),
        "blue_sts3250_count": sum(item["role"] == "dimension_controlled_sts3250" for item in components),
        "old_claw_count": 0,
        "large_q_hand_count": 0,
        "head_expansion_each_side_mm": 5.0,
        "head_local_underside_exception": "0.8 mm B-Rep pockets around both shoulder servos; local lower-corner solid expansion is about 3.9 mm",
        "purchased_head_module": "M5Stack UnitV2",
        "electronics_service_strategy": "reversible white rear service pod because the released torso is a solid 6368-face STL conversion, not a hollow parametric shell",
        "hip_yaw_installation": "left/right symmetric -30/+30 degree clocking plus 2 mm output-axis PCD14 spacer; SolidWorks zero-volume screening pass",
        "lower_leg_change": "18 mm removed only from each straight carrier mid-span; both terminal interfaces preserved",
        "ankle_roll_transmission": {
            "type": "direct serial STS3250",
            "pitch_to_roll_shaft_spacing_mm": 26.5,
            "nominal_case_clearance_mm": 1.64,
            "foot_frame_shift_from_release_mm": [0.0, 0.0, -8.5],
        },
        "manufacturing_tool_not_installed": "parts/head_mount_4xm3_drill_jig.step",
        "components": components,
        "symmetry_policy": "preserve physical source axes; gate mirrored axis-line and housing errors, not arbitrary origin displacement along a revolute axis",
        "truth_boundary": "official source carrier geometry + controlled STS3250 envelope + official-size UnitV2 envelope; purchased first articles and as-built mass/inertia remain HOLD",
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(OUT)
    print(json.dumps({key: payload[key] for key in ("component_count", "movable_joint_count", "blue_sts3250_count", "old_claw_count", "large_q_hand_count")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
