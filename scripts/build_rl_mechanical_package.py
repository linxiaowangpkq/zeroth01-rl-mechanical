from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_ready.urdf"
GLOBAL_BOX_REPORT = ROOT / "reports" / "global_collision_box_search.json"
SOURCE_MANIFEST = ROOT / "reports" / "source_asset_manifest.csv"
GENERATED_CONFIG = ROOT / "generated" / "config"
ACTUATOR_JSON = GENERATED_CONFIG / "zeroth01_actuator_metadata.json"
CALIBRATION_CSV = GENERATED_CONFIG / "zeroth01_hardware_calibration_template.csv"
COLLISION_JSON = GENERATED_CONFIG / "zeroth01_collision_policy.json"
JOINT_FRAMES_CSV = ROOT / "reports" / "joint_servo_frames.csv"
INERTIA_CSV = ROOT / "reports" / "link_inertial_audit.csv"
COMPLETENESS_CSV = ROOT / "reports" / "rl_mechanical_completeness.csv"
COMPLETENESS_JSON = ROOT / "reports" / "rl_mechanical_completeness_summary.json"

IDENTITY = np.eye(3)

# K-Scale family bus-ID convention. The 5-DoF leg mapping is present in the
# archived official zbot metadata. The 3-DoF arm chain is adapted from the
# official zbot-6dof chain order and therefore must be confirmed by bus scan.
CANDIDATE_SERVO_IDS = {
    "left_shoulder_pitch": 11,
    "left_shoulder_yaw": 12,
    "left_elbow_yaw": 13,
    "right_shoulder_pitch": 21,
    "right_shoulder_yaw": 22,
    "right_elbow_yaw": 23,
    "left_hip_yaw": 31,
    "left_hip_roll": 32,
    "left_hip_pitch": 33,
    "left_knee_pitch": 34,
    "left_ankle_pitch": 35,
    "right_hip_yaw": 41,
    "right_hip_roll": 42,
    "right_hip_pitch": 43,
    "right_knee_pitch": 44,
    "right_ankle_pitch": 45,
}

LOWER_BODY_JOINTS = {
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
}

OFFICIAL_STANDING_POSE = {
    "left_hip_pitch": 0.23,
    "left_knee_pitch": -0.741,
    "left_hip_yaw": 0.0,
    "left_hip_roll": 0.0,
    "left_ankle_pitch": -0.5,
    "right_hip_pitch": -0.23,
    "right_knee_pitch": 0.741,
    "right_ankle_pitch": 0.5,
    "right_hip_yaw": 0.0,
    "right_hip_roll": 0.0,
}

# Feetech STS3250 official manufacturer data at 12 V.
SERVO_MASS_KG = 0.0745
SERVO_SIZE_M = [0.04522, 0.02472, 0.035]
RATED_TORQUE_NM = 16.0 * 0.0980665
STALL_TORQUE_NM = 50.0 * 0.0980665
STALL_CURRENT_A = 4.2
NOMINAL_VOLTAGE_V = 12.0
NO_LOAD_SPEED_RAD_S = (math.pi / 3.0) / 0.133
EFFECTIVE_OUTPUT_RESISTANCE_OHM = NOMINAL_VOLTAGE_V / STALL_CURRENT_A
EFFECTIVE_OUTPUT_KT_NM_A = STALL_TORQUE_NM / STALL_CURRENT_A


def parse_vec(text: str | None, default=(0.0, 0.0, 0.0)) -> np.ndarray:
    return np.array(
        [float(value) for value in text.split()] if text else default,
        dtype=float,
    )


def rpy_matrix(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)
    return rz @ ry @ rx


def fmt_vec(vector: np.ndarray) -> str:
    return " ".join(f"{float(value):.9f}" for value in vector)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields: list[str] = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def load_mesh_audit() -> dict[str, dict[str, str]]:
    with SOURCE_MANIFEST.open(encoding="utf-8-sig", newline="") as stream:
        return {
            row["target_name"]: row
            for row in csv.DictReader(stream)
            if row.get("target_name")
        }


def load_urdf() -> ET.Element:
    if not URDF.is_file():
        raise FileNotFoundError(URDF)
    return ET.parse(URDF).getroot()


def joint_data(root: ET.Element) -> list[dict[str, object]]:
    rows = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        dynamics = joint.find("dynamics")
        rows.append(
            {
                "name": joint.get("name", ""),
                "type": joint.get("type", ""),
                "parent": joint.find("parent").get("link", ""),
                "child": joint.find("child").get("link", ""),
                "xyz": parse_vec(origin.get("xyz") if origin is not None else None),
                "rpy": parse_vec(origin.get("rpy") if origin is not None else None),
                "axis": parse_vec(
                    axis.get("xyz") if axis is not None else None,
                    (0.0, 0.0, 1.0),
                ),
                "lower": (
                    float(limit.get("lower"))
                    if limit is not None and limit.get("lower") is not None
                    else None
                ),
                "upper": (
                    float(limit.get("upper"))
                    if limit is not None and limit.get("upper") is not None
                    else None
                ),
                "effort": (
                    float(limit.get("effort"))
                    if limit is not None and limit.get("effort") is not None
                    else None
                ),
                "velocity": (
                    float(limit.get("velocity"))
                    if limit is not None and limit.get("velocity") is not None
                    else None
                ),
                "damping": (
                    float(dynamics.get("damping"))
                    if dynamics is not None
                    and dynamics.get("damping") is not None
                    else None
                ),
                "friction": (
                    float(dynamics.get("friction"))
                    if dynamics is not None
                    and dynamics.get("friction") is not None
                    else None
                ),
            }
        )
    return rows


def forward_kinematics(
    joints: list[dict[str, object]],
) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    transforms = {"base": (IDENTITY.copy(), np.zeros(3))}
    pending = list(joints)
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = str(joint["parent"])
            if parent not in transforms:
                continue
            parent_rotation, parent_position = transforms[parent]
            local_rotation = rpy_matrix(np.asarray(joint["rpy"]))
            local_position = np.asarray(joint["xyz"])
            transforms[str(joint["child"])] = (
                parent_rotation @ local_rotation,
                parent_position + parent_rotation @ local_position,
            )
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise RuntimeError(
                f"unresolved URDF joints: {[row['name'] for row in pending]}"
            )
    return transforms


def make_inertia_reports(
    root: ET.Element,
    mesh_audit: dict[str, dict[str, str]],
) -> tuple[list[dict[str, object]], float, bool]:
    rows: list[dict[str, object]] = []
    total_mass = 0.0
    all_pass = True
    for link in root.findall("link"):
        name = link.get("name", "")
        inertial = link.find("inertial")
        if inertial is None:
            rows.append({"link": name, "status": "FAIL", "reason": "missing inertial"})
            all_pass = False
            continue
        mass = float(inertial.find("mass").get("value"))
        origin = inertial.find("origin")
        com = parse_vec(origin.get("xyz") if origin is not None else None)
        inertia = inertial.find("inertia")
        matrix = np.array(
            [
                [
                    float(inertia.get("ixx")),
                    float(inertia.get("ixy")),
                    float(inertia.get("ixz")),
                ],
                [
                    float(inertia.get("ixy")),
                    float(inertia.get("iyy")),
                    float(inertia.get("iyz")),
                ],
                [
                    float(inertia.get("ixz")),
                    float(inertia.get("iyz")),
                    float(inertia.get("izz")),
                ],
            ]
        )
        eigenvalues = np.linalg.eigvalsh(matrix)
        positive = bool(mass > 0.0 and np.all(eigenvalues > 0.0))
        triangle = bool(
            eigenvalues[2] <= eigenvalues[0] + eigenvalues[1] + 1e-12
        )
        passed = positive and triangle
        all_pass = all_pass and passed
        total_mass += mass
        mesh = link.find("./visual/geometry/mesh")
        mesh_name = (
            Path(mesh.get("filename", "").replace("\\", "/")).name
            if mesh is not None
            else ""
        )
        audit = mesh_audit.get(mesh_name, {})
        rows.append(
            {
                "link": name,
                "mass_kg": f"{mass:.12f}",
                "com_xyz_link_m": fmt_vec(com),
                "ixx_kg_m2": f"{matrix[0, 0]:.12g}",
                "ixy_kg_m2": f"{matrix[0, 1]:.12g}",
                "ixz_kg_m2": f"{matrix[0, 2]:.12g}",
                "iyy_kg_m2": f"{matrix[1, 1]:.12g}",
                "iyz_kg_m2": f"{matrix[1, 2]:.12g}",
                "izz_kg_m2": f"{matrix[2, 2]:.12g}",
                "principal_moments_kg_m2": fmt_vec(eigenvalues),
                "positive_definite": str(positive).lower(),
                "triangle_inequality": str(triangle).lower(),
                "source": "official_zeroth_sim_urdf",
                "scope": "aggregate_link_including_servo_and_structure",
                "mesh_watertight": audit.get("watertight", "not_applicable"),
                "status": "PASS" if passed else "FAIL",
            }
        )
    return rows, total_mass, all_pass


def make_joint_frame_reports(
    joints: list[dict[str, object]],
) -> list[dict[str, object]]:
    transforms = forward_kinematics(joints)
    rows: list[dict[str, object]] = []
    for joint in joints:
        if joint["type"] not in {"revolute", "continuous"}:
            continue
        name = str(joint["name"])
        parent_rotation, parent_position = transforms[str(joint["parent"])]
        joint_rotation = parent_rotation @ rpy_matrix(np.asarray(joint["rpy"]))
        world_position = parent_position + parent_rotation @ np.asarray(joint["xyz"])
        world_axis = joint_rotation @ np.asarray(joint["axis"])
        id_confidence = (
            "official_archived_zbot_5dof_leg_requires_bus_scan"
            if name in LOWER_BODY_JOINTS
            else "derived_kscale_family_chain_order_requires_bus_scan"
        )
        rows.append(
            {
                "joint": name,
                "servo_model": "Feetech_STS3250",
                "parent_link": joint["parent"],
                "child_link": joint["child"],
                "shaft_origin_xyz_parent_m": fmt_vec(np.asarray(joint["xyz"])),
                "shaft_origin_rpy_parent_rad": fmt_vec(np.asarray(joint["rpy"])),
                "positive_axis_joint_frame": fmt_vec(np.asarray(joint["axis"])),
                "neutral_shaft_xyz_world_m": fmt_vec(world_position),
                "neutral_positive_axis_world": fmt_vec(world_axis),
                "guarded_lower_rad": f"{float(joint['lower']):.9f}",
                "guarded_upper_rad": f"{float(joint['upper']):.9f}",
                "candidate_bus_id": CANDIDATE_SERVO_IDS[name],
                "candidate_bus_id_confidence": id_confidence,
                "nominal_zero_count": 2048,
                "counts_per_revolution": 4096,
                "urdf_to_servo_direction_sign": "REQUIRES_JOG_CALIBRATION",
                "hardware_zero_offset_counts": "REQUIRES_PHYSICAL_CALIBRATION",
                "spatial_semantics": (
                    "URDF joint frame equals output-shaft frame; servo housing "
                    "center is not separately recoverable from the aggregate STL"
                ),
                "source": "official_urdf_frame",
            }
        )
    return rows


def make_actuator_payload(
    joint_rows: list[dict[str, object]],
) -> dict[str, object]:
    actuators = {}
    for joint in joint_rows:
        name = str(joint["name"])
        lower_body = name in LOWER_BODY_JOINTS
        actuators[name] = {
            "model": "Feetech STS3250",
            "candidate_bus_id": CANDIDATE_SERVO_IDS[name],
            "candidate_bus_id_status": (
                "official_archived_zbot_5dof_leg_requires_bus_scan"
                if lower_body
                else "derived_kscale_family_chain_order_requires_bus_scan"
            ),
            "joint_lower_rad": joint["lower"],
            "joint_upper_rad": joint["upper"],
            "joint_velocity_limit_rad_s": joint["velocity"],
            "urdf_effort_limit_nm": joint["effort"],
            "manufacturer_rated_torque_nm_at_12v": RATED_TORQUE_NM,
            "manufacturer_stall_torque_nm_at_12v": STALL_TORQUE_NM,
            "manufacturer_no_load_speed_rad_s_at_12v": NO_LOAD_SPEED_RAD_S,
            "manufacturer_mass_kg": SERVO_MASS_KG,
            "official_sim_joint_damping_nm_s_rad": joint["damping"],
            "official_sim_joint_frictionloss_nm": joint["friction"],
            "official_sim_armature_kg_m2": 0.008793405204572328,
            "rl_pd_kp_nominal": 17.681462808698132 if lower_body else 5.0,
            "rl_pd_kd_nominal": 0.5354656169048285 if lower_body else 0.3,
            "rl_pd_source": (
                "official_stompymicro_active_leg_config"
                if lower_body
                else "official_stompymicro_commented_arm_candidate_requires_tuning"
            ),
            "rl_vmax_rad_s": 5.0,
            "rl_amax_rad_s2": 39.0,
            "hardware_zero_count_nominal": 2048,
            "hardware_zero_offset_counts": None,
            "urdf_to_servo_direction_sign": None,
            "calibration_gate": "bus_scan_then_zero_fixture_then_low_torque_jog",
        }
    return {
        "schema": "zeroth01.rl_actuator_metadata.v1",
        "robot": "Zeroth-01 StompyMicro 16DoF",
        "control_frequency_hz_candidate": 50,
        "control_frequency_source": "official_archived_kscale_zbot_family_metadata",
        "manufacturer": {
            "model": "Feetech STS3250",
            "size_xyz_m": SERVO_SIZE_M,
            "mass_kg": SERVO_MASS_KG,
            "nominal_voltage_v": NOMINAL_VOLTAGE_V,
            "rated_torque_nm_at_12v": RATED_TORQUE_NM,
            "stall_torque_nm_at_12v": STALL_TORQUE_NM,
            "stall_current_a_at_12v": STALL_CURRENT_A,
            "no_load_speed_rad_s_at_12v": NO_LOAD_SPEED_RAD_S,
            "encoder_counts_per_revolution": 4096,
            "neutral_count_nominal": 2048,
        },
        "derived_effective_output_model": {
            "resistance_ohm_from_12v_over_stall_current": EFFECTIVE_OUTPUT_RESISTANCE_OHM,
            "kt_nm_per_a_from_stall_torque_over_stall_current": EFFECTIVE_OUTPUT_KT_NM_A,
            "warning": (
                "Output-side effective parameters include gearbox/controller "
                "effects and are not bare motor winding constants."
            ),
        },
        "training_policy": {
            "continuous_torque_reference_nm": RATED_TORQUE_NM,
            "urdf_and_official_sim_peak_command_limit_nm": 2.0,
            "manufacturer_stall_torque_nm_not_for_continuous_rl": STALL_TORQUE_NM,
            "randomization": {
                "link_mass_scale": [0.95, 1.05],
                "joint_damping_scale": [0.7, 1.3],
                "armature_scale": [0.7, 1.3],
                "frictionloss_scale": [0.5, 1.5],
                "joint_zero_offset_rad": [
                    -math.radians(2.0),
                    math.radians(2.0),
                ],
            },
        },
        "actuators": actuators,
        "unresolved_hardware_fields": [
            "confirmed bus ID for this exact 16DoF harness",
            "URDF-to-servo positive direction sign",
            "per-unit zero offset",
            "per-unit backlash/deadband",
            "identified armature/damping/friction and torque-current curve",
            "thermal continuous-duty envelope",
        ],
    }


def make_calibration_rows(
    joint_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    return [
        {
            "joint": joint["name"],
            "servo_model": "STS3250",
            "candidate_bus_id": CANDIDATE_SERVO_IDS[str(joint["name"])],
            "confirmed_bus_id": "",
            "nominal_zero_count": 2048,
            "measured_zero_count": "",
            "zero_offset_counts": "",
            "urdf_to_servo_direction_sign": "",
            "measured_hardstop_lower_count": "",
            "measured_hardstop_upper_count": "",
            "backlash_counts": "",
            "no_load_current_a": "",
            "notes": "REQUIRED_BEFORE_HARDWARE_DEPLOYMENT",
        }
        for joint in joint_rows
        if joint["type"] in {"revolute", "continuous"}
    ]


def make_completeness_rows(
    root: ET.Element,
    joint_rows: list[dict[str, object]],
    inertia_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    inertia_by_link = {str(row["link"]): row for row in inertia_rows}
    for link in root.findall("link"):
        name = link.get("name", "")
        visual = link.find("visual")
        collision = link.find("collision")
        inertia = inertia_by_link[name]
        for field_group, present, source, confidence, action in (
            (
                "mass_com_inertia",
                inertia["status"] == "PASS",
                "official_zeroth_sim_urdf",
                "official_aggregate_cad_export_unmeasured",
                "weigh assembled links and update after first hardware build",
            ),
            (
                "visual_geometry",
                visual is not None or name == "base",
                "official_drive_stl",
                "official_open_surface_mesh",
                "replace with native B-Rep only if released or reverse-modeled",
            ),
            (
                "collision_geometry",
                collision is not None or name == "base",
                "official_drive_stl_plus_collision_policy",
                "sampled_mesh_collision_not_tolerance_proof",
                "keep online self-collision guard outside startup envelope",
            ),
        ):
            rows.append(
                {
                    "object_type": "link",
                    "object_name": name,
                    "field_group": field_group,
                    "status": "PASS" if present else "FAIL",
                    "source": source,
                    "confidence": confidence,
                    "remaining_action": action,
                }
            )

    for joint in joint_rows:
        if joint["type"] not in {"revolute", "continuous"}:
            continue
        name = str(joint["name"])
        checks = (
            (
                "kinematic_frame_axis",
                np.linalg.norm(np.asarray(joint["axis"])) > 0.999,
                "official_zeroth_sim_urdf",
                "official",
                "none",
            ),
            (
                "guarded_limits_dynamics",
                joint["lower"] < joint["upper"]
                and joint["effort"] is not None
                and joint["velocity"] is not None
                and joint["damping"] is not None
                and joint["friction"] is not None,
                "official_sim_plus_collision_audit",
                "sampled_guarded",
                "expand only with collision-aware policy",
            ),
            (
                "servo_model_and_spatial_position",
                name in CANDIDATE_SERVO_IDS,
                "official_bom_plus_urdf_joint_frame",
                "official_model_and_shaft_frame",
                "housing center remains embedded in aggregate mesh",
            ),
            (
                "hardware_id_zero_direction",
                False,
                "kscale_family_candidate_only",
                "requires_physical_calibration",
                "bus scan, zero fixture and low-torque jog are mandatory",
            ),
        )
        for field_group, passed, source, confidence, action in checks:
            rows.append(
                {
                    "object_type": "joint",
                    "object_name": name,
                    "field_group": field_group,
                    "status": "PASS" if passed else "BLOCKED_HARDWARE_CALIBRATION",
                    "source": source,
                    "confidence": confidence,
                    "remaining_action": action,
                }
            )
    return rows


def main() -> None:
    root = load_urdf()
    mesh_audit = load_mesh_audit()
    joints = joint_data(root)
    moving = [
        joint
        for joint in joints
        if joint["type"] in {"revolute", "continuous"}
    ]
    if len(moving) != 16:
        raise RuntimeError(f"expected 16 moving joints, got {len(moving)}")
    if set(CANDIDATE_SERVO_IDS) != {str(row["name"]) for row in moving}:
        raise RuntimeError("candidate actuator mapping does not match URDF joints")

    inertia_rows, total_mass, inertia_pass = make_inertia_reports(
        root, mesh_audit
    )
    frame_rows = make_joint_frame_reports(joints)
    actuator_payload = make_actuator_payload(moving)
    calibration_rows = make_calibration_rows(joints)
    search = json.loads(GLOBAL_BOX_REPORT.read_text(encoding="utf-8"))
    neutral_allowed_pairs = []
    for value in search.get("allowed_neutral_pairs", []):
        names = [name.strip() for name in str(value).split("::")]
        if len(names) != 2 or not all(names):
            raise ValueError(f"invalid neutral overlap pair: {value!r}")
        neutral_allowed_pairs.append((names[0], names[1]))
    collision_payload = {
        "schema": "zeroth01.collision_policy.v1",
        "geometry": (
            "official open-surface STL used for visual and collision geometry"
        ),
        "allowed_assembly_overlap_pairs": [
            {"body1": first, "body2": second}
            for first, second in sorted(neutral_allowed_pairs)
        ],
        "all_other_self_contacts": "prohibited",
        "guarded_startup_box": search["selected"],
        "validation": {
            "contact_margin_mm": search["contact_margin_mm"],
            "penetration_epsilon_mm": search["penetration_epsilon_mm"],
            "random_sample_count": search["selected"]["random_samples"],
            "corner_sample_count": search["selected"]["corner_samples"],
            "result": "PASS",
            "scope": (
                "sampled mesh kinematics; excludes cables, fasteners, covers, "
                "print tolerance and deformation"
            ),
        },
        "full_audited_mechanical_range_policy": (
            "allowed only with online self-collision query/action projection "
            "and collision termination; independent full-range sampling is unsafe"
        ),
    }
    completeness_rows = make_completeness_rows(
        root, joints, inertia_rows
    )
    blocked = [
        row
        for row in completeness_rows
        if row["status"] == "BLOCKED_HARDWARE_CALIBRATION"
    ]
    failed = [row for row in completeness_rows if row["status"] == "FAIL"]
    summary = {
        "urdf": str(URDF),
        "link_count": len(root.findall("link")),
        "joint_count": len(root.findall("joint")),
        "moving_joint_count": len(moving),
        "total_mass_kg": total_mass,
        "inertia_gate": "PASS" if inertia_pass else "FAIL",
        "connected_tree_gate": "PASS",
        "guarded_collision_gate": "PASS",
        "hardware_calibration_block_count": len(blocked),
        "hard_fail_count": len(failed),
        "rl_simulation_ready": not failed and inertia_pass,
        "hardware_deployment_ready": False,
        "reason_hardware_not_ready": (
            "per-unit servo ID, zero, direction, backlash and SysID remain "
            "physical calibration tasks"
        ),
        "aggregate_link_inertia_warning": (
            "Link inertias already include servos/structure; do not add 16 "
            "separate 74.5 g servo masses to this URDF."
        ),
    }

    GENERATED_CONFIG.mkdir(parents=True, exist_ok=True)
    ACTUATOR_JSON.write_text(
        json.dumps(actuator_payload, indent=2) + "\n", encoding="utf-8"
    )
    COLLISION_JSON.write_text(
        json.dumps(collision_payload, indent=2) + "\n", encoding="utf-8"
    )
    COMPLETENESS_JSON.write_text(
        json.dumps(summary, indent=2) + "\n", encoding="utf-8"
    )
    write_csv(CALIBRATION_CSV, calibration_rows)
    write_csv(JOINT_FRAMES_CSV, frame_rows)
    write_csv(INERTIA_CSV, inertia_rows)
    write_csv(COMPLETENESS_CSV, completeness_rows)
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
