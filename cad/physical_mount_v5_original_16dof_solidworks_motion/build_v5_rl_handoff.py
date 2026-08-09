"""Generate the complete 16-DoF actuator, calibration and RL handoff."""

from __future__ import annotations

import csv
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
URDF = ROOT / "generated" / "urdf" / "physical_mount_v5_original_16dof_solidworks_motion" / "zeroth01_physical_mount_v5_original_16dof_solidworks_motion.urdf"
MJCF = ROOT / "generated" / "mujoco" / "physical_mount_v5_original_16dof_solidworks_motion" / "zeroth01_physical_mount_v5_original_16dof_solidworks_motion_mjx.xml"
MASS_REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "urdf_mass_inertia_gate.json"
MJCF_REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "mjcf_compile_gate.json"
BREP_REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "offline_brep_component_interference.json"
SW_GATE = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "solidworks_gate.json"
SW_MOTION_GATE = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "solidworks_isolated_motion_gate.json"
V1_ACTUATORS = ROOT / "generated" / "config" / "physical_mount_v1_actuators.json"
CONFIG_ROOT = ROOT / "generated" / "config"
ACTUATORS = CONFIG_ROOT / "physical_mount_v5_original_16dof_solidworks_motion_actuator_layout.json"
CALIBRATION = CONFIG_ROOT / "physical_mount_v5_original_16dof_solidworks_motion_hardware_calibration.csv"
HANDOFF = CONFIG_ROOT / "physical_mount_v5_original_16dof_solidworks_motion_rl_handoff.json"
RELEASE_GATE = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "release_gate.json"
MODEL_CONTRACT = Path(__file__).with_name("RL_MODEL_CONTRACT.md")


def optional_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8")) if path.is_file() else None


def current_gate(path: Path, manifest_sha: str):
    payload = optional_json(path)
    if payload is None:
        return "PENDING_RERUN"
    if payload.get("manifest_sha256") not in (None, manifest_sha):
        return "STALE_MANIFEST_RERUN_REQUIRED"
    return payload.get("overall", "UNKNOWN")


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    manifest_sha = hashlib.sha256(MANIFEST.read_bytes()).hexdigest()
    robot = ET.parse(URDF).getroot()
    joints = {str(joint.get("name")): joint for joint in robot.findall("joint")}
    mjcf = json.loads(MJCF_REPORT.read_text(encoding="utf-8"))
    model_contract = mjcf.get("model_contract", {})
    mapping_by_joint = {
        str(row["joint"]): row for row in model_contract.get("index_mapping", [])
    }
    v1 = {str(row["joint"]): row for row in json.loads(V1_ACTUATORS.read_text(encoding="utf-8"))["servos"]}
    components = {str(row["component_id"]): row for row in manifest["components"]}
    rows = []
    for spec in manifest["joint_specs"]:
        servo_id = str(spec["id"])
        joint_name = str(spec["name"])
        servo = components[f"{servo_id}_STS3250_{joint_name}"]
        bridge = components[f"{servo_id}_PCD14_OUTPUT_BRIDGE_{joint_name}"]
        transform = servo["transform_local_mm_to_world_mm"]
        limit = joints[joint_name].find("limit")
        original = v1[joint_name]
        index_mapping = mapping_by_joint.get(joint_name)
        if index_mapping is None:
            raise RuntimeError(f"missing MJCF index mapping for {joint_name}")
        rows.append({
            "id": servo_id,
            "joint": joint_name,
            "housing_owner_link": servo["owner_link"],
            "output_bridge_owner_link": bridge["owner_link"],
            "shaft_origin_body_neutral_m": [float(transform[index][3]) * 0.001 for index in range(3)],
            "shaft_axis_body_neutral": [float(transform[index][2]) for index in range(3)],
            "urdf_axis_joint_frame": [float(value) for value in joints[joint_name].find("axis").get("xyz").split()],
            "limit_rad": [float(limit.get("lower")), float(limit.get("upper"))],
            "training_continuous_effort_limit_nm": float(limit.get("effort")),
            "rated_effort_nm": 1.569064,
            "stall_effort_nm_not_for_training": 4.903,
            "velocity_limit_rad_s": float(limit.get("velocity")),
            "model": "FEETECH STS3250-C001",
            "supply_voltage_v": 12.0,
            "protocol": "TTL half-duplex serial",
            "encoder_counts_per_revolution": 4096,
            "gear_ratio": 1.0,
            "mass_kg": 0.0745,
            "cad_source": servo["source"],
            "cad_catalog_sha256": manifest["sts3250_catalog_sha256"],
            "cad_fidelity": "purchased_exact_step",
            "mechanical_transmission": "case fixed to housing-owner carrier; four-M3 PCD14 output bridge fixed to output-owner carrier",
            "bus_id_candidate_from_archived_v1": original.get("bus_id_candidate"),
            "bus_id": "REQUIRES_ONE_AT_A_TIME_PHYSICAL_BUS_SCAN",
            "neutral_count": "REQUIRES_JOG_CALIBRATION",
            "direction_sign": "REQUIRES_JOG_CALIBRATION",
            "hardware_zero_offset_counts": "REQUIRES_PHYSICAL_CALIBRATION",
            "mount_gate": "CAD_BREP_PASS_PHYSICAL_FIRST_ARTICLE_HOLD",
            "mjcf_ctrl_index": int(index_mapping["ctrl_index"]),
            "mjcf_qpos_index": int(index_mapping["qpos_index"]),
            "mjcf_qvel_index": int(index_mapping["qvel_index"]),
        })
    rows.sort(key=lambda row: row["id"])
    if len(rows) != 16 or any("ankle_roll" in row["joint"] for row in rows):
        raise RuntimeError("actuator topology is not original 16-DoF")

    actuator_payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.actuator_layout.v1",
        "manifest_sha256": manifest_sha,
        "frame": "released body-neutral/world convention, X forward, Y left, Z up, metres",
        "count": len(rows),
        "total_actuator_mass_kg": sum(float(row["mass_kg"]) for row in rows),
        "actuators": rows,
        "mjcf_control_order": model_contract.get("actuator_order", []),
        "mjcf_source_sha256": mjcf.get("mjcf_sha256"),
        "calibration_truth_boundary": "Candidate IDs are non-authoritative history. Bus IDs, neutral counts, direction signs, backlash, torque gain and latency require powered physical calibration.",
    }
    CONFIG_ROOT.mkdir(parents=True, exist_ok=True)
    ACTUATORS.write_text(json.dumps(actuator_payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")

    with CALIBRATION.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=(
            "id", "joint", "scanned_bus_id", "neutral_count", "direction_sign",
            "min_safe_count", "max_safe_count", "measured_backlash_rad",
            "measured_latency_s", "torque_gain_scale", "technician", "date", "status", "notes",
        ))
        writer.writeheader()
        for row in rows:
            writer.writerow({"id": row["id"], "joint": row["joint"], "status": "UNMEASURED"})

    mass = json.loads(MASS_REPORT.read_text(encoding="utf-8"))
    brep = json.loads(BREP_REPORT.read_text(encoding="utf-8"))
    handoff = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.rl_handoff.v1",
        "robot": "Zeroth-01 v5 original 16DoF, physically mounted STS3250",
        "manifest_sha256": manifest_sha,
        "urdf": URDF.relative_to(ROOT).as_posix(),
        "mjcf_mjx": MJCF.relative_to(ROOT).as_posix(),
        "actuator_layout": ACTUATORS.relative_to(ROOT).as_posix(),
        "hardware_calibration_template": CALIBRATION.relative_to(ROOT).as_posix(),
        "model_contract_ledger": MODEL_CONTRACT.relative_to(ROOT).as_posix(),
        "nominal_total_mass_kg": mass["nominal_total_mass_kg"],
        "mass_limit_kg": mass["hard_mass_limit_kg"],
        "movable_joint_count": 16,
        "actuator": {
            "model": "FEETECH STS3250-C001",
            "count": 16,
            "training_continuous_effort_nm": 1.2552512,
            "rated_effort_nm": 1.569064,
            "stall_effort_nm_not_for_training": 4.903,
            "max_velocity_rad_s": 3.0,
            "mass_each_kg": 0.0745,
            "encoder_counts_per_revolution": 4096,
        },
        "mechanical": {
            "manufacturing_component_count": manifest["component_count"],
            "solidworks_motion_rigid_link_count": 17,
            "ankle_roll_actuator_count": 0,
            "white_tapered_sole_thickness_mm": 9.0,
            "wrist_support": "small rounded two-M3 through-bolt support; no gripper/claw/large palm",
            "static_exact_brep_positive_interferences": brep["positive_interference_count"],
            "cad_standing_height_mm": brep.get("standing_height_mm"),
        },
        "sensors": {
            "camera": "M5Stack UnitV2 GC2145 envelope in removable head cradle",
            "microphone": "M5Stack UnitV2 integrated microphone",
            "torso_imu_frame": "torso_imu_frame",
            "foot_touch_sites": [f"{side}_sole_{fore_aft}_{lateral}" for side in ("left", "right") for fore_aft in ("front", "rear") for lateral in ("medial", "lateral")],
        },
        "model_contract": {
            "mjcf_sha256": mjcf.get("mjcf_sha256"),
            "frame_convention": model_contract.get("frame_convention"),
            "compiled_joint_order": model_contract.get("compiled_joint_order", []),
            "actuator_order": model_contract.get("actuator_order", []),
            "index_mapping": model_contract.get("index_mapping", []),
            "recommended_training_reset_keyframe": model_contract.get("recommended_training_reset_keyframe"),
            "calibration_zero_keyframe": model_contract.get("calibration_zero_keyframe"),
            "contract_gate": model_contract.get("overall", "UNKNOWN"),
            "physical_zero_verification": "HOLD_NATIVE_SOLIDWORKS_AND_POWERED_CALIBRATION_REQUIRED",
        },
        "payload_reserved_envelopes_mm": {
            "compute": [70, 12, 32],
            "battery": [75, 22, 34],
            "torso_imu": [32, 8, 25],
            "m5stack_unitv2": [48, 18.5, 24],
        },
        "sysid": {
            "architecture_reference": "ToddlerBot-style MJX identification workflow",
            "identify": [
                "per-joint direction/zero/safe count range",
                "torque gain and deadband at multiple supply voltages",
                "viscous and Coulomb friction",
                "reflected armature/rotor inertia",
                "command-to-motion latency and jitter",
                "backlash/hysteresis",
                "battery voltage sag and thermal derating",
            ],
            "recommended_sequence": [
                "one actuator at a time bus scan and low-speed jog",
                "unloaded chirp and step response",
                "known-mass pendulum/swing test",
                "paired-leg quasi-static torque/current sweep",
                "whole-body stance residual fitting",
            ],
        },
        "domain_randomization_start": {
            "link_mass_scale": [0.90, 1.10],
            "full_inertia_scale": [0.80, 1.20],
            "joint_damping_scale": [0.60, 1.50],
            "coulomb_friction_scale": [0.50, 1.80],
            "motor_strength_scale": [0.80, 1.05],
            "control_latency_s": [0.0, 0.035],
            "backlash_rad": [0.0, 0.035],
            "battery_voltage_v": [10.5, 12.6],
            "ground_friction": [0.60, 1.40],
        },
        "verified_gates": {
            "exact_brep_static_interference": brep["overall"],
            "urdf_mass_inertia_graph_limits_contacts": mass["overall"],
            "mujoco_runtime_compile": mjcf["runtime_compile_gate"],
            "mjcf_index_contract": model_contract.get("overall", "UNKNOWN"),
            "gait_neutral_keyframe": model_contract.get("keyframe_gates", {}).get("gait_neutral", {}).get("overall", "UNKNOWN"),
            "foot_site_semantics": "PASS" if model_contract.get("foot_site_semantics_pass") else "FAIL",
            "solidworks_static": current_gate(SW_GATE, manifest_sha),
            "solidworks_all_joints_isolated_motion": current_gate(SW_MOTION_GATE, manifest_sha),
        },
        "training_readiness": "DIGITAL_RL_BASELINE_PASS_NATIVE_SOLIDWORKS_HOLD_PHYSICAL_FIRST_ARTICLE_HOLD",
        "holds": [
            "native SOLIDWORKS static and isolated full-limit Motion rerun must pass on the current manifest",
            "print one STS3250 carrier/output-stack and both wrist/sole coupons before full-set printing",
            "weigh the first article and identify each link COM/full inertia; replace the engineering ledger",
            "complete the hardware calibration CSV before real-robot torque enable",
            "confirm physical knee/ankle zero and hard stops in native SOLIDWORKS and on the powered first article",
            "repeat collision/strain-relief sweep with the as-built cable harness",
        ],
    }
    HANDOFF.write_text(json.dumps(handoff, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    digital_pass = all(
        handoff["verified_gates"][name] == "PASS"
        for name in (
            "exact_brep_static_interference",
            "urdf_mass_inertia_graph_limits_contacts",
            "mujoco_runtime_compile",
        )
    )
    native_sw_pass = all(
        handoff["verified_gates"][name] == "PASS"
        for name in ("solidworks_static", "solidworks_all_joints_isolated_motion")
    )
    release = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.release_gate.v1",
        "manifest_sha256": manifest_sha,
        "digital_rl_baseline": "PASS" if digital_pass else "HOLD",
        "native_solidworks_motion": "PASS" if native_sw_pass else "HOLD",
        "physical_first_article": "HOLD",
        "movable_joint_count": 16,
        "component_count": manifest["component_count"],
        "exact_sts3250_count": manifest["blue_sts3250_count"],
        "nominal_total_mass_kg": handoff["nominal_total_mass_kg"],
        "cad_envelope_height_mm": brep.get("standing_height_mm"),
        "mjcf_standing_kinematic_height_m": mjcf.get("standing_height_m"),
        "gates": handoff["verified_gates"],
        "truth_boundary": (
            "PASS releases only the generated URDF/MJCF and associated engineering ledger to digital RL. "
            "It does not certify native SOLIDWORKS Motion, printing, assembly, electrical safety, "
            "purchased-part tolerances or real-robot operation."
        ),
        "overall": "DIGITAL_RL_PASS_WITH_NATIVE_AND_PHYSICAL_HOLDS" if digital_pass else "HOLD",
    }
    RELEASE_GATE.parent.mkdir(parents=True, exist_ok=True)
    RELEASE_GATE.write_text(json.dumps(release, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({
        "actuator_count": len(rows),
        "nominal_total_mass_kg": handoff["nominal_total_mass_kg"],
        "urdf_gate": handoff["verified_gates"]["urdf_mass_inertia_graph_limits_contacts"],
        "mujoco_gate": handoff["verified_gates"]["mujoco_runtime_compile"],
        "solidworks_motion_gate": handoff["verified_gates"]["solidworks_all_joints_isolated_motion"],
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
