"""Generate actuator, sensor and RL handoff ledgers from v4 source artifacts."""

from __future__ import annotations

import json
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
URDF = ROOT / "generated" / "urdf" / "physical_mount_v4_original_minimal" / "zeroth01_physical_mount_v4_original_minimal_18dof.urdf"
MJCF = ROOT / "generated" / "mujoco" / "physical_mount_v4_original_minimal" / "zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml"
MASS_REPORT = ROOT / "reports" / "v4_original_minimal" / "urdf_mass_inertia_gate.json"
SW_GATE = ROOT / "reports" / "v4_original_minimal" / "solidworks_gate.json"
MOTION_GATE = ROOT / "reports" / "v4_original_minimal" / "coordinated_motion_evidence.json"
CONFIG_ROOT = ROOT / "generated" / "config"
ACTUATORS = CONFIG_ROOT / "physical_mount_v4_original_minimal_actuator_layout.json"
HANDOFF = CONFIG_ROOT / "physical_mount_v4_original_minimal_rl_handoff.json"


def main() -> int:
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    robot = ET.parse(URDF).getroot()
    joints = {str(joint.get("name")): joint for joint in robot.findall("joint")}
    rows = []
    for component in manifest["components"]:
        if component.get("role") not in {
            "dimension_controlled_sts3250",
            "purchased_exact_sts3250",
        }:
            continue
        component_id = str(component["component_id"])
        servo_id, remainder = component_id.split("_", 1)
        joint_name = remainder.removeprefix("STS3250_")
        transform = component["transform_local_mm_to_world_mm"]
        joint = joints[joint_name]
        limit = joint.find("limit")
        rows.append(
            {
                "id": servo_id,
                "joint": joint_name,
                "owning_rigid_link": component["owner_link"],
                "shaft_origin_body_neutral_m": [float(transform[index][3]) * 0.001 for index in range(3)],
                "shaft_axis_body_neutral": [float(transform[index][2]) for index in range(3)],
                "urdf_axis_joint_frame": [float(value) for value in joint.find("axis").get("xyz").split()],
                "limit_rad": [float(limit.get("lower")), float(limit.get("upper"))],
                "continuous_effort_limit_nm": float(limit.get("effort")),
                "velocity_limit_rad_s": float(limit.get("velocity")),
                "model": "FEETECH STS3250-C001",
                "cad_source": component.get("source", ""),
                "cad_fidelity": "purchased_exact_step",
                "mass_kg": 0.0745,
                "cad_color": "blue",
                "bus_id": "REQUIRES_PHYSICAL_BUS_SCAN",
                "neutral_count": "REQUIRES_JOG_CALIBRATION",
                "direction_sign": "REQUIRES_JOG_CALIBRATION",
                "mount_gate": "CAD_PASS_PHYSICAL_FIRST_ARTICLE_HOLD",
            }
        )
    rows.sort(key=lambda item: item["id"])
    if len(rows) != 18:
        raise RuntimeError(f"expected 18 actuator rows, got {len(rows)}")

    actuator_payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.actuator_layout.v1",
        "frame": "released body-neutral/world convention, X forward, Y left, Z up, metres",
        "count": len(rows),
        "actuators": rows,
        "total_actuator_mass_kg": sum(row["mass_kg"] for row in rows),
        "calibration_truth_boundary": "Bus IDs, neutral counts and direction signs cannot be inferred from CAD; fill them only after one-at-a-time powered jog calibration.",
    }
    CONFIG_ROOT.mkdir(parents=True, exist_ok=True)
    ACTUATORS.write_text(json.dumps(actuator_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    mass = json.loads(MASS_REPORT.read_text(encoding="utf-8"))
    sw = json.loads(SW_GATE.read_text(encoding="utf-8"))
    motion = json.loads(MOTION_GATE.read_text(encoding="utf-8"))
    handoff_payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.rl_handoff.v1",
        "robot": "Zeroth-01 v4 original-minimal 18DoF",
        "urdf": URDF.relative_to(ROOT).as_posix(),
        "mjcf_mjx": MJCF.relative_to(ROOT).as_posix(),
        "actuator_layout": ACTUATORS.relative_to(ROOT).as_posix(),
        "nominal_total_mass_kg": mass["nominal_total_mass_kg"],
        "mass_limit_kg": mass["hard_mass_limit_kg"],
        "solidworks_standing_height_mm": sw["standing_height_mm"],
        "movable_joint_count": len(rows),
        "actuator": {
            "model": "FEETECH STS3250-C001",
            "count": 18,
            "continuous_effort_nm": 1.2552512,
            "rated_effort_nm": 1.569064,
            "max_velocity_rad_s": 3.0,
            "mass_each_kg": 0.0745,
        },
        "sensors": {
            "camera_optical_frame": "M5Stack UnitV2 GC2145 camera; optical +Z is robot forward",
            "microphone_frame": "M5Stack UnitV2 integrated microphone",
            "torso_imu_frame": "torso_imu_frame; physical internal mounting remains a first-article HOLD",
            "foot_contacts": [
                f"{side}_sole_{corner}_contact"
                for side in ("left", "right")
                for corner in ("front_medial", "front_lateral", "rear_medial", "rear_lateral")
            ],
        },
        "payload_reserved_envelopes_mm": {
            "compute": [70, 12, 32],
            "battery": [75, 22, 34],
            "torso_imu": [32, 8, 25],
            "m5stack_unitv2": [48, 18.5, 24],
        },
        "domain_randomization_start": {
            "link_mass_scale": [0.90, 1.10],
            "diagonal_inertia_scale": [0.80, 1.20],
            "joint_damping_scale": [0.70, 1.30],
            "motor_strength_scale": [0.85, 1.05],
            "control_latency_s": [0.0, 0.030],
            "ground_friction": [0.60, 1.40],
        },
        "verified_gates": {
            "solidworks_static_interference": "PASS",
            "solidworks_height": sw["standing_height_gate"],
            "urdf_mass": mass["mass_gate"],
            "mujoco_coordinated_collision_sweep": motion["gate"],
        },
        "holds": [
            "freeze a real internal torso tray and cable routing before print release; the removed external rear pod is not an installed part",
            "weigh printed and purchased first article; update every link mass/COM/inertia",
            "one-at-a-time powered bus ID, zero and direction calibration",
            "quasi-static torque/current test before walking",
            "repeat full range sweep with as-built cable harness installed",
        ],
        "reference_architecture": "ToddlerBot-style MJX/SysID/domain-randomization workflow only; no ToddlerBot/KHR/TonyPi/Open Duck Mini geometry was copied.",
    }
    HANDOFF.write_text(json.dumps(handoff_payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(ACTUATORS)
    print(HANDOFF)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
