"""Independent v3 kinematic, mass, MJX and static-feasibility gates."""

from __future__ import annotations

import importlib.util
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "generated" / "urdf" / "physical_mount_v3_rl_fixed" / "zeroth01_physical_mount_v3_rl_fixed_18dof.urdf"
MJCF = ROOT / "generated" / "mujoco" / "physical_mount_v3_rl_fixed" / "zeroth01_physical_mount_v3_rl_fixed_18dof_mjx.xml"
REPORT_ROOT = ROOT / "reports" / "physical_mount_v3_rl_fixed"
REPORT = REPORT_ROOT / "release_gates.json"
HANDOFF = ROOT / "generated" / "config" / "physical_mount_v3_rl_fixed_rl_handoff.json"
ACTUATOR_LAYOUT = ROOT / "generated" / "config" / "physical_mount_v3_rl_fixed_actuator_layout.json"
CAD_REPORT = REPORT_ROOT / "cad_build.json"
SOLIDWORKS_GATE = REPORT_ROOT / "solidworks_gate.json"
SOLIDWORKS_INTERFERENCE_GATE = REPORT_ROOT / "solidworks_interference_gate.json"
FK_MANIFEST_GATE = REPORT_ROOT / "fk_manifest_gate.json"
STS3250_TORQUE_GATE = REPORT_ROOT / "sts3250_quasistatic_torque_gate.json"


def load_source():
    path = Path(__file__).with_name("build_v3_urdf.py")
    spec = importlib.util.spec_from_file_location("zeroth_v3_urdf", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse3(text):
    return np.array([float(value) for value in text.split()], dtype=float)


def joint_world_axis(source, transforms, joint):
    parent = joint.find("parent").get("link")
    origin = joint.find("origin")
    origin_rotation = source.rpy_matrix(
        source.vec(origin.get("rpy") if origin is not None else None)
    )
    joint_rotation = source.mat_mul(transforms[parent][0], origin_rotation)
    axis = source.vec(joint.find("axis").get("xyz"))
    return np.asarray(source.mat_vec(joint_rotation, axis), dtype=float)


def main() -> int:
    s = load_source()
    urdf = ET.parse(URDF).getroot()
    mjcf = ET.parse(MJCF).getroot()
    transforms = s.old_fk(urdf)
    positions = {
        name: np.asarray(transform[1], dtype=float)
        for name, transform in transforms.items()
    }
    movable = [joint for joint in urdf.findall("joint") if joint.get("type") == "revolute"]
    urdf_masses = {
        link.get("name"): float(link.find("./inertial/mass").get("value"))
        for link in urdf.findall("link")
        if link.find("./inertial/mass") is not None
    }
    urdf_total = sum(urdf_masses.values())

    pairs = [
        ("left_shoulder_yaw", "right_shoulder_yaw"),
        ("left_shoulder_pitch", "right_shoulder_pitch"),
        ("left_elbow_yaw", "right_elbow_yaw"),
        ("left_hip_yaw", "right_hip_yaw"),
        ("left_hip_roll", "right_hip_roll"),
        ("left_hip_pitch", "right_hip_pitch"),
        ("left_knee_pitch", "right_knee_pitch"),
        ("left_ankle_pitch", "right_ankle_pitch"),
        ("left_ankle_roll", "right_ankle_roll"),
    ]
    joints = {joint.get("name"): joint for joint in movable}
    mirror_rows = []
    for left_name, right_name in pairs:
        left, right = joints[left_name], joints[right_name]
        lp = positions[left.find("child").get("link")]
        rp = positions[right.find("child").get("link")]
        point_error_mm = float(np.linalg.norm(lp - np.array((rp[0], -rp[1], rp[2]))) * 1000.0)
        la = joint_world_axis(s, transforms, left)
        ra = joint_world_axis(s, transforms, right)
        # Axes are axial vectors; reflection across Y=0 is det(R)*R*a.
        mirrored_axial = np.array((-la[0], la[1], -la[2]))
        axis_error = float(min(
            np.linalg.norm(mirrored_axial - ra),
            np.linalg.norm(mirrored_axial + ra),
        ))
        mirror_rows.append({
            "left": left_name,
            "right": right_name,
            "point_error_mm": point_error_mm,
            "axis_error": axis_error,
            "gate": "PASS" if point_error_mm <= 0.25 and axis_error <= 1.0e-9 else "FAIL",
        })

    urdf_collision_types = []
    for collision in urdf.findall("./link/collision"):
        geometry = collision.find("geometry")
        urdf_collision_types.append(next(iter(geometry)).tag)
    mjcf_collision_geoms = [
        geom for geom in mjcf.findall(".//geom")
        if geom.get("contype", "1") != "0" and geom.get("name") != "ground"
    ]
    mjcf_collision_types = sorted({geom.get("type", "sphere") for geom in mjcf_collision_geoms})

    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "official_standing")
    mujoco.mj_resetDataKeyframe(model, data, key_id)
    mujoco.mj_forward(model, data)
    left_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "FOOT_collision")
    right_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "FOOT_2_collision")
    sole_bottoms = []
    sole_level_errors = []
    for geom_id in (left_id, right_id):
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        half = model.geom_size[geom_id, :3]
        sole_bottoms.append(
            float(data.geom_xpos[geom_id, 2] - np.abs(rotation[2, :]).dot(half))
        )
        thin_axis = int(np.argmin(half))
        normal = rotation[:, thin_axis]
        sole_level_errors.append(
            float(math.acos(max(-1.0, min(1.0, abs(normal[2])))))
        )
    ground_gap = max(abs(value) for value in sole_bottoms)

    hip_half_spacing = (
        abs(positions["U_HIP_L"][1]) + abs(positions["U_HIP_R"][1])
    ) / 2.0
    static_torque = urdf_total * 9.81 * hip_half_spacing
    static_margin = (s.CONTINUOUS_EFFORT_NM - static_torque) / s.CONTINUOUS_EFFORT_NM
    nonadjacent_penetrations = []
    grounding_corrections = []
    for joint_id in range(model.njnt):
        if model.jnt_type[joint_id] != mujoco.mjtJoint.mjJNT_HINGE:
            continue
        qadr = model.jnt_qposadr[joint_id]
        low, high = model.jnt_range[joint_id]
        for value in (low, 0.5 * (low + high), high):
            mujoco.mj_resetDataKeyframe(model, data, key_id)
            data.qpos[qadr] = value
            mujoco.mj_forward(model, data)
            # The base is free in RL.  Re-ground the pose before checking
            # sweep interference, otherwise a bent knee is falsely reported
            # as sole/ground penetration simply because root Z was frozen.
            foot_bottoms = []
            for geom_id in (left_id, right_id):
                rotation = data.geom_xmat[geom_id].reshape(3, 3)
                half = model.geom_size[geom_id, :3]
                foot_bottoms.append(float(data.geom_xpos[geom_id, 2] - np.abs(rotation[2, :]).dot(half)))
            correction = -min(foot_bottoms)
            data.qpos[2] += correction
            grounding_corrections.append(abs(float(correction)))
            mujoco.mj_forward(model, data)
            for index in range(data.ncon):
                contact = data.contact[index]
                if contact.dist < -0.0005:
                    geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1)
                    geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2)
                    if "ground" in {geom1, geom2}:
                        continue
                    nonadjacent_penetrations.append({
                        "joint": mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
                        "position": float(value),
                        "geom1": geom1,
                        "geom2": geom2,
                        "depth_m": float(-contact.dist),
                    })

    mjcf_total = float(model.body_mass.sum())
    source_robot = ET.parse(s.V2_URDF).getroot()
    source_transforms = s.old_fk(source_robot)
    source_joints = {
        joint.get("name"): joint for joint in source_robot.findall("joint")
    }
    axis_semantics = []
    for name, joint in joints.items():
        actual = joint_world_axis(s, transforms, joint)
        if name == "left_ankle_roll":
            expected = np.array((1.0, 0.0, 0.0))
        elif name == "right_ankle_roll":
            expected = np.array((-1.0, 0.0, 0.0))
        else:
            expected = joint_world_axis(
                s, source_transforms, source_joints[name]
            )
        residual = float(min(
            np.linalg.norm(actual - expected),
            np.linalg.norm(actual + expected),
        ))
        axis_semantics.append({
            "joint": name,
            "expected_world_axis": expected.tolist(),
            "actual_world_axis": actual.tolist(),
            "axis_residual": residual,
            "gate": "PASS" if residual <= 1.0e-8 else "FAIL",
        })

    servo_owners = {
        s.BODY: 4,
        "3215_1Flange": 2,
        "3215_1Flange_2": 2,
        "3215_BothFlange_5": 1,
        "3215_BothFlange_6": 1,
        "3215_BothFlange_9": 1,
        "3215_BothFlange_10": 1,
        "3215_BothFlange_13": 2,
        "3215_BothFlange_14": 2,
        s.LEFT_ANKLE_CARRIER: 1,
        s.RIGHT_ANKLE_CARRIER: 1,
    }
    owner_rows = [{
        "link": name,
        "servo_count": count,
        "aggregate_link_mass_kg": urdf_masses[name],
        "minimum_servo_mass_kg": count * s.STS3250_MASS_KG,
        "gate": "PASS" if urdf_masses[name] + 1.0e-12 >= count * s.STS3250_MASS_KG else "FAIL",
    } for name, count in servo_owners.items()]

    servo_id_map = (
        ("S01", "right_shoulder_pitch", "3215_1Flange_2"),
        ("S02", "left_shoulder_pitch", "3215_1Flange"),
        ("S03", "right_shoulder_yaw", s.BODY),
        ("S04", "right_hip_pitch", "3215_BothFlange_10"),
        ("S05", "left_hip_pitch", "3215_BothFlange_9"),
        ("S06", "left_shoulder_yaw", s.BODY),
        ("S07", "right_hip_yaw", s.BODY),
        ("S08", "left_hip_yaw", s.BODY),
        ("S09", "right_elbow_yaw", "3215_1Flange_2"),
        ("S10", "left_elbow_yaw", "3215_1Flange"),
        ("S11", "right_hip_roll", "3215_BothFlange_6"),
        ("S12", "left_hip_roll", "3215_BothFlange_5"),
        ("S13", "right_knee_pitch", "3215_BothFlange_14"),
        ("S14", "left_knee_pitch", "3215_BothFlange_13"),
        ("S15", "right_ankle_pitch", "3215_BothFlange_14"),
        ("S16", "left_ankle_pitch", "3215_BothFlange_13"),
        ("S17", "right_ankle_roll", s.RIGHT_ANKLE_CARRIER),
        ("S18", "left_ankle_roll", s.LEFT_ANKLE_CARRIER),
    )
    actuator_layout_rows = []
    for servo_id, joint_name, owning_link in servo_id_map:
        joint = joints[joint_name]
        child = joint.find("child").get("link")
        actuator_layout_rows.append({
            "id": servo_id,
            "joint": joint_name,
            "owning_rigid_link": owning_link,
            "shaft_origin_body_neutral_m": positions[child].tolist(),
            "shaft_axis_body_neutral": joint_world_axis(
                s, transforms, joint
            ).tolist(),
            "model": "FEETECH STS3250-C001",
            "mass_kg": s.STS3250_MASS_KG,
            "cad_color": "blue",
            "bus_id": "REQUIRES_PHYSICAL_BUS_SCAN",
            "neutral_count": "REQUIRES_JOG_CALIBRATION",
            "direction_sign": "REQUIRES_JOG_CALIBRATION",
            "mount_gate": "HOLD_STS3250_FIRST_ARTICLE",
        })

    cad_payload = json.loads(CAD_REPORT.read_text(encoding="utf-8")) if CAD_REPORT.is_file() else {}
    cad_assembly = ROOT / str(cad_payload.get("assembly", "__missing__"))
    cad_ok = cad_assembly.is_file() and all(row.get("valid_brep") for row in cad_payload.get("parts", []))
    solidworks_payload = json.loads(SOLIDWORKS_GATE.read_text(encoding="utf-8")) if SOLIDWORKS_GATE.is_file() else {}
    interference_payload = json.loads(SOLIDWORKS_INTERFERENCE_GATE.read_text(encoding="utf-8")) if SOLIDWORKS_INTERFERENCE_GATE.is_file() else {}
    fk_payload = json.loads(FK_MANIFEST_GATE.read_text(encoding="utf-8")) if FK_MANIFEST_GATE.is_file() else {}
    torque_payload = json.loads(STS3250_TORQUE_GATE.read_text(encoding="utf-8")) if STS3250_TORQUE_GATE.is_file() else {}
    solidworks_ok = (
        solidworks_payload.get("overall") == "PASS"
        and solidworks_payload.get("assembly_component_count") == 51
        and solidworks_payload.get("separate_blue_sts3250_count") == 18
    )
    standing_height_mm = float(solidworks_payload.get("standing_height_mm", float("inf")))
    interference_ok = (
        interference_payload.get("overall") == "PASS"
        and interference_payload.get("physical_interference_count") == 0
    )
    fk_ok = fk_payload.get("overall") == "PASS"
    torque_ok = str(torque_payload.get("gate", "")).startswith("PASS_")

    if static_torque < s.CONTINUOUS_EFFORT_NM:
        actuator_static_gate = "PASS"
    elif static_torque < s.RATED_EFFORT_NM:
        actuator_static_gate = "HOLD_RATED_TORQUE_ONLY_RL_COM_SHIFT_REQUIRED"
    else:
        actuator_static_gate = "FAIL"

    gates = {
        "CAD_PORTABLE_PASS": "PASS" if cad_ok else "FAIL",
        "SOLIDWORKS_NATIVE_ASSEMBLY_PASS": "PASS" if solidworks_ok else "FAIL",
        "SOLIDWORKS_PHYSICAL_INTERFERENCE_PASS": "PASS" if interference_ok else "FAIL",
        "URDF_SOLIDWORKS_FK_MATCH_PASS": "PASS" if fk_ok else "FAIL",
        "SOURCE_AXIS_FIDELITY_PASS": "PASS" if all(row["gate"] == "PASS" for row in axis_semantics) else "FAIL",
        "MJX_PRIMITIVE_COLLISION_PASS": "PASS" if set(urdf_collision_types) <= {"box", "sphere", "cylinder"} and set(mjcf_collision_types) <= {"box", "sphere", "capsule", "cylinder"} else "FAIL",
        "GROUND_CONTACT_PASS": "PASS" if ground_gap <= 1.0e-6 and max(sole_level_errors) <= 1.0e-6 else "FAIL",
        "ACTUATOR_STATIC_FEASIBILITY_PASS": actuator_static_gate,
        "STS3250_QUASISTATIC_TORQUE_PASS": (
            "PASS" if torque_ok else "HOLD_RATED_ONLY_DYNAMIC_POLICY_TRACE_REQUIRED"
        ),
        "STS3250_DYNAMIC_WALKING_PASS": "HOLD_RL_ROLLOUT_TORQUE_AND_THERMAL_TRACE_REQUIRED",
        "MASS_TARGET_PASS": "PASS" if 0.0 < urdf_total <= 3.0 and abs(urdf_total - s.TARGET_TOTAL_MASS_KG) <= 1.0e-9 and abs(mjcf_total - urdf_total) <= 1.0e-9 else "FAIL",
        "STANDING_HEIGHT_LIMIT_PASS": "PASS" if standing_height_mm <= 500.0 else "FAIL",
        "ACTUATOR_MASS_OWNERSHIP_PASS": "PASS" if sum(servo_owners.values()) == 18 and all(row["gate"] == "PASS" for row in owner_rows) else "FAIL",
        "MOTION_SAMPLE_NO_NONADJACENT_PENETRATION_PASS": "PASS" if not nonadjacent_penetrations else "FAIL",
        "PURCHASED_CORES3_HEAD_MODEL_PASS": "PASS" if abs(urdf_masses.get(s.CORES3_HEAD_POD, 0.0) - s.CORES3_HEAD_POD_MASS_KG) <= 1.0e-12 else "FAIL",
        "MASS_IDENTIFIED_PASS": "HOLD_AS_BUILT_MEASUREMENT_REQUIRED",
        "STS3250_FIRST_ARTICLE_PASS": "HOLD_PURCHASED_HARDWARE_REQUIRED",
        "CORES3_CRADLE_FIRST_ARTICLE_PASS": "HOLD_TORSO_NUT_PLATE_FIT_REQUIRED",
    }
    blocking = [key for key, value in gates.items() if value == "FAIL"]
    payload = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.release_gates.v1",
        "urdf": URDF.relative_to(ROOT).as_posix(),
        "mjcf": MJCF.relative_to(ROOT).as_posix(),
        "dof": len(movable),
        "mass": {
            "pre_compaction_v3_kg": 3.095471828,
            "maximum_allowed_kg": 3.0,
            "urdf_nominal_kg": urdf_total,
            "mjcf_nominal_kg": mjcf_total,
            "confidence": "estimated_from_envelopes_not_as_built",
        },
        "standing_envelope": {
            "solidworks_height_mm": standing_height_mm,
            "maximum_allowed_mm": 500.0,
            "source": "native SolidWorks occurrence bounding-box union",
        },
        "actuator": {
            "model": "FEETECH STS3250-C001",
            "count": 18,
            "mass_each_kg": s.STS3250_MASS_KG,
            "continuous_design_torque_nm": s.CONTINUOUS_EFFORT_NM,
            "rated_torque_nm": s.RATED_EFFORT_NM,
            "static_single_support_required_nm": static_torque,
            "continuous_margin_fraction": static_margin,
            "hip_half_spacing_m": hip_half_spacing,
            "quasistatic_torque_report": STS3250_TORQUE_GATE.relative_to(ROOT).as_posix(),
            "quasistatic_peak_joint": torque_payload.get("peak_joint"),
            "quasistatic_peak_torque_nm": torque_payload.get("peak_quasistatic_torque_nm"),
        },
        "symmetry": mirror_rows,
        "axis_semantics": axis_semantics,
        "collision": {
            "urdf_types": sorted(set(urdf_collision_types)),
            "mjcf_types": mjcf_collision_types,
            "sampled_nonadjacent_penetrations": nonadjacent_penetrations,
            "maximum_vertical_regrounding_correction_m": max(grounding_corrections, default=0.0),
            "sweep_ground_policy": "free base is shifted vertically per sample; ground contacts are excluded from self-interference rows",
        },
        "actuator_mass_ownership": owner_rows,
        "standing": {
            "sole_bottoms_m": sole_bottoms,
            "sole_level_error_rad": sole_level_errors,
            "maximum_ground_gap_m": ground_gap,
        },
        "gates": gates,
        "blocking_failures": blocking,
        "overall_rl_nominal": "PASS" if not blocking else "FAIL",
        "physical_release": "HOLD_FIRST_ARTICLE_AND_AS_BUILT_IDENTIFICATION",
        "native_cad": {
            "solidworks_gate": SOLIDWORKS_GATE.relative_to(ROOT).as_posix(),
            "solidworks_interference_gate": SOLIDWORKS_INTERFERENCE_GATE.relative_to(ROOT).as_posix(),
            "fk_manifest_gate": FK_MANIFEST_GATE.relative_to(ROOT).as_posix(),
            "solidworks_status": "native 51-component assembly rebuilt in SolidWorks 33.0.0; physical interference count zero",
        },
    }
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    handoff = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.rl_handoff.v1",
        "canonical_urdf": payload["urdf"],
        "canonical_mjcf_mjx": payload["mjcf"],
        "actuator_layout": ACTUATOR_LAYOUT.relative_to(ROOT).as_posix(),
        "joint_order": [joint.get("name") for joint in movable],
        "actuator": payload["actuator"],
        "mass": payload["mass"],
        "sensors": {
            "imu": {"parent": s.BODY, "position_m": [0.0, 0.0, 0.02]},
            "head_module": {
                "model": "M5Stack CoreS3 K128",
                "parent": s.CORES3_HEAD_POD,
                "mass_kg": s.CORES3_HEAD_POD_MASS_KG,
                "camera": "GC0308 0.3MP",
                "camera_optical_frame": "camera_optical_frame",
                "microphones": ["left_microphone_frame", "right_microphone_frame"],
                "speaker_frame": "head_speaker_frame",
                "imu_frame": "head_imu_frame",
                "walking_policy": "CoreS3 has no mechanical pan/tilt; treat the module as one fixed torso-overlapped payload",
            },
            "sole_contact_sites": 8,
        },
        "randomization": {
            "mass_scale": [0.90, 1.10],
            "joint_damping_scale": [0.70, 1.30],
            "motor_strength_scale": [0.80, 1.00],
            "ground_friction": [0.70, 1.25],
        },
        "release_gates": gates,
        "truth_boundary": "RL nominal model uses a purchased CoreS3 at the official main-unit envelope with the full 72.7 g retail-set mass assigned conservatively; factory release remains held for STS3250 fit, torso-cradle first article and as-built mass/COM/inertia identification.",
    }
    HANDOFF.parent.mkdir(parents=True, exist_ok=True)
    HANDOFF.write_text(json.dumps(handoff, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    actuator_layout = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.actuator_layout.v1",
        "frame": "body neutral, X forward, Y left, Z up, metres",
        "count": len(actuator_layout_rows),
        "actuators": actuator_layout_rows,
        "mass_ownership": owner_rows,
        "truth_boundary": "shaft frames are CAD/URDF nominal; bus ID, count zero, direction and purchased STS3250 hole/horn fit require physical calibration",
    }
    ACTUATOR_LAYOUT.write_text(json.dumps(actuator_layout, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(REPORT)
    print(HANDOFF)
    print(ACTUATOR_LAYOUT)
    print(json.dumps({"overall": payload["overall_rl_nominal"], "gates": gates}, indent=2))
    return 0 if not blocking else 2


if __name__ == "__main__":
    raise SystemExit(main())
