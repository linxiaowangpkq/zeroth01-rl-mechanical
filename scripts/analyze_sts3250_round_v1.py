from __future__ import annotations

import csv
import json
from pathlib import Path

import mujoco
import numpy as np


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
ROUND_MJCF = ROOT / "generated" / "mujoco" / "zeroth01_rl_round_v1.xml"
BASE_MJCF = ROOT / "generated" / "mujoco" / "zeroth01_rl_ready.xml"
ACTUATOR_METADATA = (
    ROOT / "generated" / "config" / "zeroth01_actuator_metadata.json"
)
PROFILE_OUTPUT = (
    ROOT / "generated" / "config" / "sts3250_round_v1_rl_profiles.json"
)
REPORT_JSON = ROOT / "reports" / "sts3250_round_v1_feasibility.json"
REPORT_CSV = ROOT / "reports" / "sts3250_round_v1_static_torque.csv"
REPORT_MD = ROOT / "reports" / "sts3250_round_v1_feasibility.md"

SAMPLES = 100_000
SEED = 20260729


def joint_data(model: mujoco.MjModel) -> list[dict[str, object]]:
    result = []
    for joint_id in range(model.njnt):
        if int(model.jnt_type[joint_id]) != int(
            mujoco.mjtJoint.mjJNT_HINGE
        ):
            continue
        result.append(
            {
                "id": joint_id,
                "name": mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_JOINT, joint_id
                ),
                "qpos": int(model.jnt_qposadr[joint_id]),
                "dof": int(model.jnt_dofadr[joint_id]),
                "lower": float(model.jnt_range[joint_id, 0]),
                "upper": float(model.jnt_range[joint_id, 1]),
            }
        )
    if len(result) != 16:
        raise ValueError(f"expected 16 hinges, got {len(result)}")
    return result


def bias_torque(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joints: list[dict[str, object]],
    qpos: np.ndarray,
) -> np.ndarray:
    mujoco.mj_resetData(model, data)
    data.qpos[:] = qpos
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    mujoco.mj_forward(model, data)
    return np.array(
        [float(data.qfrc_bias[int(joint["dof"])]) for joint in joints],
        dtype=float,
    )


def pose_summary(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joints: list[dict[str, object]],
    qpos: np.ndarray,
) -> dict[str, float]:
    values = bias_torque(model, data, joints, qpos)
    return {
        str(joint["name"]): float(value)
        for joint, value in zip(joints, values)
    }


def sample_bias(
    model: mujoco.MjModel,
    joints: list[dict[str, object]],
    normalized_samples: np.ndarray,
) -> np.ndarray:
    data = mujoco.MjData(model)
    values = np.zeros((len(normalized_samples), len(joints)), dtype=float)
    for row, normalized in enumerate(normalized_samples):
        mujoco.mj_resetData(model, data)
        for column, joint in enumerate(joints):
            lower = float(joint["lower"])
            upper = float(joint["upper"])
            data.qpos[int(joint["qpos"])] = lower + normalized[column] * (
                upper - lower
            )
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        mujoco.mj_forward(model, data)
        values[row, :] = [
            float(data.qfrc_bias[int(joint["dof"])]) for joint in joints
        ]
    return values


def main() -> None:
    metadata = json.loads(ACTUATOR_METADATA.read_text(encoding="utf-8"))
    manufacturer = metadata["manufacturer"]
    rated = float(manufacturer["rated_torque_nm_at_12v"])
    stall = float(manufacturer["stall_torque_nm_at_12v"])
    no_load_speed = float(
        manufacturer["no_load_speed_rad_s_at_12v"]
    )
    sim_limit = float(
        metadata["training_policy"][
            "urdf_and_official_sim_peak_command_limit_nm"
        ]
    )
    conservative = 0.8 * rated

    round_model = mujoco.MjModel.from_xml_path(str(ROUND_MJCF))
    base_model = mujoco.MjModel.from_xml_path(str(BASE_MJCF))
    round_joints = joint_data(round_model)
    base_joints = joint_data(base_model)
    round_names = [str(item["name"]) for item in round_joints]
    if round_names != [str(item["name"]) for item in base_joints]:
        raise ValueError("baseline and round-v1 joint orders differ")

    rng = np.random.default_rng(SEED)
    normalized = rng.random((SAMPLES, len(round_joints)))
    round_values = sample_bias(round_model, round_joints, normalized)
    base_values = sample_bias(base_model, base_joints, normalized)
    round_abs = np.abs(round_values)
    base_abs = np.abs(base_values)

    round_data = mujoco.MjData(round_model)
    neutral = round_model.qpos0.copy()
    standing = round_model.key_qpos[0].copy()
    neutral_torque = pose_summary(
        round_model, round_data, round_joints, neutral
    )
    standing_torque = pose_summary(
        round_model, round_data, round_joints, standing
    )

    rows: list[dict[str, object]] = []
    for index, joint in enumerate(round_joints):
        name = str(joint["name"])
        statistics = {
            "p50": float(np.percentile(round_abs[:, index], 50)),
            "p95": float(np.percentile(round_abs[:, index], 95)),
            "p99": float(np.percentile(round_abs[:, index], 99)),
            "max": float(np.max(round_abs[:, index])),
        }
        baseline_max = float(np.max(base_abs[:, index]))
        rows.append(
            {
                "joint": name,
                "neutral_gravity_torque_nm": (
                    f"{neutral_torque[name]:.9f}"
                ),
                "official_standing_gravity_torque_nm": (
                    f"{standing_torque[name]:.9f}"
                ),
                "random_static_abs_p50_nm": f"{statistics['p50']:.9f}",
                "random_static_abs_p95_nm": f"{statistics['p95']:.9f}",
                "random_static_abs_p99_nm": f"{statistics['p99']:.9f}",
                "random_static_abs_max_nm": f"{statistics['max']:.9f}",
                "baseline_random_static_abs_max_nm": (
                    f"{baseline_max:.9f}"
                ),
                "round_minus_baseline_max_nm": (
                    f"{statistics['max'] - baseline_max:.9f}"
                ),
                "max_over_conservative_80pct_rated": (
                    f"{statistics['max'] / conservative:.9f}"
                ),
                "max_over_manufacturer_rated": (
                    f"{statistics['max'] / rated:.9f}"
                ),
                "max_over_legacy_sim_limit": (
                    f"{statistics['max'] / sim_limit:.9f}"
                ),
                "static_gravity_rated_gate": (
                    "PASS" if statistics["max"] <= rated else "FAIL"
                ),
                "walking_dynamic_gate": (
                    "UNVERIFIED_REQUIRES_TRAINED_POLICY_TORQUE_SPEED_THERMAL_"
                    "ROLLOUT"
                ),
            }
        )

    overall_max_index = np.unravel_index(
        int(np.argmax(round_abs)), round_abs.shape
    )
    overall_max = float(round_abs[overall_max_index])
    overall_joint = round_names[int(overall_max_index[1])]
    static_gate = all(
        row["static_gravity_rated_gate"] == "PASS" for row in rows
    )
    payload = {
        "schema": "zeroth01.sts3250.round_v1.feasibility.v1",
        "mujoco_version": mujoco.__version__,
        "round_model": str(ROUND_MJCF),
        "baseline_model": str(BASE_MJCF),
        "sample_count": SAMPLES,
        "seed": SEED,
        "sampling_scope": (
            "deterministic uniform poses inside the guarded 16-DoF joint "
            "box, qvel=qacc=0; qfrc_bias is a quasi-static gravity/coupling "
            "load with the floating base acceleration constrained to zero"
        ),
        "round_v1_total_mass_kg": float(np.sum(round_model.body_mass)),
        "baseline_total_mass_kg": float(np.sum(base_model.body_mass)),
        "manufacturer": manufacturer,
        "training_profiles": {
            "conservative_thermal_start": {
                "torque_limit_nm": conservative,
                "derivation": "80% of manufacturer rated torque",
                "hardware_claim": "engineering_starting_point_not_tested",
            },
            "manufacturer_rated_evaluation": {
                "torque_limit_nm": rated,
                "hardware_claim": "manufacturer_rated_at_12V",
            },
            "legacy_official_sim": {
                "torque_limit_nm": sim_limit,
                "ratio_to_rated": sim_limit / rated,
                "hardware_claim": (
                    "simulation_parameter_not_continuous_hardware_rating"
                ),
            },
            "manufacturer_stall_boundary": {
                "torque_limit_nm": stall,
                "hardware_claim": (
                    "stall_boundary_never_use_as_continuous_RL_limit"
                ),
            },
        },
        "overall_random_static_abs_max_nm": overall_max,
        "overall_random_static_abs_max_joint": overall_joint,
        "overall_static_gravity_rated_gate": (
            "PASS" if static_gate else "FAIL"
        ),
        "walking_feasibility_gate": (
            "UNVERIFIED: static gravity margin passes, but no trained gait "
            "rollout, torque-speed envelope, bus-voltage sag, thermal model, "
            "impact load, backlash or physical endurance data exists"
        ),
        "required_policy_acceptance": {
            "default_training_torque_cap_nm": conservative,
            "evaluate_again_at_rated_cap_nm": rated,
            "reject_or_retrain_if": [
                "joint torque exceeds rated cap in sustained windows",
                "commanded speed/torque exceeds identified torque-speed curve",
                "RMS current or winding/case temperature exceeds bench limits",
                "foot impact, bus voltage sag, or tracking error exceeds test limits",
            ],
            "required_rollout_exports": [
                "per-joint torque p50/p95/p99/max and RMS",
                "per-joint speed p50/p95/p99/max",
                "torque-speed scatter versus identified envelope",
                "contact impulse and foot slip",
                "bus voltage/current and servo temperature",
            ],
        },
        "joint_rows": rows,
    }

    profile = {
        "schema": "zeroth01.sts3250.rl_profiles.v1",
        "servo_model": "FEETECH STS3250",
        "nominal_voltage_v": 12.0,
        "encoder_counts_per_revolution": int(
            manufacturer["encoder_counts_per_revolution"]
        ),
        "neutral_count_nominal": int(
            manufacturer["neutral_count_nominal"]
        ),
        "no_load_speed_rad_s": no_load_speed,
        "urdf_velocity_limit_rad_s": 5.0,
        "profiles": payload["training_profiles"],
        "simulation_dynamics": {
            "joint_damping_nm_s_rad": 0.53,
            "joint_frictionloss_nm": 0.001,
            "armature_kg_m2": 0.008793405204572328,
            "source": "official_stompymicro_simulation_baseline",
            "physical_identification_status": "NOT_IDENTIFIED",
        },
        "hardware_calibration_required": [
            "bus_id_scan",
            "joint_zero_offset_counts",
            "urdf_to_servo_direction_sign",
            "backlash_and_deadband",
            "loaded_torque_speed_curve",
            "current_and_temperature_limits",
            "command_latency_and_rate",
        ],
    }

    REPORT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    REPORT_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    PROFILE_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    PROFILE_OUTPUT.write_text(
        json.dumps(profile, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    REPORT_MD.write_text(
        "\n".join(
            [
                "# STS3250 / Zeroth-01 round-v1 feasibility",
                "",
                f"- Round-v1 nominal mass: `{payload['round_v1_total_mass_kg']:.6f} kg`",
                f"- Baseline mass: `{payload['baseline_total_mass_kg']:.6f} kg`",
                f"- Static samples: `{SAMPLES}`",
                (
                    "- Worst sampled quasi-static joint torque: "
                    f"`{overall_max:.6f} N.m` at `{overall_joint}`"
                ),
                (
                    "- Manufacturer rated / legacy simulation / stall: "
                    f"`{rated:.6f} / {sim_limit:.6f} / {stall:.6f} N.m`"
                ),
                (
                    "- Quasi-static gravity gate versus rated torque: "
                    f"**{'PASS' if static_gate else 'FAIL'}**"
                ),
                "- Walking feasibility: **UNVERIFIED**",
                "",
                (
                    "Static gravity margin does not validate walking. A trained "
                    "policy must pass torque-speed, RMS current, thermal, impact, "
                    "tracking-error and bus-voltage tests before STS3250 is "
                    "accepted for hardware walking."
                ),
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
