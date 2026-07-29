from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
DEFAULT_MJCF = ROOT / "generated" / "mujoco" / "zeroth01_rl_ready.xml"
REFERENCE_URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_ready.urdf"
REPORT_JSON = ROOT / "reports" / "mujoco_rl_ready_gate.json"
REPORT_CSV = ROOT / "reports" / "mujoco_rl_ready_joint_gate.csv"
REPORT_MD = ROOT / "reports" / "mujoco_rl_ready_gate.md"


def name(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    object_id: int,
) -> str:
    value = mujoco.mj_id2name(model, object_type, int(object_id))
    return value or f"{object_type.name}:{object_id}"


def self_collision_pairs(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    epsilon_m: float,
) -> set[str]:
    pairs: set[str] = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        if float(contact.dist) >= -epsilon_m:
            continue
        body1 = int(model.geom_bodyid[int(contact.geom1)])
        body2 = int(model.geom_bodyid[int(contact.geom2)])
        if body1 == 0 or body2 == 0 or body1 == body2:
            continue
        names = sorted(
            [
                name(model, mujoco.mjtObj.mjOBJ_BODY, body1),
                name(model, mujoco.mjtObj.mjOBJ_BODY, body2),
            ]
        )
        pairs.add(" :: ".join(names))
    return pairs


def rotation_distance(first: np.ndarray, second: np.ndarray) -> float:
    relative = first.T @ second
    cosine = float(
        np.clip((float(np.trace(relative)) - 1.0) * 0.5, -1.0, 1.0)
    )
    return math.acos(cosine)


def reset_neutral(
    model: mujoco.MjModel,
    data: mujoco.MjData,
) -> None:
    mujoco.mj_resetData(model, data)
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    data.ctrl[:] = 0.0


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Validate the floating-base Zeroth-01 MuJoCo RL package."
    )
    parser.add_argument("--mjcf", type=Path, default=DEFAULT_MJCF)
    parser.add_argument("--urdf", type=Path, default=REFERENCE_URDF)
    parser.add_argument(
        "--report-prefix",
        type=Path,
        help=(
            "Output prefix without extension. Defaults to the historical "
            "mujoco_rl_ready_gate report names."
        ),
    )
    parser.add_argument("--samples-per-joint", type=int, default=101)
    parser.add_argument("--random-samples", type=int, default=100000)
    parser.add_argument("--seed", type=int, default=20260729)
    parser.add_argument("--penetration-epsilon-mm", type=float, default=0.01)
    args = parser.parse_args()

    mjcf = args.mjcf.resolve()
    reference_urdf = args.urdf.resolve()
    if args.report_prefix is None:
        report_json = REPORT_JSON
        report_csv = REPORT_CSV
        report_md = REPORT_MD
    else:
        prefix = args.report_prefix.resolve()
        report_json = prefix.with_suffix(".json")
        report_csv = prefix.parent / f"{prefix.name}_joint_gate.csv"
        report_md = prefix.with_suffix(".md")
    model = mujoco.MjModel.from_xml_path(str(mjcf))
    data = mujoco.MjData(model)
    epsilon_m = args.penetration_epsilon_mm / 1000.0
    hinges = [
        joint_id
        for joint_id in range(model.njnt)
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_HINGE)
    ]
    if len(hinges) != 16:
        raise RuntimeError(f"expected 16 hinge joints, got {len(hinges)}")

    reset_neutral(model, data)
    mujoco.mj_forward(model, data)
    neutral_pairs = self_collision_pairs(model, data, epsilon_m)

    joint_rows: list[dict[str, object]] = []
    all_axis_pairs: set[str] = set()
    for joint_id in hinges:
        joint_name = name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        qpos_address = int(model.jnt_qposadr[joint_id])
        child_body = int(model.jnt_bodyid[joint_id])
        lower, upper = (float(value) for value in model.jnt_range[joint_id])
        rotations: list[np.ndarray] = []
        collision_samples = 0
        observed_pairs: set[str] = set()
        for angle in np.linspace(lower, upper, args.samples_per_joint):
            reset_neutral(model, data)
            data.qpos[qpos_address] = float(angle)
            mujoco.mj_forward(model, data)
            rotations.append(data.xmat[child_body].reshape(3, 3).copy())
            pairs = self_collision_pairs(model, data, epsilon_m)
            if pairs:
                collision_samples += 1
                observed_pairs.update(pairs)
                all_axis_pairs.update(pairs)
        observed_rotation = rotation_distance(rotations[0], rotations[-1])
        motion_pass = observed_rotation > min(0.05, 0.1 * (upper - lower))

        original_gravity = model.opt.gravity.copy()
        model.opt.gravity[:] = 0.0
        reset_neutral(model, data)
        data.qpos[0:3] = [0.0, 0.0, 0.5]
        actuator_ids = np.where(model.actuator_trnid[:, 0] == joint_id)[0]
        if len(actuator_ids) != 1:
            raise RuntimeError(
                f"joint {joint_name} has {len(actuator_ids)} actuators"
            )
        actuator_id = int(actuator_ids[0])
        positive_room = upper
        negative_room = -lower
        torque = 0.5 if positive_room >= negative_room else -0.5
        data.ctrl[actuator_id] = torque
        initial_angle = float(data.qpos[qpos_address])
        for _ in range(100):
            mujoco.mj_step(model, data)
        dynamic_delta = abs(float(data.qpos[qpos_address]) - initial_angle)
        dynamic_finite = bool(
            np.all(np.isfinite(data.qpos))
            and np.all(np.isfinite(data.qvel))
            and dynamic_delta > 1e-5
        )
        model.opt.gravity[:] = original_gravity
        joint_rows.append(
            {
                "joint": joint_name,
                "lower_rad": f"{lower:.9f}",
                "upper_rad": f"{upper:.9f}",
                "kinematic_rotation_deg": (
                    f"{math.degrees(observed_rotation):.6f}"
                ),
                "kinematic_motion_gate": "PASS" if motion_pass else "FAIL",
                "axis_sweep_samples": args.samples_per_joint,
                "self_collision_samples": collision_samples,
                "self_collision_pairs": " | ".join(sorted(observed_pairs)),
                "axis_collision_gate": (
                    "PASS" if collision_samples == 0 else "FAIL"
                ),
                "dynamic_test_torque_nm": torque,
                "dynamic_angle_delta_rad": f"{dynamic_delta:.9f}",
                "dynamic_response_gate": "PASS" if dynamic_finite else "FAIL",
                "status": (
                    "PASS"
                    if motion_pass
                    and collision_samples == 0
                    and dynamic_finite
                    else "FAIL"
                ),
            }
        )

    rng = np.random.default_rng(args.seed)
    random_collision_samples = 0
    random_pairs: set[str] = set()
    for _ in range(args.random_samples):
        reset_neutral(model, data)
        for joint_id in hinges:
            lower, upper = (
                float(value) for value in model.jnt_range[joint_id]
            )
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(
                rng.uniform(lower, upper)
            )
        mujoco.mj_forward(model, data)
        pairs = self_collision_pairs(model, data, epsilon_m)
        if pairs:
            random_collision_samples += 1
            random_pairs.update(pairs)

    corner_collision_samples = 0
    corner_pairs: set[str] = set()
    for endpoints in itertools.product((0, 1), repeat=len(hinges)):
        reset_neutral(model, data)
        for joint_id, endpoint in zip(hinges, endpoints):
            data.qpos[int(model.jnt_qposadr[joint_id])] = float(
                model.jnt_range[joint_id][endpoint]
            )
        mujoco.mj_forward(model, data)
        pairs = self_collision_pairs(model, data, epsilon_m)
        if pairs:
            corner_collision_samples += 1
            corner_pairs.update(pairs)

    if model.nkey < 1:
        raise RuntimeError("expected official standing keyframe")
    reset_neutral(model, data)
    data.qpos[:] = model.key_qpos[0]
    mujoco.mj_forward(model, data)
    standing_pairs = self_collision_pairs(model, data, epsilon_m)

    topology_pass = (
        model.nq == 23
        and model.nv == 22
        and model.njnt == 17
        and model.nbody == 18
        and model.nu == 16
        and model.nsensor == 3
    )
    mass = float(np.sum(model.body_mass))
    expected_mass = sum(
        float(element.get("value", "0"))
        for element in ET.parse(reference_urdf).getroot().findall(
            "./link/inertial/mass"
        )
    )
    mass_pass = abs(mass - expected_mass) < 1e-9
    joints_pass = all(row["status"] == "PASS" for row in joint_rows)
    random_pass = random_collision_samples == 0
    corners_pass = corner_collision_samples == 0
    neutral_pass = not neutral_pairs
    standing_pass = not standing_pairs
    overall = all(
        [
            topology_pass,
            mass_pass,
            joints_pass,
            random_pass,
            corners_pass,
            neutral_pass,
            standing_pass,
        ]
    )
    payload = {
        "mjcf": str(mjcf),
        "mujoco_version": mujoco.__version__,
        "topology": {
            "nq": model.nq,
            "nv": model.nv,
            "joint_count": model.njnt,
            "hinge_count": len(hinges),
            "body_count_including_world": model.nbody,
            "actuator_count": model.nu,
            "sensor_count": model.nsensor,
            "gate": "PASS" if topology_pass else "FAIL",
        },
        "total_mass_kg": mass,
        "expected_urdf_mass_kg": expected_mass,
        "mass_gate": "PASS" if mass_pass else "FAIL",
        "neutral_self_collision_pairs": sorted(neutral_pairs),
        "neutral_self_collision_gate": "PASS" if neutral_pass else "FAIL",
        "standing_self_collision_pairs": sorted(standing_pairs),
        "standing_self_collision_gate": "PASS" if standing_pass else "FAIL",
        "joint_motion_dynamic_and_axis_collision_gate": (
            "PASS" if joints_pass else "FAIL"
        ),
        "axis_observed_self_collision_pairs": sorted(all_axis_pairs),
        "random_sample_count": args.random_samples,
        "random_self_collision_samples": random_collision_samples,
        "random_self_collision_pairs": sorted(random_pairs),
        "random_gate": "PASS" if random_pass else "FAIL",
        "corner_sample_count": 2 ** len(hinges),
        "corner_self_collision_samples": corner_collision_samples,
        "corner_self_collision_pairs": sorted(corner_pairs),
        "corner_gate": "PASS" if corners_pass else "FAIL",
        "overall": "PASS" if overall else "FAIL",
        "evidence_scope": (
            "MuJoCo mesh kinematics/dynamics; not continuous collision proof "
            "or manufacturing tolerance/cable/fastener/flexible-cover signoff"
        ),
    }
    report_json.parent.mkdir(parents=True, exist_ok=True)
    report_json.write_text(
        json.dumps(payload, indent=2) + "\n", encoding="utf-8"
    )
    write_csv(report_csv, joint_rows)
    report_md.write_text(
        "\n".join(
            [
                "# Zeroth-01 RL-ready MuJoCo gate",
                "",
                f"- Model: `{mjcf}`",
                f"- MuJoCo: `{mujoco.__version__}`",
                f"- Topology: **{payload['topology']['gate']}**",
                f"- Mass/inertia import: **{payload['mass_gate']}**",
                (
                    "- Neutral / official standing self-collision: "
                    f"**{payload['neutral_self_collision_gate']} / "
                    f"{payload['standing_self_collision_gate']}**"
                ),
                (
                    "- 16 joint kinematic/dynamic/axis sweep: "
                    f"**{payload['joint_motion_dynamic_and_axis_collision_gate']}**"
                ),
                (
                    f"- Random poses: `{args.random_samples}`, collisions "
                    f"`{random_collision_samples}`, **{payload['random_gate']}**"
                ),
                (
                    f"- Box corners: `{2 ** len(hinges)}`, collisions "
                    f"`{corner_collision_samples}`, **{payload['corner_gate']}**"
                ),
                f"- Overall: **{payload['overall']}**",
                "",
                payload["evidence_scope"],
                "",
            ]
        ),
        encoding="utf-8",
    )
    print(json.dumps(payload, indent=2))
    if not overall:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
