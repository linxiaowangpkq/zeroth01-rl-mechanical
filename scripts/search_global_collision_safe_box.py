from __future__ import annotations

import argparse
import csv
import itertools
import json
from pathlib import Path

import mujoco
import numpy as np


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
DEFAULT_URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_audited.urdf"
REPORT_CSV = ROOT / "reports" / "global_collision_box_search.csv"
REPORT_JSON = ROOT / "reports" / "global_collision_box_search.json"

# Official stompymicro standing pose from upstream
# sim/resources/stompymicro/joints.py. Arm joints are neutral because that
# source disables its arm controller.
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


def object_name(
    model: mujoco.MjModel,
    object_type: mujoco.mjtObj,
    object_id: int,
) -> str:
    value = mujoco.mj_id2name(model, object_type, int(object_id))
    return value or f"{object_type.name}:{object_id}"


def body_name(model: mujoco.MjModel, body_id: int) -> str:
    # The URDF importer welds base/Torso to world. Preserve the URDF vocabulary
    # in the report so it matches the native-MJCF collision policy.
    if body_id == 0:
        return "Torso"
    return object_name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)


def penetrating_pairs(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    epsilon_m: float,
) -> set[str]:
    pairs: set[str] = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        if float(contact.dist) >= -epsilon_m:
            continue
        body_ids = (
            int(model.geom_bodyid[int(contact.geom1)]),
            int(model.geom_bodyid[int(contact.geom2)]),
        )
        if body_ids[0] == body_ids[1]:
            continue
        names = sorted(body_name(model, body_id) for body_id in body_ids)
        pairs.add(" :: ".join(names))
    return pairs


def scaled_box(
    name: str,
    source_lower: float,
    source_upper: float,
    scale: float,
) -> tuple[float, float]:
    nominal = float(OFFICIAL_STANDING_POSE.get(name, 0.0))
    lower = nominal + scale * (source_lower - nominal)
    upper = nominal + scale * (source_upper - nominal)
    # Both the official standing pose and mathematical zero remain admitted.
    return min(lower, nominal, 0.0), max(upper, nominal, 0.0)


def evaluate_random(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joints: list[dict[str, object]],
    bounds: dict[str, tuple[float, float]],
    allowed_pairs: set[str],
    sample_count: int,
    seed: int,
    epsilon_m: float,
) -> tuple[int, set[str], dict[str, float] | None]:
    rng = np.random.default_rng(seed)
    collision_count = 0
    pairs: set[str] = set()
    first_qpos: dict[str, float] | None = None
    for _ in range(sample_count):
        data.qpos[:] = 0.0
        qpos: dict[str, float] = {}
        for joint in joints:
            name = str(joint["name"])
            lower, upper = bounds[name]
            angle = float(rng.uniform(lower, upper))
            data.qpos[int(joint["qpos_address"])] = angle
            qpos[name] = angle
        mujoco.mj_forward(model, data)
        new_pairs = penetrating_pairs(model, data, epsilon_m) - allowed_pairs
        if new_pairs:
            collision_count += 1
            pairs.update(new_pairs)
            if first_qpos is None:
                first_qpos = qpos
    return collision_count, pairs, first_qpos


def evaluate_corners(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    joints: list[dict[str, object]],
    bounds: dict[str, tuple[float, float]],
    allowed_pairs: set[str],
    epsilon_m: float,
) -> tuple[int, set[str], dict[str, float] | None]:
    collision_count = 0
    pairs: set[str] = set()
    first_qpos: dict[str, float] | None = None
    for corner in itertools.product((0, 1), repeat=len(joints)):
        data.qpos[:] = 0.0
        qpos: dict[str, float] = {}
        for joint, endpoint in zip(joints, corner):
            name = str(joint["name"])
            angle = bounds[name][endpoint]
            data.qpos[int(joint["qpos_address"])] = angle
            qpos[name] = angle
        mujoco.mj_forward(model, data)
        new_pairs = penetrating_pairs(model, data, epsilon_m) - allowed_pairs
        if new_pairs:
            collision_count += 1
            pairs.update(new_pairs)
            if first_qpos is None:
                first_qpos = qpos
    return collision_count, pairs, first_qpos


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Search a conservative rectangular Zeroth-01 joint box that "
            "contains neutral and the official standing pose."
        )
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--random-samples", type=int, default=20000)
    parser.add_argument("--seed", type=int, default=20260729)
    parser.add_argument("--contact-margin-mm", type=float, default=1.0)
    parser.add_argument("--penetration-epsilon-mm", type=float, default=0.01)
    parser.add_argument(
        "--scales",
        default="1.0,0.9,0.8,0.7,0.6,0.5,0.4,0.3,0.2,0.15,0.1",
    )
    args = parser.parse_args()

    urdf = args.urdf.resolve()
    model = mujoco.MjModel.from_xml_path(str(urdf))
    data = mujoco.MjData(model)
    model.geom_margin[:] = np.maximum(
        model.geom_margin, args.contact_margin_mm / 1000.0
    )
    epsilon_m = args.penetration_epsilon_mm / 1000.0
    joint_ids = [
        joint_id
        for joint_id in range(model.njnt)
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_HINGE)
    ]
    if len(joint_ids) != 16:
        raise RuntimeError(f"expected 16 hinge joints, got {len(joint_ids)}")
    joints = [
        {
            "name": object_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id),
            "qpos_address": int(model.jnt_qposadr[joint_id]),
            "source_lower": float(model.jnt_range[joint_id][0]),
            "source_upper": float(model.jnt_range[joint_id][1]),
        }
        for joint_id in joint_ids
    ]

    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    allowed_pairs = penetrating_pairs(model, data, epsilon_m)
    scales = [float(value.strip()) for value in args.scales.split(",")]
    rows: list[dict[str, object]] = []
    selected: dict[str, object] | None = None
    for candidate_index, scale in enumerate(scales):
        bounds = {
            str(joint["name"]): scaled_box(
                str(joint["name"]),
                float(joint["source_lower"]),
                float(joint["source_upper"]),
                scale,
            )
            for joint in joints
        }
        random_count, random_pairs, random_qpos = evaluate_random(
            model,
            data,
            joints,
            bounds,
            allowed_pairs,
            args.random_samples,
            args.seed + candidate_index,
            epsilon_m,
        )
        corner_count: int | None = None
        corner_pairs: set[str] = set()
        corner_qpos: dict[str, float] | None = None
        if random_count == 0:
            corner_count, corner_pairs, corner_qpos = evaluate_corners(
                model,
                data,
                joints,
                bounds,
                allowed_pairs,
                epsilon_m,
            )
        passed = random_count == 0 and corner_count == 0
        rows.append(
            {
                "scale": scale,
                "random_samples": args.random_samples,
                "random_collision_samples": random_count,
                "random_new_pairs": " | ".join(sorted(random_pairs)),
                "corner_samples": 2 ** len(joints) if corner_count is not None else 0,
                "corner_collision_samples": (
                    corner_count if corner_count is not None else ""
                ),
                "corner_new_pairs": " | ".join(sorted(corner_pairs)),
                "status": "PASS" if passed else "FAIL",
            }
        )
        if passed:
            selected = {
                "scale": scale,
                "bounds": {
                    name: {"lower_rad": lower, "upper_rad": upper}
                    for name, (lower, upper) in bounds.items()
                },
                "random_samples": args.random_samples,
                "random_seed": args.seed + candidate_index,
                "corner_samples": 2 ** len(joints),
            }
            break
        if selected is None and candidate_index == len(scales) - 1:
            selected = {
                "scale": None,
                "bounds": {},
                "first_random_collision_qpos": random_qpos,
                "first_corner_collision_qpos": corner_qpos,
            }

    REPORT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    payload = {
        "urdf": str(urdf),
        "method": (
            "uniform scale about official standing pose; neutral and official "
            "standing pose are forcibly retained; neutral mesh-overlap pairs "
            "are treated as intended assembly exclusions"
        ),
        "allowed_neutral_pairs": sorted(allowed_pairs),
        "contact_margin_mm": args.contact_margin_mm,
        "penetration_epsilon_mm": args.penetration_epsilon_mm,
        "selected": selected,
        "statistical_not_continuous_proof": True,
        "rows": rows,
    }
    REPORT_JSON.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    if selected is None or selected.get("scale") is None:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
