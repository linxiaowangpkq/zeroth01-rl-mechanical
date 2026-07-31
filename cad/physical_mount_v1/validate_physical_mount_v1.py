from __future__ import annotations

import argparse
import csv
import json
import random
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = (
    ROOT
    / "generated"
    / "urdf"
    / "physical_mount_v1"
    / "zeroth01_physical_mount_v1.urdf"
)
REPORT_ROOT = ROOT / "reports" / "physical_mount_v1"
REPORT_PATH = REPORT_ROOT / "dynamic_collision_gate.json"
CONTACT_PATH = REPORT_ROOT / "dynamic_collision_contacts.csv"
EPSILON_M = 1.0e-5


def _name(model: mujoco.MjModel, object_type: int, object_id: int) -> str:
    return (
        mujoco.mj_id2name(model, object_type, int(object_id))
        or f"{object_type}:{object_id}"
    )


def _joint_rows(model: mujoco.MjModel) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for joint_id in range(model.njnt):
        joint_type = int(model.jnt_type[joint_id])
        if joint_type != int(mujoco.mjtJoint.mjJNT_HINGE):
            continue
        rows.append(
            {
                "joint_id": joint_id,
                "name": _name(
                    model,
                    mujoco.mjtObj.mjOBJ_JOINT,
                    joint_id,
                ),
                "qpos_address": int(model.jnt_qposadr[joint_id]),
                "lower": float(model.jnt_range[joint_id][0]),
                "upper": float(model.jnt_range[joint_id][1]),
            }
        )
    return rows


def _is_adjacent(model: mujoco.MjModel, first: int, second: int) -> bool:
    if first == second:
        return True
    return (
        int(model.body_parentid[first]) == second
        or int(model.body_parentid[second]) == first
    )


def _contacts(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    *,
    pose: str,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    allowed: list[dict[str, object]] = []
    failures: list[dict[str, object]] = []
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        penetration = max(0.0, -float(contact.dist))
        if penetration <= EPSILON_M:
            continue
        geom_a = int(contact.geom1)
        geom_b = int(contact.geom2)
        body_a = int(model.geom_bodyid[geom_a])
        body_b = int(model.geom_bodyid[geom_b])
        adjacent = _is_adjacent(model, body_a, body_b)
        row = {
            "pose": pose,
            "body_a": _name(model, mujoco.mjtObj.mjOBJ_BODY, body_a),
            "geom_a": _name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_a),
            "body_b": _name(model, mujoco.mjtObj.mjOBJ_BODY, body_b),
            "geom_b": _name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_b),
            "penetration_mm": penetration * 1000.0,
            "classification": (
                "ALLOWED_DIRECT_JOINT_INTERFACE"
                if adjacent
                else "FAIL_NONADJACENT_LINK_INTERFERENCE"
            ),
        }
        (allowed if adjacent else failures).append(row)
    return allowed, failures


def _sample_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    positions: dict[int, float],
    pose: str,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    for qpos_address, value in positions.items():
        data.qpos[qpos_address] = value
    mujoco.mj_forward(model, data)
    return _contacts(model, data, pose=pose)


def validate(
    single_samples: int,
    coordinated_samples: int,
    random_samples: int,
    seed: int,
) -> dict[str, object]:
    model = mujoco.MjModel.from_xml_path(str(URDF_PATH))
    data = mujoco.MjData(model)
    joints = _joint_rows(model)
    if len(joints) != 16:
        raise ValueError(f"expected 16 hinge joints, got {len(joints)}")

    all_allowed: list[dict[str, object]] = []
    all_failures: list[dict[str, object]] = []
    diagnostic_random_allowed: list[dict[str, object]] = []
    diagnostic_random_failures: list[dict[str, object]] = []
    joint_gates: list[dict[str, object]] = []

    allowed, failures = _sample_pose(model, data, {}, "neutral")
    all_allowed.extend(allowed)
    all_failures.extend(failures)

    for joint in joints:
        before = len(all_failures)
        values = np.linspace(
            float(joint["lower"]),
            float(joint["upper"]),
            single_samples,
        )
        for sample_index, value in enumerate(values):
            allowed, failures = _sample_pose(
                model,
                data,
                {int(joint["qpos_address"]): float(value)},
                f"single:{joint['name']}:{sample_index:02d}",
            )
            all_allowed.extend(allowed)
            all_failures.extend(failures)
        joint_failures = len(all_failures) - before
        joint_gates.append(
            {
                "joint": joint["name"],
                "lower_rad": joint["lower"],
                "upper_rad": joint["upper"],
                "samples": single_samples,
                "nonadjacent_contact_count": joint_failures,
                "gate": "PASS" if joint_failures == 0 else "FAIL",
            }
        )

    coordinated_pose_failures = 0
    coordinated_before = len(all_failures)
    for sample_index in range(coordinated_samples):
        phase = (
            2.0
            * np.pi
            * sample_index
            / max(1, coordinated_samples - 1)
        )
        positions: dict[int, float] = {}
        for joint_index, joint in enumerate(joints):
            lower = float(joint["lower"])
            upper = float(joint["upper"])
            negative_room = max(0.0, -lower)
            positive_room = max(0.0, upper)
            amplitude = min(
                np.deg2rad(8.0),
                0.6 * negative_room
                if negative_room
                else np.deg2rad(3.0),
                0.6 * positive_room
                if positive_room
                else np.deg2rad(3.0),
            )
            positions[int(joint["qpos_address"])] = float(
                amplitude
                * np.sin(phase + (joint_index % 4) * np.pi / 2.0)
            )
        allowed, failures = _sample_pose(
            model,
            data,
            positions,
            f"coordinated:{sample_index:04d}",
        )
        all_allowed.extend(allowed)
        all_failures.extend(failures)
        if failures:
            coordinated_pose_failures += 1
    coordinated_failure_contacts = len(all_failures) - coordinated_before

    generator = random.Random(seed)
    random_pose_failures = 0
    for sample_index in range(random_samples):
        positions = {
            int(joint["qpos_address"]): generator.uniform(
                float(joint["lower"]),
                float(joint["upper"]),
            )
            for joint in joints
        }
        allowed, failures = _sample_pose(
            model,
            data,
            positions,
            f"random:{sample_index:04d}",
        )
        diagnostic_random_allowed.extend(allowed)
        diagnostic_random_failures.extend(failures)
        if failures:
            random_pose_failures += 1

    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    contact_rows = (
        all_allowed
        + all_failures
        + diagnostic_random_allowed
        + diagnostic_random_failures
    )
    fields = [
        "pose",
        "body_a",
        "geom_a",
        "body_b",
        "geom_b",
        "penetration_mm",
        "classification",
    ]
    with CONTACT_PATH.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(contact_rows)

    maximum_failure_penetration_mm = max(
        (
            float(row["penetration_mm"])
            for row in all_failures
        ),
        default=0.0,
    )
    report = {
        "schema": "zeroth01.physical_mount_v1.dynamic_collision_gate.v1",
        "urdf": str(URDF_PATH.relative_to(ROOT)).replace("\\", "/"),
        "mujoco_version": mujoco.__version__,
        "body_count": int(model.nbody),
        "hinge_joint_count": len(joints),
        "collision_geom_count": int(model.ngeom),
        "penetration_epsilon_mm": EPSILON_M * 1000.0,
        "adjacent_interface_policy": (
            "Contacts between a body and its direct kinematic parent are "
            "allowed because the source meshes include intentionally "
            "coincident horn, shaft and carrier interface geometry. Contacts "
            "between nonadjacent links fail."
        ),
        "neutral_allowed_direct_interface_contacts": sum(
            1 for row in all_allowed if row["pose"] == "neutral"
        ),
        "neutral_nonadjacent_failures": sum(
            1 for row in all_failures if row["pose"] == "neutral"
        ),
        "single_joint_samples_per_joint": single_samples,
        "joint_gates": joint_gates,
        "coordinated_motion_sample_count": coordinated_samples,
        "coordinated_motion_failure_pose_count": coordinated_pose_failures,
        "coordinated_motion_failure_contact_count": coordinated_failure_contacts,
        "coordinated_motion_gate": (
            "PASS" if coordinated_failure_contacts == 0 else "FAIL"
        ),
        "random_pose_count": random_samples,
        "random_pose_failure_count": random_pose_failures,
        "random_pose_semantics": (
            "DIAGNOSTIC_ONLY: independent uniform sampling of the full 16-D "
            "joint-limit Cartesian product intentionally includes physically "
            "self-colliding humanoid poses and is not a pass/fail gate."
        ),
        "allowed_direct_interface_contact_observations": len(all_allowed),
        "nonadjacent_failure_contact_observations": len(all_failures),
        "diagnostic_random_nonadjacent_contact_observations": len(
            diagnostic_random_failures
        ),
        "maximum_nonadjacent_penetration_mm": maximum_failure_penetration_mm,
        "overall": (
            "PASS"
            if not all_failures
            and all(item["gate"] == "PASS" for item in joint_gates)
            and coordinated_failure_contacts == 0
            else "FAIL"
        ),
        "claim_boundary": (
            "PASS demonstrates no MuJoCo-detected collision between "
            "nonadjacent rigid links at neutral, across the sampled full "
            "guarded range of every individual joint, and across the "
            "coordinated multi-joint motion used for the review GIF. The "
            "independent 16-D random-box sample is diagnostic because a "
            "humanoid's Cartesian product of joint limits necessarily "
            "contains self-colliding poses. PASS does not certify continuous "
            "configuration space, cable bend radius, fastener strength or "
            "manufacturing tolerance."
        ),
    }
    REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return report


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Validate the physical-mount URDF over guarded joint ranges while "
            "distinguishing direct joint-interface contact from true "
            "nonadjacent-link interference."
        )
    )
    parser.add_argument("--single-samples", type=int, default=61)
    parser.add_argument("--coordinated-samples", type=int, default=73)
    parser.add_argument("--random-samples", type=int, default=250)
    parser.add_argument("--seed", type=int, default=3250)
    args = parser.parse_args()
    if args.single_samples < 3:
        raise ValueError("--single-samples must be at least 3")
    if args.coordinated_samples < 3:
        raise ValueError("--coordinated-samples must be at least 3")
    if args.random_samples < 0:
        raise ValueError("--random-samples must be nonnegative")
    report = validate(
        args.single_samples,
        args.coordinated_samples,
        args.random_samples,
        args.seed,
    )
    print(REPORT_PATH)
    print(CONTACT_PATH)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
