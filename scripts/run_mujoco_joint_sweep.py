from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import mujoco
import numpy as np


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
DEFAULT_URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_reference.urdf"
REPORT_DIR = ROOT / "reports"
SUMMARY_CSV = REPORT_DIR / "mujoco_joint_sweep_summary.csv"
SAMPLES_CSV = REPORT_DIR / "mujoco_joint_sweep_samples.csv"
RANDOM_CSV = REPORT_DIR / "mujoco_random_pose_collisions.csv"
SUMMARY_JSON = REPORT_DIR / "mujoco_motion_summary.json"
SUMMARY_MD = REPORT_DIR / "mujoco_motion_report.md"


def id_name(model: mujoco.MjModel, objtype: mujoco.mjtObj, object_id: int) -> str:
    value = mujoco.mj_id2name(model, objtype, int(object_id))
    return value or f"{objtype.name}:{object_id}"


def body_name(model: mujoco.MjModel, body_id: int) -> str:
    # MuJoCo fuses the fixed URDF base and Torso into body 0. Naming it Torso
    # keeps contact reports in the same vocabulary as the source URDF.
    if body_id == 0:
        return "Torso"
    return id_name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)


def contact_rows(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    penetration_epsilon_m: float,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for index in range(data.ncon):
        contact = data.contact[index]
        if float(contact.dist) >= -penetration_epsilon_m:
            continue
        geom1 = int(contact.geom1)
        geom2 = int(contact.geom2)
        body1 = int(model.geom_bodyid[geom1])
        body2 = int(model.geom_bodyid[geom2])
        if body1 == body2:
            continue
        body_names = sorted(
            [
                body_name(model, body1),
                body_name(model, body2),
            ]
        )
        rows.append(
            {
                "pair": " :: ".join(body_names),
                "body1": body_names[0],
                "body2": body_names[1],
                "geom1": id_name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1),
                "geom2": id_name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2),
                "distance_m": float(contact.dist),
            }
        )
    return rows


def rotation_distance(a: np.ndarray, b: np.ndarray) -> float:
    relative = a.T @ b
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.acos(cosine)


def write_csv(path: Path, rows: list[dict[str, object]], fields: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Sweep every Zeroth-01 hinge and audit MuJoCo self-penetration."
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--samples-per-joint", type=int, default=61)
    parser.add_argument("--random-samples", type=int, default=2000)
    parser.add_argument("--seed", type=int, default=20260728)
    parser.add_argument("--contact-margin-mm", type=float, default=1.0)
    parser.add_argument("--penetration-epsilon-mm", type=float, default=0.01)
    parser.add_argument(
        "--report-prefix",
        default="mujoco",
        help="Prefix for report filenames; default preserves the reference report names.",
    )
    parser.add_argument(
        "--allow-neutral-baseline-as-assembly-overlap",
        action="store_true",
        help=(
            "Treat neutral penetration pairs as an explicit assembly-overlap "
            "allowlist. The report still records that raw neutral geometry is "
            "not totally clear."
        ),
    )
    parser.add_argument("--strict", action="store_true")
    args = parser.parse_args()
    if args.samples_per_joint < 3:
        raise ValueError("--samples-per-joint must be >= 3")
    report_prefix = safe_prefix = "".join(
        character
        for character in args.report_prefix
        if character.isalnum() or character in {"_", "-"}
    )
    if not safe_prefix:
        raise ValueError("--report-prefix must contain a safe filename character")
    if report_prefix == "mujoco":
        summary_csv = SUMMARY_CSV
        samples_csv = SAMPLES_CSV
        random_csv = RANDOM_CSV
        summary_json = SUMMARY_JSON
        summary_md = SUMMARY_MD
    else:
        summary_csv = REPORT_DIR / f"{report_prefix}_joint_sweep_summary.csv"
        samples_csv = REPORT_DIR / f"{report_prefix}_joint_sweep_samples.csv"
        random_csv = REPORT_DIR / f"{report_prefix}_random_pose_collisions.csv"
        summary_json = REPORT_DIR / f"{report_prefix}_motion_summary.json"
        summary_md = REPORT_DIR / f"{report_prefix}_motion_report.md"

    urdf = args.urdf.resolve()
    model = mujoco.MjModel.from_xml_path(str(urdf))
    data = mujoco.MjData(model)
    margin_m = args.contact_margin_mm / 1000.0
    epsilon_m = args.penetration_epsilon_mm / 1000.0
    model.geom_margin[:] = np.maximum(model.geom_margin, margin_m)

    hinges = [
        joint_id
        for joint_id in range(model.njnt)
        if int(model.jnt_type[joint_id]) == int(mujoco.mjtJoint.mjJNT_HINGE)
    ]
    if len(hinges) != 16:
        raise RuntimeError(f"expected 16 MuJoCo hinge joints, got {len(hinges)}")
    enabled_geoms = int(
        np.count_nonzero(
            np.logical_or(model.geom_contype != 0, model.geom_conaffinity != 0)
        )
    )
    if enabled_geoms == 0:
        raise RuntimeError("MuJoCo loaded no collision-enabled geometry")

    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    baseline_contacts = contact_rows(model, data, epsilon_m)
    baseline_pairs = {str(row["pair"]) for row in baseline_contacts}

    summary_rows: list[dict[str, object]] = []
    sample_rows: list[dict[str, object]] = []
    all_observed_pairs: set[str] = set()
    all_new_pairs: set[str] = set()
    safe_limits: dict[str, dict[str, object]] = {}

    for joint_id in hinges:
        joint_name = id_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        qpos_address = int(model.jnt_qposadr[joint_id])
        child_body = int(model.jnt_bodyid[joint_id])
        child_body_name = id_name(model, mujoco.mjtObj.mjOBJ_BODY, child_body)
        lower, upper = (float(value) for value in model.jnt_range[joint_id])
        values = np.linspace(lower, upper, args.samples_per_joint)
        orientations: list[np.ndarray] = []
        raw_collision_samples = 0
        new_collision_samples = 0
        observed_pairs: set[str] = set()
        observed_new_pairs: set[str] = set()
        minimum_distance = 0.0
        sample_is_new_collision_free: list[bool] = []

        for sample_index, angle in enumerate(values):
            data.qpos[:] = 0.0
            data.qpos[qpos_address] = angle
            mujoco.mj_forward(model, data)
            orientations.append(data.xmat[child_body].reshape(3, 3).copy())
            contacts = contact_rows(model, data, epsilon_m)
            pairs = sorted({str(row["pair"]) for row in contacts})
            new_pairs_at_sample = sorted(set(pairs) - baseline_pairs)
            if contacts:
                raw_collision_samples += 1
                observed_pairs.update(pairs)
                all_observed_pairs.update(pairs)
                minimum_distance = min(
                    minimum_distance,
                    min(float(row["distance_m"]) for row in contacts),
                )
            if new_pairs_at_sample:
                new_collision_samples += 1
                observed_new_pairs.update(new_pairs_at_sample)
                all_new_pairs.update(new_pairs_at_sample)
            sample_is_new_collision_free.append(not new_pairs_at_sample)
            sample_rows.append(
                {
                    "joint": joint_name,
                    "sample_index": sample_index,
                    "angle_rad": f"{angle:.9f}",
                    "angle_deg": f"{math.degrees(angle):.6f}",
                    "penetration_pair_count": len(pairs),
                    "penetration_pairs": " | ".join(pairs),
                    "new_pair_count_vs_neutral": len(new_pairs_at_sample),
                    "new_pairs_vs_neutral": " | ".join(new_pairs_at_sample),
                    "minimum_distance_m": (
                        f"{min(float(row['distance_m']) for row in contacts):.9f}"
                        if contacts
                        else ""
                    ),
                }
            )

        max_rotation = max(
            rotation_distance(orientations[0], orientation)
            for orientation in orientations[1:]
        )
        zero_index = int(np.argmin(np.abs(values)))
        lower_index = zero_index
        upper_index = zero_index
        if sample_is_new_collision_free[zero_index]:
            while lower_index > 0 and sample_is_new_collision_free[lower_index - 1]:
                lower_index -= 1
            while (
                upper_index + 1 < len(values)
                and sample_is_new_collision_free[upper_index + 1]
            ):
                upper_index += 1
        safe_lower = float(values[lower_index])
        safe_upper = float(values[upper_index])
        safe_limits[joint_name] = {
            "source_lower_rad": lower,
            "source_upper_rad": upper,
            "single_axis_sampled_safe_lower_rad": safe_lower,
            "single_axis_sampled_safe_upper_rad": safe_upper,
            "sample_step_rad": float(values[1] - values[0]),
            "contains_zero": bool(sample_is_new_collision_free[zero_index]),
            "full_source_range_has_no_new_pairs": not observed_new_pairs,
        }
        motion_ok = max_rotation > min(0.05, 0.1 * abs(upper - lower))
        no_new_pairs_ok = new_collision_samples == 0
        summary_rows.append(
            {
                "joint": joint_name,
                "child_body": child_body_name,
                "lower_rad": f"{lower:.9f}",
                "upper_rad": f"{upper:.9f}",
                "range_deg": f"{math.degrees(upper - lower):.6f}",
                "samples": len(values),
                "max_observed_child_rotation_deg": f"{math.degrees(max_rotation):.6f}",
                "motion_ok": str(motion_ok).lower(),
                "raw_penetration_sample_count": raw_collision_samples,
                "raw_penetration_pair_count": len(observed_pairs),
                "new_penetration_sample_count_vs_neutral": new_collision_samples,
                "new_pair_count_vs_neutral": len(observed_new_pairs),
                "minimum_distance_m": f"{minimum_distance:.9f}",
                "single_axis_sampled_safe_lower_rad": f"{safe_lower:.9f}",
                "single_axis_sampled_safe_upper_rad": f"{safe_upper:.9f}",
                "no_new_pairs_ok": str(no_new_pairs_ok).lower(),
                "status": "PASS" if motion_ok and no_new_pairs_ok else "FAIL",
            }
        )

    rng = np.random.default_rng(args.seed)
    random_rows: list[dict[str, object]] = []
    random_raw_collision_samples = 0
    random_new_collision_samples = 0
    random_pairs: set[str] = set()
    random_new_pairs: set[str] = set()
    for sample_index in range(args.random_samples):
        data.qpos[:] = 0.0
        angles: list[str] = []
        for joint_id in hinges:
            lower, upper = (float(value) for value in model.jnt_range[joint_id])
            angle = float(rng.uniform(lower, upper))
            data.qpos[int(model.jnt_qposadr[joint_id])] = angle
            angles.append(
                f"{id_name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)}={angle:.6f}"
            )
        mujoco.mj_forward(model, data)
        contacts = contact_rows(model, data, epsilon_m)
        if not contacts:
            continue
        random_raw_collision_samples += 1
        pairs = sorted({str(row["pair"]) for row in contacts})
        new_pairs_at_sample = sorted(set(pairs) - baseline_pairs)
        random_pairs.update(pairs)
        if new_pairs_at_sample:
            random_new_collision_samples += 1
            random_new_pairs.update(new_pairs_at_sample)
        random_rows.append(
            {
                "sample_index": sample_index,
                "penetration_pair_count": len(pairs),
                "penetration_pairs": " | ".join(pairs),
                "new_pair_count_vs_neutral": len(new_pairs_at_sample),
                "new_pairs_vs_neutral": " | ".join(new_pairs_at_sample),
                "minimum_distance_m": f"{min(float(row['distance_m']) for row in contacts):.9f}",
                "qpos": " | ".join(angles),
            }
        )

    summary_fields = [
        "joint",
        "child_body",
        "lower_rad",
        "upper_rad",
        "range_deg",
        "samples",
        "max_observed_child_rotation_deg",
        "motion_ok",
        "raw_penetration_sample_count",
        "raw_penetration_pair_count",
        "new_penetration_sample_count_vs_neutral",
        "new_pair_count_vs_neutral",
        "minimum_distance_m",
        "single_axis_sampled_safe_lower_rad",
        "single_axis_sampled_safe_upper_rad",
        "no_new_pairs_ok",
        "status",
    ]
    sample_fields = [
        "joint",
        "sample_index",
        "angle_rad",
        "angle_deg",
        "penetration_pair_count",
        "penetration_pairs",
        "new_pair_count_vs_neutral",
        "new_pairs_vs_neutral",
        "minimum_distance_m",
    ]
    random_fields = [
        "sample_index",
        "penetration_pair_count",
        "penetration_pairs",
        "new_pair_count_vs_neutral",
        "new_pairs_vs_neutral",
        "minimum_distance_m",
        "qpos",
    ]
    write_csv(summary_csv, summary_rows, summary_fields)
    write_csv(samples_csv, sample_rows, sample_fields)
    write_csv(random_csv, random_rows, random_fields)

    motion_pass = all(row["motion_ok"] == "true" for row in summary_rows)
    axis_no_new_pairs_pass = all(
        row["no_new_pairs_ok"] == "true" for row in summary_rows
    )
    neutral_total_clear = not baseline_contacts
    neutral_policy_pass = (
        neutral_total_clear
        or args.allow_neutral_baseline_as_assembly_overlap
    )
    random_no_new_pairs_pass = random_new_collision_samples == 0
    payload = {
        "urdf": str(urdf),
        "mujoco_version": mujoco.__version__,
        "joint_count": len(hinges),
        "collision_enabled_geom_count": enabled_geoms,
        "samples_per_joint": args.samples_per_joint,
        "axis_sample_count": len(sample_rows),
        "random_seed": args.seed,
        "random_sample_count": args.random_samples,
        "contact_margin_mm": args.contact_margin_mm,
        "penetration_epsilon_mm": args.penetration_epsilon_mm,
        "neutral_penetration_pairs": sorted(baseline_pairs),
        "observed_axis_penetration_pairs": sorted(all_observed_pairs),
        "observed_axis_new_pairs_vs_neutral": sorted(all_new_pairs),
        "random_raw_penetration_sample_count": random_raw_collision_samples,
        "random_new_penetration_sample_count_vs_neutral": random_new_collision_samples,
        "random_penetration_pairs": sorted(random_pairs),
        "random_new_pairs_vs_neutral": sorted(random_new_pairs),
        "single_axis_sampled_safe_limits": safe_limits,
        "motion_gate": "PASS" if motion_pass else "FAIL",
        "neutral_total_clear_gate": "PASS" if neutral_total_clear else "FAIL",
        "neutral_collision_policy_gate": (
            "PASS" if neutral_policy_pass else "FAIL"
        ),
        "neutral_baseline_treated_as_assembly_overlap": (
            args.allow_neutral_baseline_as_assembly_overlap
        ),
        "axis_no_new_pairs_gate": "PASS" if axis_no_new_pairs_pass else "FAIL",
        "random_no_new_pairs_gate": "PASS" if random_no_new_pairs_pass else "FAIL",
        "overall": (
            "PASS"
            if (
                motion_pass
                and neutral_policy_pass
                and axis_no_new_pairs_pass
                and random_no_new_pairs_pass
            )
            else "FAIL"
        ),
    }
    summary_json.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")

    failed_joints = [str(row["joint"]) for row in summary_rows if row["status"] != "PASS"]
    markdown = f"""# Zeroth-01 MuJoCo motion / collision report

- URDF: `{urdf}`
- MuJoCo: `{mujoco.__version__}`
- Moving joints: `{len(hinges)}`
- Per-joint samples: `{args.samples_per_joint}` (`{len(sample_rows)}` total)
- Random full-range poses: `{args.random_samples}` (seed `{args.seed}`)
- Motion gate: **{payload['motion_gate']}**
- Neutral total-clear gate: **{payload['neutral_total_clear_gate']}**
- Neutral collision-policy gate: **{payload['neutral_collision_policy_gate']}**
- Per-axis no-new-pairs gate: **{payload['axis_no_new_pairs_gate']}**
- Random-pose no-new-pairs gate: **{payload['random_no_new_pairs_gate']}**
- Overall: **{payload['overall']}**

Failed joints: `{', '.join(failed_joints) if failed_joints else 'none'}`

Neutral penetration pairs: `{'; '.join(sorted(baseline_pairs)) if baseline_pairs else 'none'}`

Observed axis-sweep penetration pairs:

{chr(10).join(f'- `{pair}`' for pair in sorted(all_observed_pairs)) if all_observed_pairs else '- none'}

New axis-sweep pairs versus neutral:

{chr(10).join(f'- `{pair}`' for pair in sorted(all_new_pairs)) if all_new_pairs else '- none'}

This is a deterministic mesh-level kinematic audit. It is not a manufacturing
tolerance, cable, fastener or flexible-cover clearance sign-off.
"""
    summary_md.write_text(markdown, encoding="utf-8")
    print(json.dumps(payload, indent=2))
    if args.strict and payload["overall"] != "PASS":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
