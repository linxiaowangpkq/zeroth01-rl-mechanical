from __future__ import annotations

import argparse
import csv
import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
REFERENCE_GENERATOR = THIS_FILE.with_name("gen_zeroth01_urdf.py")
MOTION_SUMMARY = ROOT / "reports" / "mujoco_motion_summary.json"
DEFAULT_OUTPUT = ROOT / "generated" / "urdf" / "zeroth01_rl_audited.urdf"
LIMIT_REPORT = ROOT / "reports" / "rl_audited_joint_limits.csv"
ROBOT_NAME = "zeroth01_rl_audited_single_axis_16dof"

# The official stompymicro Python control interface contributes an additional
# software-limit constraint. Final limits are the intersection of the frozen
# geometry-compatible URDF, these control limits, and the motion audit.
OFFICIAL_CONTROL_LIMITS = {
    "left_hip_pitch": (-1.5707963, 1.5707963),
    "left_hip_yaw": (-1.5707963, 0.087266463),
    "left_hip_roll": (-0.78539816, 0.78539816),
    "left_knee_pitch": (-1.0471976, 0.0),
    "left_ankle_pitch": (-1.5707963, 1.5707963),
    "right_hip_pitch": (-1.5707963, 1.5707963),
    "right_hip_yaw": (-0.087266463, 1.5707963),
    "right_hip_roll": (-0.78539816, 0.78539816),
    "right_knee_pitch": (0.0, 1.0471976),
    "right_ankle_pitch": (-1.5707963, 1.5707963),
}


def load_reference_module():
    spec = importlib.util.spec_from_file_location(
        "zeroth01_reference_generator", REFERENCE_GENERATOR
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {REFERENCE_GENERATOR}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def gen_urdf() -> ET.Element:
    if not MOTION_SUMMARY.is_file():
        raise FileNotFoundError(
            f"run run_mujoco_joint_sweep.py first: {MOTION_SUMMARY}"
        )
    reference = load_reference_module()
    root = reference.gen_urdf()
    root.set("name", ROBOT_NAME)
    summary = json.loads(MOTION_SUMMARY.read_text(encoding="utf-8"))
    safe_limits = summary["single_axis_sampled_safe_limits"]

    for joint in root.findall("joint"):
        name = joint.get("name", "")
        limit = joint.find("limit")
        if limit is None or name not in safe_limits:
            continue
        source_lower = float(limit.get("lower"))
        source_upper = float(limit.get("upper"))
        audit = safe_limits[name]
        audited_lower = float(audit["single_axis_sampled_safe_lower_rad"])
        audited_upper = float(audit["single_axis_sampled_safe_upper_rad"])
        step = abs(float(audit["sample_step_rad"]))

        # When a sampled collision boundary exists, move one additional sample
        # inward instead of placing the training limit directly on the last
        # collision-free sample.
        if audited_lower > source_lower + 1e-9:
            audited_lower += step
        if audited_upper < source_upper - 1e-9:
            audited_upper -= step

        if name in OFFICIAL_CONTROL_LIMITS:
            control_lower, control_upper = OFFICIAL_CONTROL_LIMITS[name]
            audited_lower = max(audited_lower, control_lower)
            audited_upper = min(audited_upper, control_upper)
        if not audited_lower < audited_upper:
            raise ValueError(
                f"audited limit collapsed for {name}: "
                f"{audited_lower}, {audited_upper}"
            )
        limit.set("lower", f"{audited_lower:.9f}")
        limit.set("upper", f"{audited_upper:.9f}")

    root.insert(
        1,
        ET.Comment(
            "RL audit overlay: lower-body bounds are intersected with the "
            "official stompymicro control limits; bounds with a one-axis new "
            "collision pair are moved one 61-sample step inside the observed "
            "collision-free interval. Multi-joint self-collision remains enabled "
            "and must be handled by termination/penalty logic."
        ),
    )
    reference._validate_tree(root)
    return root


def write_limit_report(root: ET.Element) -> None:
    reference = load_reference_module().gen_urdf()
    source = {
        joint.get("name", ""): joint.find("limit")
        for joint in reference.findall("joint")
        if joint.find("limit") is not None
    }
    rows = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is None:
            continue
        name = joint.get("name", "")
        original = source[name]
        rows.append(
            {
                "joint": name,
                "source_lower_rad": original.get("lower"),
                "source_upper_rad": original.get("upper"),
                "audited_lower_rad": limit.get("lower"),
                "audited_upper_rad": limit.get("upper"),
                "official_control_limit_applied": str(
                    name in OFFICIAL_CONTROL_LIMITS
                ).lower(),
                "method": (
                    "intersection(source_urdf, official_control, "
                    "buffered_single_axis_sweep)"
                ),
            }
        )
    LIMIT_REPORT.parent.mkdir(parents=True, exist_ok=True)
    with LIMIT_REPORT.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate the audited Zeroth-01 RL URDF limit overlay."
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    root = gen_urdf()
    ET.indent(root, space="  ")
    output = args.output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    write_limit_report(root)
    print(f"URDF={output}")
    print(f"LIMIT_REPORT={LIMIT_REPORT}")


if __name__ == "__main__":
    main()
