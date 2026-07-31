from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


CANONICAL_URDF = (
    Path("generated")
    / "urdf"
    / "physical_mount_v1"
    / "zeroth01_physical_mount_v1.urdf"
)


def find_root(explicit: str | None) -> Path:
    if explicit:
        root = Path(explicit).resolve()
        if not (root / CANONICAL_URDF).is_file():
            raise FileNotFoundError(root / CANONICAL_URDF)
        return root
    here = Path(__file__).resolve()
    for candidate in (here.parent, *here.parents):
        if (candidate / CANONICAL_URDF).is_file():
            return candidate
    raise FileNotFoundError(CANONICAL_URDF)


def read_json(path: Path) -> dict[str, object]:
    return json.loads(path.read_text(encoding="utf-8"))


def add_check(
    checks: dict[str, dict[str, object]],
    failures: list[str],
    name: str,
    passed: bool,
    value: object,
    expected: object,
) -> None:
    checks[name] = {
        "value": value,
        "expected": expected,
        "gate": "PASS" if passed else "FAIL",
    }
    if not passed:
        failures.append(name)


def validate(root: Path) -> dict[str, object]:
    checks: dict[str, dict[str, object]] = {}
    failures: list[str] = []
    warnings: list[str] = []

    urdf_path = root / CANONICAL_URDF
    robot = ET.parse(urdf_path).getroot()
    links = robot.findall("link")
    revolute = [
        joint
        for joint in robot.findall("joint")
        if joint.get("type") in {"revolute", "continuous"}
    ]
    fixed = [
        joint for joint in robot.findall("joint") if joint.get("type") == "fixed"
    ]
    add_check(checks, failures, "urdf_link_count", len(links) == 21, len(links), 21)
    add_check(
        checks,
        failures,
        "urdf_actuated_joint_count",
        len(revolute) == 16,
        len(revolute),
        16,
    )
    add_check(
        checks,
        failures,
        "urdf_fixed_grippers",
        {"left_gripper", "right_gripper"}.issubset(
            {joint.get("name") for joint in fixed}
        ),
        sorted(
            joint.get("name")
            for joint in fixed
            if joint.get("name") in {"left_gripper", "right_gripper"}
        ),
        ["left_gripper", "right_gripper"],
    )

    mesh_paths: set[Path] = set()
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename:
            mesh_paths.add((urdf_path.parent / filename).resolve())
    missing_meshes = [
        str(path.relative_to(root)) if path.is_relative_to(root) else str(path)
        for path in sorted(mesh_paths)
        if not path.is_file()
    ]
    add_check(
        checks,
        failures,
        "urdf_unique_mesh_count",
        len(mesh_paths) == 36,
        len(mesh_paths),
        36,
    )
    add_check(
        checks,
        failures,
        "urdf_mesh_references",
        not missing_meshes,
        missing_meshes,
        [],
    )

    mass_total = 0.0
    bad_inertials: list[str] = []
    for link in links:
        inertial = link.find("inertial")
        name = str(link.get("name"))
        if inertial is None:
            bad_inertials.append(f"{name}:missing")
            continue
        mass_element = inertial.find("mass")
        inertia = inertial.find("inertia")
        if mass_element is None or inertia is None:
            bad_inertials.append(f"{name}:incomplete")
            continue
        mass = float(mass_element.get("value", "nan"))
        diagonal = [
            float(inertia.get(axis, "nan")) for axis in ("ixx", "iyy", "izz")
        ]
        if (
            not math.isfinite(mass)
            or mass <= 0.0
            or any(not math.isfinite(value) or value <= 0.0 for value in diagonal)
            or diagonal[0] + diagonal[1] < diagonal[2] - 1e-12
            or diagonal[0] + diagonal[2] < diagonal[1] - 1e-12
            or diagonal[1] + diagonal[2] < diagonal[0] - 1e-12
        ):
            bad_inertials.append(name)
        mass_total += mass
    add_check(
        checks,
        failures,
        "urdf_inertials",
        not bad_inertials,
        bad_inertials,
        [],
    )
    add_check(
        checks,
        failures,
        "urdf_total_mass_kg",
        abs(mass_total - 4.064411) <= 1e-9,
        round(mass_total, 9),
        4.064411,
    )

    actuator_path = (
        root / "generated" / "config" / "physical_mount_v1_actuators.json"
    )
    actuators = read_json(actuator_path)
    servo_rows = list(actuators.get("servos", []))
    servo_ids = {str(item["id"]) for item in servo_rows}
    actuator_joints = {str(item["joint"]) for item in servo_rows}
    urdf_joints = {str(joint.get("name")) for joint in revolute}
    add_check(
        checks,
        failures,
        "actuator_count_and_identity",
        len(servo_rows) == 16
        and len(servo_ids) == 16
        and actuator_joints == urdf_joints,
        {
            "rows": len(servo_rows),
            "unique_ids": len(servo_ids),
            "joint_set_matches": actuator_joints == urdf_joints,
        },
        {"rows": 16, "unique_ids": 16, "joint_set_matches": True},
    )
    unresolved_calibration = all(
        item.get("bus_id_gate") == "REQUIRES_PHYSICAL_BUS_SCAN"
        and item.get("neutral_count_gate") == "REQUIRES_JOG_CALIBRATION"
        and item.get("urdf_to_servo_direction_sign")
        == "REQUIRES_JOG_CALIBRATION"
        for item in servo_rows
    )
    add_check(
        checks,
        failures,
        "hardware_unknowns_not_fabricated",
        unresolved_calibration,
        unresolved_calibration,
        True,
    )

    reports = root / "reports" / "physical_mount_v1"
    source_gate = read_json(reports / "source_component_gate.json")
    motion_gate = read_json(reports / "dynamic_collision_gate.json")
    sw_gate = read_json(reports / "solidworks_physical_mount_gate.json")
    mount_gate = read_json(reports / "kinematic_mount_audit.json")
    gauge_gate = read_json(reports / "sts3250_interface_gauge.json")
    add_check(
        checks,
        failures,
        "source_component_gate",
        source_gate.get("overall") == "PASS",
        source_gate.get("overall"),
        "PASS",
    )
    add_check(
        checks,
        failures,
        "dynamic_collision_gate",
        motion_gate.get("overall") == "PASS"
        and motion_gate.get("neutral_nonadjacent_failures") == 0
        and motion_gate.get("coordinated_motion_failure_pose_count") == 0
        and all(
            row.get("gate") == "PASS"
            for row in motion_gate.get("joint_gates", [])
        ),
        {
            "overall": motion_gate.get("overall"),
            "neutral_nonadjacent_failures": motion_gate.get(
                "neutral_nonadjacent_failures"
            ),
            "joint_passes": sum(
                row.get("gate") == "PASS"
                for row in motion_gate.get("joint_gates", [])
            ),
            "coordinated_failure_poses": motion_gate.get(
                "coordinated_motion_failure_pose_count"
            ),
        },
        {
            "overall": "PASS",
            "neutral_nonadjacent_failures": 0,
            "joint_passes": 16,
            "coordinated_failure_poses": 0,
        },
    )
    add_check(
        checks,
        failures,
        "solidworks_gate",
        sw_gate.get("overall") == "PASS"
        and sw_gate.get("native_surface_part_count") == 36
        and sw_gate.get("fixed_link_subassembly_count") == 20
        and sw_gate.get("blue_servo_part_count") == 16,
        {
            "overall": sw_gate.get("overall"),
            "parts": sw_gate.get("native_surface_part_count"),
            "links": sw_gate.get("fixed_link_subassembly_count"),
            "servos": sw_gate.get("blue_servo_part_count"),
        },
        {"overall": "PASS", "parts": 36, "links": 20, "servos": 16},
    )
    add_check(
        checks,
        failures,
        "mount_axis_and_provenance",
        mount_gate.get("axis_alignment_gate") == "PASS"
        and mount_gate.get("overall_mount_provenance_gate") == "PASS",
        {
            "axis": mount_gate.get("axis_alignment_gate"),
            "provenance": mount_gate.get("overall_mount_provenance_gate"),
        },
        {"axis": "PASS", "provenance": "PASS"},
    )
    if mount_gate.get("strict_left_right_symmetry_gate") != "PASS":
        warnings.append(
            "Pinned source keeps ~1.45-1.48 mm hip-pitch/knee/ankle "
            "bilateral deviations; see kinematic_mount_audit.json."
        )
    print_gate = gauge_gate.get("gate", {}).get("full_robot_print_release")
    add_check(
        checks,
        failures,
        "manufacturing_hold_is_explicit",
        print_gate == "HOLD",
        print_gate,
        "HOLD",
    )

    sw_root = root / "generated" / "solidworks" / "physical_mount_v1"
    skeleton_parts = list((sw_root / "parts" / "skeleton").glob("*.SLDPRT"))
    servo_parts = list((sw_root / "parts" / "servos").glob("*.SLDPRT"))
    link_assemblies = list((sw_root / "links").glob("*.SLDASM"))
    bad_sw_files = [
        str(path.relative_to(root))
        for path in (*skeleton_parts, *servo_parts, *link_assemblies)
        if path.stat().st_size < 1024
    ]
    add_check(
        checks,
        failures,
        "solidworks_artifacts",
        len(skeleton_parts) == 20
        and len(servo_parts) == 16
        and len(link_assemblies) == 20
        and not bad_sw_files
        and (
            sw_root
            / "OPEN_FIRST_ZEROTH01_PHYSICAL_MOUNT_V1_16_BLUE_SERVOS.SLDASM"
        ).is_file()
        and (
            sw_root
            / "ZEROTH01_PHYSICAL_MOUNT_V1_16_BLUE_SERVOS_XRAY.SLDASM"
        ).is_file(),
        {
            "skeleton_parts": len(skeleton_parts),
            "servo_parts": len(servo_parts),
            "link_assemblies": len(link_assemblies),
            "invalid_files": bad_sw_files,
        },
        {
            "skeleton_parts": 20,
            "servo_parts": 16,
            "link_assemblies": 20,
            "invalid_files": [],
        },
    )
    misleading_names = [
        str(path.relative_to(root))
        for path in servo_parts
        if "STS3250_FORM_FIT" in path.name or "DIAGNOSTIC" in path.name
    ]
    add_check(
        checks,
        failures,
        "servo_part_naming_claim_boundary",
        not misleading_names,
        misleading_names,
        [],
    )

    required_files = [
        root
        / "generated"
        / "cad"
        / "physical_mount_v1"
        / "sts3250_interface"
        / "FEETECH_STS3250_C001_DIMENSION_REFERENCE.step",
        root
        / "generated"
        / "cad"
        / "physical_mount_v1"
        / "sts3250_interface"
        / "STS3250_4XM2_FIRST_ARTICLE_FACE_GAUGE.step",
        root
        / "generated"
        / "print"
        / "physical_mount_v1"
        / "first_article"
        / "STS3250_4XM2_FIRST_ARTICLE_FACE_GAUGE.stl",
        root
        / "generated"
        / "config"
        / "physical_mount_v1_hardware_calibration_template.csv",
        root
        / "generated"
        / "config"
        / "physical_mount_v1_rl_handoff.json",
        root / "one-seq.md",
    ]
    missing_required = [
        str(path.relative_to(root)) for path in required_files if not path.is_file()
    ]
    add_check(
        checks,
        failures,
        "handoff_artifacts",
        not missing_required,
        missing_required,
        [],
    )
    one_seq_lines = [
        line.strip()
        for line in (root / "one-seq.md").read_text(encoding="utf-8").splitlines()
        if line.strip()
    ]
    add_check(
        checks,
        failures,
        "one_seq_is_one_sentence_line",
        len(one_seq_lines) == 1
        and str(CANONICAL_URDF).replace("\\", "/") in one_seq_lines[0],
        len(one_seq_lines),
        1,
    )

    overall = (
        "PASS_WITH_MANUFACTURING_HOLD"
        if not failures
        else "FAIL"
    )
    return {
        "schema": "zeroth01.physical_mount_v1.release_gate.v1",
        "root": str(root),
        "canonical_urdf": str(CANONICAL_URDF).replace("\\", "/"),
        "checks": checks,
        "warnings": warnings,
        "failures": failures,
        "rl_training": "READY_WITH_DOMAIN_RANDOMIZATION" if not failures else "BLOCKED",
        "sim_to_real": "BLOCKED_PENDING_HARDWARE_CALIBRATION",
        "full_robot_print": "HOLD",
        "overall": overall,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root")
    args = parser.parse_args()
    root = find_root(args.root)
    result = validate(root)
    output = root / "reports" / "release_gate.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not result["failures"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
