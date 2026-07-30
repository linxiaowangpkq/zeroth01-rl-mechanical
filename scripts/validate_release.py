from __future__ import annotations

import hashlib
import json
import re
import sys
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "release_gate.json"
EXPECTED_TOTAL_MASS_KG = 4.997342616724
MANIFEST_EXCLUDED = {
    "RELEASE_MANIFEST.json",
    "reports/release_gate.json",
}

checks: list[dict[str, object]] = []


def record(name: str, passed: bool, actual: object, expected: object) -> None:
    checks.append(
        {
            "check": name,
            "status": "PASS" if passed else "FAIL",
            "actual": actual,
            "expected": expected,
        }
    )


def load_json(relative: str) -> dict[str, object]:
    return json.loads((ROOT / relative).read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_required_files() -> None:
    required = [
        "generated/cad/round_v1/ZEROTH01_ROUND_V3_WHITE_EVA_16_BLUE_SERVOS_ASSEMBLY.step",
        "generated/solidworks/portable_flat_round_v3/ZEROTH01_ROUND_V3_WHITE_EXTERIOR.SLDASM",
        "generated/solidworks/portable_flat_round_v3/OPEN_FIRST_ZEROTH01_ROUND_V3_WHITE_EVA_16_BLUE_SERVOS_XRAY.SLDASM",
        "generated/solidworks/portable_flat_round_v3/ZEROTH01_STS3250_C001_BLUE_DIAGNOSTIC.SLDPRT",
        "generated/urdf/zeroth01_rl_round_v1.urdf",
        "generated/mujoco/zeroth01_rl_round_v1.xml",
        "snapshots/solidworks/round_v1/zeroth01_round_v3_white_front.png",
        "snapshots/solidworks/round_v1/zeroth01_round_v3_16_blue_servos_annotated_front.png",
        "snapshots/solidworks/round_v1/zeroth01_round_v3_electronics_annotated_front.png",
        "snapshots/solidworks/round_v1/zeroth01_round_v3_16_blue_servos_motion.gif",
        "reports/round_v3_arm_fit_gate.json",
        "reports/mujoco_round_v3_gate.json",
        "one-seq.md",
        "LICENSES/POPPY_EVA_HEAD_CC_BY_SA_4_0.md",
        "source_assets/open_source_head/Poppy-eva-head-design/SOURCE_COMMIT.txt",
    ]
    missing = [name for name in required if not (ROOT / name).is_file()]
    record("required_release_files", not missing, missing, [])


def validate_solidworks() -> None:
    gate = load_json("reports/solidworks_round_v1_gate.json")
    expected = {
        "component_count": 57,
        "source_link_component_count": 17,
        "round_overlay_component_count": 24,
        "diagnostic_blue_sts3250_c001_component_count": 16,
        "diagnostic_blue_servo_part_reuse_count": 1,
        "nonphysical_colored_joint_marker_count": 0,
        "explicit_replacement_sts3250_component_count": 0,
        "new_servo_cage_component_count": 0,
        "new_child_output_hub_component_count": 0,
    }
    mismatches = {
        key: gate.get(key)
        for key, value in expected.items()
        if gate.get(key) != value
    }
    record("solidworks_component_contract", not mismatches, mismatches, expected)
    record(
        "solidworks_review_gate",
        str(gate.get("overall_review_gate", "")).startswith("PASS_"),
        gate.get("overall_review_gate"),
        "PASS_*",
    )

    portable = ROOT / "generated" / "solidworks" / "portable_flat_round_v3"
    part_count = len(list(portable.glob("*.SLDPRT")))
    assembly_count = len(list(portable.glob("*.SLDASM")))
    record("portable_solidworks_unique_parts", part_count == 42, part_count, 42)
    record("portable_solidworks_assemblies", assembly_count == 2, assembly_count, 2)

    motion = load_json("reports/solidworks_round_v3_motion_only_gate.json")
    record("solidworks_motion_gate", motion.get("status") == "PASS", motion.get("status"), "PASS")
    record(
        "solidworks_motion_component_count",
        motion.get("component_count") == 57,
        motion.get("component_count"),
        57,
    )


def validate_print_and_fit() -> None:
    print_gate = load_json("reports/round_v1_print_mesh_gate.json")
    final_dir = ROOT / "generated" / "print" / "round_v1" / "final"
    stl_count = len(list(final_dir.glob("*.stl")))
    record("printable_stl_count", stl_count == 15, stl_count, 15)
    record("print_report_part_count", print_gate.get("part_count") == 15, print_gate.get("part_count"), 15)
    record(
        "print_mesh_topology_gate",
        print_gate.get("mesh_topology_gate") == "PASS",
        print_gate.get("mesh_topology_gate"),
        "PASS",
    )
    row_failures = [
        row.get("part")
        for row in print_gate.get("rows", [])
        if row.get("mesh_topology_gate") != "PASS"
        or not row.get("final_watertight")
        or not row.get("final_winding_consistent")
        or int(row.get("boundary_edge_count", -1)) != 0
        or int(row.get("nonmanifold_edge_count", -1)) != 0
        or float(row.get("step_mesh_volume_error_ratio", 1.0)) > 0.005
    ]
    record("print_mesh_row_contract", not row_failures, row_failures, [])

    fit = load_json("reports/round_v3_arm_fit_gate.json")
    fit_failures = [
        row.get("fit")
        for row in fit.get("rows", [])
        if row.get("fit_gate") != "PASS"
        or float(row.get("intersection_volume_mm3", 1.0)) != 0.0
    ]
    record("arm_and_hand_fit_case_count", fit.get("case_count") == 6, fit.get("case_count"), 6)
    record("arm_and_hand_fit_gate", fit.get("fit_gate") == "PASS", fit.get("fit_gate"), "PASS")
    record("arm_and_hand_zero_intersection", not fit_failures, fit_failures, [])


def validate_robot_description() -> None:
    urdf_path = ROOT / "generated" / "urdf" / "zeroth01_rl_round_v1.urdf"
    urdf = ET.parse(urdf_path).getroot()
    links = urdf.findall("link")
    joints = urdf.findall("joint")
    moving = [joint for joint in joints if joint.get("type") != "fixed"]
    urdf_mass = sum(
        float(link.find("./inertial/mass").get("value"))
        for link in links
        if link.find("./inertial/mass") is not None
    )
    record("urdf_link_count", len(links) == 26, len(links), 26)
    record("urdf_joint_count", len(joints) == 25, len(joints), 25)
    record("urdf_moving_joint_count", len(moving) == 16, len(moving), 16)
    record(
        "urdf_total_mass",
        abs(urdf_mass - EXPECTED_TOTAL_MASS_KG) < 1e-9,
        urdf_mass,
        EXPECTED_TOTAL_MASS_KG,
    )

    mjcf = ET.parse(ROOT / "generated" / "mujoco" / "zeroth01_rl_round_v1.xml").getroot()
    actuator_root = mjcf.find("actuator")
    sensor_root = mjcf.find("sensor")
    actuators = list(actuator_root) if actuator_root is not None else []
    sensors = list(sensor_root) if sensor_root is not None else []
    cameras = mjcf.findall(".//camera")
    record("mjcf_actuator_count", len(actuators) == 16, len(actuators), 16)
    record("mjcf_sensor_count", len(sensors) == 8, len(sensors), 8)
    record("mjcf_camera_count", len(cameras) == 1, len(cameras), 1)

    mass = load_json("generated/config/round_v1_mass_properties.json")
    record(
        "printed_overlay_mass",
        abs(float(mass.get("installed_printed_overlay_mass_kg", 0.0)) - 1.2348707885249097) < 1e-12,
        mass.get("installed_printed_overlay_mass_kg"),
        1.2348707885249097,
    )
    record(
        "nominal_electronics_mass",
        abs(float(mass.get("nominal_electronics_mass_kg", 0.0)) - 0.667) < 1e-12,
        mass.get("nominal_electronics_mass_kg"),
        0.667,
    )
    record(
        "nominal_total_mass_with_electronics",
        abs(float(mass.get("nominal_total_mass_with_electronics_kg", 0.0)) - EXPECTED_TOTAL_MASS_KG) < 1e-9,
        mass.get("nominal_total_mass_with_electronics_kg"),
        EXPECTED_TOTAL_MASS_KG,
    )


def validate_simulation_evidence() -> None:
    gate = load_json("reports/mujoco_round_v3_gate.json")
    record("mujoco_full_gate", gate.get("overall") == "PASS", gate.get("overall"), "PASS")
    record("mujoco_random_samples", gate.get("random_sample_count") == 100000, gate.get("random_sample_count"), 100000)
    record("mujoco_random_collisions", gate.get("random_self_collision_samples") == 0, gate.get("random_self_collision_samples"), 0)
    record("mujoco_corner_samples", gate.get("corner_sample_count") == 65536, gate.get("corner_sample_count"), 65536)
    record("mujoco_corner_collisions", gate.get("corner_self_collision_samples") == 0, gate.get("corner_self_collision_samples"), 0)
    record(
        "mujoco_report_mass",
        abs(float(gate.get("total_mass_kg", 0.0)) - EXPECTED_TOTAL_MASS_KG) < 1e-9,
        gate.get("total_mass_kg"),
        EXPECTED_TOTAL_MASS_KG,
    )

    smoke = load_json("reports/mujoco_round_v1_smoke_gate.json")
    record("mujoco_smoke_gate", smoke.get("overall") == "PASS", smoke.get("overall"), "PASS")
    portability = load_json("reports/rl_package_portability_gate.json")
    record("rl_mesh_portability_gate", portability.get("overall") == "PASS", portability.get("overall"), "PASS")
    feasibility = load_json("reports/sts3250_round_v1_feasibility.json")
    record(
        "sts3250_static_gravity_gate",
        feasibility.get("overall_static_gravity_rated_gate") == "PASS",
        feasibility.get("overall_static_gravity_rated_gate"),
        "PASS",
    )
    record(
        "sts3250_walking_claim_boundary",
        str(feasibility.get("walking_feasibility_gate", "")).startswith("UNVERIFIED"),
        feasibility.get("walking_feasibility_gate"),
        "UNVERIFIED_*",
    )


def validate_provenance_and_prompt() -> None:
    source_commit = (
        ROOT
        / "source_assets"
        / "open_source_head"
        / "Poppy-eva-head-design"
        / "SOURCE_COMMIT.txt"
    ).read_text(encoding="utf-8")
    record(
        "poppy_eva_pinned_commit",
        "844654a0b29fb771c23b7400997d1de3d42e0e2e" in source_commit,
        source_commit.strip(),
        "pinned commit present",
    )
    notices = (ROOT / "THIRD_PARTY_NOTICES.md").read_text(encoding="utf-8")
    record("poppy_eva_license_notice", "CC BY-SA 4.0" in notices, "CC BY-SA 4.0" in notices, True)
    record("waveshare_selected_display_notice", "Waveshare 4.3inch DSI QLED" in notices, "Waveshare 4.3inch DSI QLED" in notices, True)

    prompt = (ROOT / "one-seq.md").read_text(encoding="utf-8").strip()
    prompt_copy = (ROOT / "RL_PROMPT.txt").read_text(encoding="utf-8").strip()
    record("one_line_prompt", len(prompt.splitlines()) == 1, len(prompt.splitlines()), 1)
    record("prompt_copy_matches", prompt == prompt_copy, prompt == prompt_copy, True)
    record("prompt_mentions_fixed_chibi_hand", "固定外壳" in prompt, "固定外壳" in prompt, True)

    rejected = [
        "generated/solidworks/portable_flat_round_v2",
        "source_assets/vendor/head_electronics/Waveshare_DualEye_LCD_Module.step",
        "source_assets/vendor/sts3250/FEETECH_STS3250.step",
    ]
    present = [name for name in rejected if (ROOT / name).exists()]
    record("rejected_assets_absent", not present, present, [])


def validate_portability_and_manifest() -> None:
    text_suffixes = {".json", ".csv", ".md", ".txt", ".py", ".yml", ".yaml", ".urdf", ".xml"}
    absolute_hits: list[str] = []
    source_workspace_pattern = re.compile(
        r"[A-Za-z]:[\\/][^\r\n`\"]*roboto_xw[\\/]reference[\\/]zeroth01",
        re.IGNORECASE,
    )
    local_user_pattern = re.compile(r"Users[\\/]thund", re.IGNORECASE)
    files = [path for path in ROOT.rglob("*") if path.is_file() and ".git" not in path.parts]
    for path in files:
        if path.suffix.lower() not in text_suffixes:
            continue
        text = path.read_text(encoding="utf-8-sig", errors="replace")
        if source_workspace_pattern.search(text) or local_user_pattern.search(text):
            absolute_hits.append(path.relative_to(ROOT).as_posix())
    record("no_machine_local_paths", not absolute_hits, absolute_hits, [])

    oversized = [
        {
            "path": path.relative_to(ROOT).as_posix(),
            "bytes": path.stat().st_size,
        }
        for path in files
        if path.stat().st_size > 100 * 1024 * 1024
    ]
    record("no_file_over_100_mib", not oversized, oversized, [])

    manifest = load_json("RELEASE_MANIFEST.json")
    manifest_rows = {str(row["path"]): row for row in manifest.get("files", [])}
    actual_paths = {
        path.relative_to(ROOT).as_posix()
        for path in files
        if path.relative_to(ROOT).as_posix() not in MANIFEST_EXCLUDED
    }
    missing_rows = sorted(actual_paths - set(manifest_rows))
    stale_rows = sorted(set(manifest_rows) - actual_paths)
    hash_failures = []
    for relative in sorted(actual_paths & set(manifest_rows)):
        path = ROOT / relative
        row = manifest_rows[relative]
        if int(row.get("bytes", -1)) != path.stat().st_size or row.get("sha256") != sha256(path):
            hash_failures.append(relative)
    record("manifest_path_set", not missing_rows and not stale_rows, {"missing": missing_rows, "stale": stale_rows}, {"missing": [], "stale": []})
    record("manifest_hashes", not hash_failures, hash_failures, [])


def main() -> None:
    validate_required_files()
    validate_solidworks()
    validate_print_and_fit()
    validate_robot_description()
    validate_simulation_evidence()
    validate_provenance_and_prompt()
    validate_portability_and_manifest()
    failures = [item for item in checks if item["status"] != "PASS"]
    payload = {
        "schema": "zeroth01.release_gate.v3",
        "release": "round_v3_white_eva_small_ears_round_arms_chibi_hands",
        "check_count": len(checks),
        "failure_count": len(failures),
        "overall": "PASS" if not failures else "FAIL",
        "checks": checks,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    if failures:
        sys.exit(1)


if __name__ == "__main__":
    main()
