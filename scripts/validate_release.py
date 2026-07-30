from __future__ import annotations

import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
REPORT = ROOT / "reports" / "release_gate.json"
VENDOR_SHA256 = "cf46f17da455e1f158114791bb31404c24d925e8a758bbd6189f8ee815a571bf"
MAX_GIT_BLOB_BYTES = 100 * 1024 * 1024


def load_json(relative: str):
    return json.loads((ROOT / relative).read_text(encoding="utf-8-sig"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    canonical_urdf = ROOT / "generated" / "urdf" / "zeroth01_rl_round_v1.urdf"
    canonical_mjcf = ROOT / "generated" / "mujoco" / "zeroth01_rl_round_v1.xml"
    portable_dir = (
        ROOT / "generated" / "solidworks" / "portable_flat_round_v2"
    )
    portable_assembly = (
        portable_dir
        / "OPEN_FIRST_ZEROTH01_ROUND_V2_MINIMAL_COSMETIC.SLDASM"
    )
    vendor_step = (
        ROOT / "source_assets" / "vendor" / "sts3250" / "FEETECH_STS3250.step"
    )

    urdf_root = ET.parse(canonical_urdf).getroot()
    urdf_links = {link.get("name", "") for link in urdf_root.findall("link")}
    urdf_joints = urdf_root.findall("joint")
    moving = [
        joint
        for joint in urdf_joints
        if joint.get("type") in {"revolute", "continuous"}
    ]
    urdf_mass = sum(
        float(mass.get("value", "0"))
        for mass in urdf_root.findall("./link/inertial/mass")
    )
    mjcf_root = ET.parse(canonical_mjcf).getroot()

    mujoco_gate = load_json("reports/mujoco_round_v1_gate.json")
    smoke_gate = load_json("reports/mujoco_round_v1_smoke_gate.json")
    print_gate = load_json("reports/round_v1_print_mesh_gate.json")
    portability_gate = load_json("reports/rl_package_portability_gate.json")
    solidworks_gate = load_json(
        "generated/solidworks/portable_flat_round_v2/"
        "solidworks_portable_gate.json"
    )
    feasibility = load_json("reports/sts3250_round_v1_feasibility.json")
    mass_properties = load_json(
        "generated/config/round_v1_mass_properties.json"
    )
    electronics = load_json(
        "generated/config/round_v1_electronics_sensor_layout.json"
    )
    component_identity = load_json("config/round_v2_component_identity.json")
    servo_geometry = load_json("config/round_v2_servo_interface_geometry.json")
    servo_provenance = load_json(
        "source_assets/vendor/sts3250/PROVENANCE.json"
    )
    head_provenance = load_json(
        "source_assets/vendor/head_electronics/PROVENANCE.json"
    )

    expected_mass = (
        float(mass_properties["round_v1_nominal_total_mass_kg"])
        + float(electronics["nominal_electronics_mass_kg"])
    )
    expected_electronics_links = {
        "eye_display_module",
        "camera_module",
        "tof_module",
        "imu_module",
        "compute_module",
        "battery_pack",
        "camera_optical_frame",
        "tof_optical_frame",
    }
    expected_cad_parts = {
        "ZEROTH01_ROUND_V1_BATTERY_PACK.step",
        "ZEROTH01_ROUND_V1_CAMERA_LENSES.step",
        "ZEROTH01_ROUND_V1_CAMERA_MODULE.step",
        "ZEROTH01_ROUND_V1_CHEST_BACK.step",
        "ZEROTH01_ROUND_V1_CHEST_FRONT.step",
        "ZEROTH01_ROUND_V1_COMPUTE_MODULE.step",
        "ZEROTH01_ROUND_V1_HEAD_BACK.step",
        "ZEROTH01_ROUND_V1_HEAD_FRONT.step",
        "ZEROTH01_ROUND_V1_IMU_MODULE.step",
        "ZEROTH01_ROUND_V1_JOINT_RING.step",
        "ZEROTH01_ROUND_V1_LEFT_SOLE.step",
        "ZEROTH01_ROUND_V1_MUZZLE_BADGE.step",
        "ZEROTH01_ROUND_V1_PELVIS_BACK.step",
        "ZEROTH01_ROUND_V1_PELVIS_FRONT.step",
        "ZEROTH01_ROUND_V1_RIGHT_SOLE.step",
        "ZEROTH01_ROUND_V1_TORSO_SPINE.step",
        "ZEROTH01_ROUND_V1_VISOR_BADGE.step",
        "ZEROTH01_ROUND_V2_EYE_DISPLAY_MODULE.step",
        "ZEROTH01_ROUND_V2_TOF_MODULE.step",
    }
    actual_cad_parts = {
        path.name
        for path in (
            ROOT / "generated" / "cad" / "round_v1" / "parts"
        ).glob("*.step")
    }

    all_files = [
        path
        for path in ROOT.rglob("*")
        if path.is_file() and ".git" not in path.relative_to(ROOT).parts
    ]
    oversized = [
        path.relative_to(ROOT).as_posix()
        for path in all_files
        if path.stat().st_size >= MAX_GIT_BLOB_BYTES
    ]

    prompt = (ROOT / "RL_PROMPT.txt").read_text(encoding="utf-8").strip()
    one_seq = (ROOT / "one-seq.md").read_text(encoding="utf-8").strip()
    readme = (ROOT / "README.md").read_text(encoding="utf-8")
    readme_zh = (ROOT / "README_zh.md").read_text(encoding="utf-8")
    local_links = [
        target
        for target in re.findall(r"\[[^\]]+\]\(([^)]+)\)", readme)
        if not target.startswith(("http://", "https://", "#"))
    ]
    missing_readme_links = [
        target for target in local_links if not (ROOT / target).exists()
    ]

    local_path_pattern = re.compile(
        r"(?:[A-Za-z]:[\\/](?:Users|Codex)[\\/])"
    )
    local_path_files: list[str] = []
    text_roots = [
        ROOT / "reports",
        ROOT / "generated" / "config",
        ROOT / "generated" / "solidworks" / "portable_flat_round_v2",
    ]
    for text_root in text_roots:
        for path in text_root.rglob("*"):
            if (
                path.is_file()
                and path.suffix.lower() in {".csv", ".json", ".md", ".txt"}
                and local_path_pattern.search(
                    path.read_text(encoding="utf-8-sig")
                )
            ):
                local_path_files.append(path.relative_to(ROOT).as_posix())

    obsolete_current_release_paths = [
        "generated/solidworks/portable_flat_round_v1",
        "generated/cad/round_v1/parts/ZEROTH01_ROUND_V1_SERVO_CAGE.step",
        "generated/cad/round_v1/parts/ZEROTH01_ROUND_V1_OUTPUT_HUB.step",
        "generated/config/zeroth01_sts3250_mount_phase.json",
    ]

    checks = {
        "canonical_urdf_exists": canonical_urdf.is_file(),
        "canonical_mjcf_exists": canonical_mjcf.is_file(),
        "urdf_link_count_26": len(urdf_links) == 26,
        "urdf_joint_count_25": len(urdf_joints) == 25,
        "urdf_moving_joint_count_16": len(moving) == 16,
        "urdf_head_and_torso_electronics_links_present": (
            expected_electronics_links.issubset(urdf_links)
        ),
        "colored_markers_not_in_urdf": not any(
            re.fullmatch(r"S(?:0[1-9]|1[0-6])", name) for name in urdf_links
        ),
        "urdf_mass_matches_round_manifest": abs(
            urdf_mass - expected_mass
        ) < 1e-9,
        "electronics_rl_layout_gate": (
            electronics.get("rl_use_gate")
            == "PASS_WITH_SELECTED_HEAD_MODULES_AND_ASSUMED_TORSO_PAYLOADS"
        ),
        "component_identity_has_16_stable_ids": (
            [item.get("id") for item in component_identity.get("servos", [])]
            == [f"S{index:02d}" for index in range(1, 17)]
        ),
        "old_servo_step_is_quarantined": (
            servo_provenance.get("status")
            == "QUARANTINED_NOT_A_CONFIRMED_STS3250_C001_MODEL"
            and servo_provenance.get("selected_assembly_usage") == "NONE"
            and servo_geometry.get("selected_geometry_policy")
            == "PRESERVE_FROZEN_ZEROTH01_ASSEMBLY_NO_REPLACEMENT_SERVO_BODY"
        ),
        "exact_head_vendor_assets_present": (
            len(head_provenance.get("assets", [])) == 2
            and (
                ROOT
                / "source_assets/vendor/head_electronics/"
                "Waveshare_DualEye_LCD_Module.step"
            ).is_file()
            and (
                ROOT
                / "source_assets/vendor/head_electronics/"
                "Raspberry_Pi_Camera_Module_3_Wide.step"
            ).is_file()
        ),
        "mjcf_sensor_count_8": len(mjcf_root.findall("./sensor/*")) == 8,
        "mjcf_camera_count_1": len(mjcf_root.findall(".//camera")) == 1,
        "mujoco_full_gate_pass": mujoco_gate.get("overall") == "PASS",
        "mujoco_smoke_1000_steps_pass": (
            smoke_gate.get("overall") == "PASS"
            and smoke_gate.get("steps") == 1000
            and smoke_gate.get("nbody") == 26
            and smoke_gate.get("nsensor") == 8
        ),
        "print_mesh_topology_pass_11": (
            print_gate.get("mesh_topology_gate") == "PASS"
            and print_gate.get("part_count") == 11
        ),
        "print_load_path_not_overclaimed": str(
            print_gate.get("functional_load_path_gate", "")
        ).startswith("FAIL"),
        "mesh_path_portability_pass": portability_gate.get("overall") == "PASS",
        "solidworks_portable_gate_pass": (
            solidworks_gate.get("overall_review_gate")
            == "PASS_MINIMAL_COSMETIC_OVERLAY_WITH_HARDWARE_LIMITATIONS"
        ),
        "solidworks_minimal_component_accounting": (
            solidworks_gate.get("component_count") == 51
            and solidworks_gate.get("source_link_component_count") == 17
            and solidworks_gate.get("round_overlay_component_count") == 18
            and solidworks_gate.get(
                "nonphysical_colored_joint_marker_count"
            )
            == 16
        ),
        "solidworks_no_replacement_servo_cage_or_hub": (
            solidworks_gate.get("explicit_replacement_sts3250_component_count")
            == 0
            and solidworks_gate.get("new_servo_cage_component_count") == 0
            and solidworks_gate.get("new_child_output_hub_component_count")
            == 0
        ),
        "solidworks_transform_and_motion_pass": (
            solidworks_gate.get("transform_gate") == "PASS"
            and solidworks_gate.get(
                "baseline_joint_transform_semantics_gate"
            )
            == "PASS"
            and solidworks_gate.get("motion_gif_gate") == "PASS"
            and (
                portable_dir
                / "previews"
                / "zeroth01_round_v1_solidworks_motion.gif"
            ).is_file()
        ),
        "solidworks_part_count_36": len(
            [
                path
                for path in portable_dir.glob("*.SLDPRT")
                if not path.name.startswith("~$")
            ]
        )
        == 36,
        "solidworks_assembly_present": portable_assembly.is_file(),
        "selected_cad_step_part_set_19": actual_cad_parts == expected_cad_parts,
        "cosmetic_electronics_step_present": (
            ROOT
            / "generated/cad/round_v1/"
            "ZEROTH01_ROUND_V2_COSMETIC_ELECTRONICS_ASSEMBLY.step"
        ).is_file(),
        "final_print_stl_count_11": len(
            list(
                (
                    ROOT
                    / "generated"
                    / "print"
                    / "round_v1"
                    / "final"
                ).glob("*.stl")
            )
        )
        == 11,
        "obsolete_replacement_servo_release_removed": not any(
            (ROOT / relative).exists()
            for relative in obsolete_current_release_paths
        ),
        "vendor_step_checksum": sha256(vendor_step) == VENDOR_SHA256,
        "static_torque_report_is_current": (
            abs(
                float(feasibility.get("round_v1_total_mass_kg", 0.0))
                - urdf_mass
            )
            < 1e-9
            and feasibility.get("sample_count") == 100000
            and feasibility.get("overall_static_gravity_rated_gate") == "PASS"
        ),
        "walking_is_not_overclaimed": str(
            feasibility.get("walking_feasibility_gate", "")
        ).startswith("UNVERIFIED"),
        "readmes_do_not_claim_print_and_walk": (
            "It is **not** a claim that the entire robot can be printed"
            in readme
            and "打印后直接拼装并行走：不可宣称" in readme_zh
        ),
        "no_git_blob_at_or_over_100_mib": not oversized,
        "no_machine_local_paths_in_release_data": not local_path_files,
        "single_line_rl_prompt": "\n" not in prompt and prompt.endswith("。"),
        "single_line_one_seq_prompt": (
            "\n" not in one_seq
            and one_seq.endswith("。")
            and one_seq == prompt
        ),
        "readme_local_links_resolve": not missing_readme_links,
    }
    payload = {
        "schema": "zeroth01.rl_mechanical.minimal_cosmetic_release_gate.v2",
        "canonical_urdf": canonical_urdf.relative_to(ROOT).as_posix(),
        "canonical_mjcf": canonical_mjcf.relative_to(ROOT).as_posix(),
        "nominal_mass_kg": urdf_mass,
        "moving_joint_count": len(moving),
        "file_count": len(all_files),
        "largest_file_bytes": max(path.stat().st_size for path in all_files),
        "oversized_files": oversized,
        "machine_local_path_files": local_path_files,
        "missing_readme_links": missing_readme_links,
        "checks": checks,
        "overall": "PASS" if all(checks.values()) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
