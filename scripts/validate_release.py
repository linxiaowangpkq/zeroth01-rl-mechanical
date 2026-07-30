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
        ROOT / "generated" / "solidworks" / "portable_flat_round_v1"
    )
    vendor_step = (
        ROOT / "source_assets" / "vendor" / "sts3250" / "FEETECH_STS3250.step"
    )

    urdf_root = ET.parse(canonical_urdf).getroot()
    moving = [
        joint
        for joint in urdf_root.findall("joint")
        if joint.get("type") in {"revolute", "continuous"}
    ]
    urdf_mass = sum(
        float(mass.get("value", "0"))
        for mass in urdf_root.findall("./link/inertial/mass")
    )
    urdf_links = {
        link.get("name", "") for link in urdf_root.findall("./link")
    }
    mjcf_root = ET.parse(canonical_mjcf).getroot()

    mujoco_gate = load_json("reports/mujoco_round_v1_gate.json")
    print_gate = load_json("reports/round_v1_print_mesh_gate.json")
    fit_gate = load_json("reports/round_v1_interface_fit_mesh_gate.json")
    interface_gate = load_json(
        "reports/round_v1_integrated_interface_gate.json"
    )
    portability_gate = load_json("reports/rl_package_portability_gate.json")
    solidworks_gate = load_json(
        "generated/solidworks/portable_flat_round_v1/"
        "solidworks_portable_gate.json"
    )
    feasibility = load_json("reports/sts3250_round_v1_feasibility.json")
    mass_properties = load_json(
        "generated/config/round_v1_mass_properties.json"
    )
    electronics = load_json(
        "generated/config/round_v1_electronics_sensor_layout.json"
    )
    expected_mass = (
        float(mass_properties["round_v1_nominal_total_mass_kg"])
        + float(electronics["nominal_electronics_mass_kg"])
    )

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
    local_links = [
        target
        for target in re.findall(r"\[[^\]]+\]\(([^)]+)\)", readme)
        if not target.startswith(("http://", "https://", "#"))
    ]
    missing_readme_links = [
        target for target in local_links if not (ROOT / target).exists()
    ]
    local_path_pattern = re.compile(
        r"(?:[A-Za-z]:\\\\(?:Users|Codex)\\\\|[A-Za-z]:\\(?:Users|Codex)\\)"
    )
    local_path_files: list[str] = []
    text_roots = [
        ROOT / "reports",
        ROOT / "generated" / "config",
        ROOT / "generated" / "solidworks" / "portable_flat_round_v1",
    ]
    for text_root in text_roots:
        for path in text_root.rglob("*"):
            if path.is_file() and path.suffix.lower() in {".csv", ".json", ".md", ".txt"}:
                if local_path_pattern.search(path.read_text(encoding="utf-8-sig")):
                    local_path_files.append(path.relative_to(ROOT).as_posix())

    checks = {
        "canonical_urdf_exists": canonical_urdf.is_file(),
        "canonical_mjcf_exists": canonical_mjcf.is_file(),
        "urdf_moving_joint_count_16": len(moving) == 16,
        "urdf_link_count_23": len(urdf_links) == 23,
        "urdf_electronics_links_present": {
            "camera_module",
            "camera_optical_frame",
            "imu_module",
            "compute_module",
            "battery_pack",
        }.issubset(urdf_links),
        "urdf_mass_matches_round_manifest": abs(
            urdf_mass - expected_mass
        ) < 1e-9,
        "electronics_rl_layout_gate": (
            electronics.get("rl_use_gate")
            == "PASS_WITH_ASSUMED_MASS_AND_SENSOR_PARAMETERS"
        ),
        "mjcf_sensor_count_7": len(mjcf_root.findall("./sensor/*")) == 7,
        "mjcf_camera_count_1": len(mjcf_root.findall(".//camera")) == 1,
        "mujoco_full_gate_pass": mujoco_gate.get("overall") == "PASS",
        "print_mesh_topology_pass": print_gate.get("mesh_topology_gate") == "PASS",
        "print_part_count_11": print_gate.get("part_count") == 11,
        "interface_fit_mesh_topology_pass": (
            fit_gate.get("mesh_topology_gate") == "PASS"
            and fit_gate.get("part_count") == 3
        ),
        "integrated_interface_gate_pass": (
            interface_gate.get("overall")
            == "PASS_WITH_HARDWARE_LIMITATIONS"
        ),
        "cad_step_part_count_22": len(
            list((ROOT / "generated" / "cad" / "round_v1" / "parts").glob("*.step"))
        )
        == 22,
        "final_print_stl_count_11": len(
            list(
                (
                    ROOT / "generated" / "print" / "round_v1" / "final"
                ).glob("*.stl")
            )
        )
        == 11,
        "final_interface_fit_stl_count_3": len(
            list(
                (
                    ROOT
                    / "generated"
                    / "print"
                    / "round_v1"
                    / "fit_check_non_load_bearing"
                    / "final"
                ).glob("*.stl")
            )
        )
        == 3,
        "mesh_path_portability_pass": portability_gate.get("overall") == "PASS",
        "solidworks_portable_gate_pass": (
            solidworks_gate.get("overall_review_gate")
            == "PASS_WITH_HARDWARE_LIMITATIONS"
        ),
        "solidworks_component_count_81": (
            solidworks_gate.get("component_count") == 81
        ),
        "solidworks_parent_child_transmission_pass": (
            solidworks_gate.get(
                "parent_housing_child_output_transmission_gate"
            )
            == "PASS"
        ),
        "solidworks_motion_gif_pass": (
            solidworks_gate.get("motion_gif_gate") == "PASS"
            and (
                ROOT
                / "snapshots"
                / "solidworks"
                / "round_v1"
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
        "solidworks_assembly_present": (
            portable_dir
            / "OPEN_FIRST_ZEROTH01_ROUND_V1_WITH_STS3250.SLDASM"
        ).is_file(),
        "vendor_step_checksum": sha256(vendor_step) == VENDOR_SHA256,
        "static_torque_gate_pass": (
            feasibility.get("overall_static_gravity_rated_gate") == "PASS"
        ),
        "walking_is_not_overclaimed": str(
            feasibility.get("walking_feasibility_gate", "")
        ).startswith("UNVERIFIED"),
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
        "schema": "zeroth01.rl_mechanical.release_gate.v1",
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
