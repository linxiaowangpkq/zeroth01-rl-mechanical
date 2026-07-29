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

    mujoco_gate = load_json("reports/mujoco_round_v1_gate.json")
    print_gate = load_json("reports/round_v1_print_mesh_gate.json")
    portability_gate = load_json("reports/rl_package_portability_gate.json")
    solidworks_gate = load_json("reports/solidworks_portable_gate.json")
    feasibility = load_json("reports/sts3250_round_v1_feasibility.json")

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
    text_roots = [ROOT / "reports", ROOT / "generated" / "config"]
    for text_root in text_roots:
        for path in text_root.rglob("*"):
            if path.is_file() and path.suffix.lower() in {".csv", ".json", ".md", ".txt"}:
                if local_path_pattern.search(path.read_text(encoding="utf-8-sig")):
                    local_path_files.append(path.relative_to(ROOT).as_posix())

    checks = {
        "canonical_urdf_exists": canonical_urdf.is_file(),
        "canonical_mjcf_exists": canonical_mjcf.is_file(),
        "urdf_moving_joint_count_16": len(moving) == 16,
        "urdf_mass_matches_round_manifest": abs(
            urdf_mass - 4.151924609464
        ) < 1e-9,
        "mujoco_full_gate_pass": mujoco_gate.get("overall") == "PASS",
        "print_mesh_topology_pass": print_gate.get("mesh_topology_gate") == "PASS",
        "print_part_count_11": print_gate.get("part_count") == 11,
        "cad_step_part_count_11": len(
            list((ROOT / "generated" / "cad" / "round_v1" / "parts").glob("*.step"))
        )
        == 11,
        "final_print_stl_count_11": len(
            list(
                (
                    ROOT / "generated" / "print" / "round_v1" / "final"
                ).glob("*.stl")
            )
        )
        == 11,
        "mesh_path_portability_pass": portability_gate.get("overall") == "PASS",
        "solidworks_portable_gate_pass": (
            solidworks_gate.get("overall_review_gate")
            == "PASS_WITH_HARDWARE_LIMITATIONS"
        ),
        "solidworks_part_count_28": len(
            [
                path
                for path in portable_dir.glob("*.SLDPRT")
                if not path.name.startswith("~$")
            ]
        )
        == 28,
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
