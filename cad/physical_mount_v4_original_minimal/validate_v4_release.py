"""Validate the v4 digital handoff and write a hash-addressed release manifest."""

from __future__ import annotations

import hashlib
import csv
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[2]
REPORT_DIR = ROOT / "reports" / "v4_original_minimal"
RELEASE_REPORT = REPORT_DIR / "release_gate.json"
RELEASE_MANIFEST = ROOT / "RELEASE_MANIFEST.json"
URDF = ROOT / "generated" / "urdf" / "physical_mount_v4_original_minimal" / "zeroth01_physical_mount_v4_original_minimal_18dof.urdf"
MJCF = ROOT / "generated" / "mujoco" / "physical_mount_v4_original_minimal" / "zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml"
STEP = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY.step"
NORMAL_ASM = ROOT / "generated" / "solidworks" / "physical_mount_v4_original_minimal" / "portable_flat" / "OPEN_FIRST_ZEROTH01_V4_ORIGINAL_MINIMAL_WHITE_18_BLUE_STS3250.SLDASM"
XRAY_ASM = ROOT / "generated" / "solidworks" / "physical_mount_v4_original_minimal" / "portable_flat" / "OPTIONAL_XRAY_ZEROTH01_V4_ORIGINAL_MINIMAL_INTERNAL_LAYOUT.SLDASM"
ACTUATOR_CONFIG = ROOT / "generated" / "config" / "physical_mount_v4_original_minimal_actuator_layout.json"
RL_CONFIG = ROOT / "generated" / "config" / "physical_mount_v4_original_minimal_rl_handoff.json"
ASSEMBLY_MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
CONNECTIVITY_REPORT = REPORT_DIR / "mechanical_connectivity_gate.json"


def read_json(path: Path):
    return json.loads(path.read_text(encoding="utf-8"))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def collect_release_files() -> list[Path]:
    roots = [
        ROOT / "cad" / "physical_mount_v4_original_minimal",
        ROOT / "generated" / "mujoco" / "physical_mount_v4_original_minimal",
        ROOT / "generated" / "urdf" / "physical_mount_v4_original_minimal",
        ROOT / "snapshots" / "cad" / "physical_mount_v4_original_minimal",
        ROOT / "snapshots" / "motion" / "physical_mount_v4_original_minimal",
    ]
    assembly_manifest = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
    portable_manifest = ROOT / "generated" / "solidworks" / "physical_mount_v4_original_minimal" / "portable_flat" / assembly_manifest.name
    component_csv = REPORT_DIR / "solidworks_component_manifest.csv"
    explicit = [
        ACTUATOR_CONFIG,
        RL_CONFIG,
        STEP,
        assembly_manifest,
        portable_manifest,
        NORMAL_ASM,
        XRAY_ASM,
        ROOT / "README.md",
        ROOT / "README_zh.md",
        ROOT / "ASSEMBLY_GUIDE_zh.md",
        ROOT / "PROCUREMENT_BOM.csv",
        ROOT / "bug.md",
        ROOT / "one-seq.md",
        ROOT / "RL_PROMPT.txt",
        ROOT / "THIRD_PARTY_NOTICES.md",
        ROOT / "source_assets" / "step_parts" / "feetech_sts3250.step",
        ROOT / "scripts" / "create_solidworks_round_v1_review.py",
        ROOT / "snapshots" / "solidworks" / "v4_original_minimal" / "v4_normal_rl_front_upright.png",
        ROOT / "snapshots" / "solidworks" / "v4_original_minimal" / "v4_normal_rl_rear_upright.png",
        REPORT_DIR / "cad_build.json",
        REPORT_DIR / "cad_render_evidence.json",
        REPORT_DIR / "coordinated_motion_evidence.json",
        REPORT_DIR / "mjcf_compile_gate.json",
        CONNECTIVITY_REPORT,
        RELEASE_REPORT,
        component_csv,
        REPORT_DIR / "solidworks_gate.json",
        REPORT_DIR / "solidworks_interference_gate.json",
        REPORT_DIR / "sts3250_quasistatic_torque_gate.json",
        REPORT_DIR / "urdf_mass_inertia_gate.json",
    ]

    manifest_payload = read_json(assembly_manifest)
    for component in manifest_payload["components"]:
        source = ROOT / str(component["source"])
        explicit.append(source)
        if source.suffix.lower() == ".step":
            stl = source.with_suffix(".stl")
            if stl.is_file():
                explicit.append(stl)
    for filename in (
        "head_mount_4xm3_drill_jig",
        "harness_strain_relief_guides",
        "torso_imu_shelf",
    ):
        explicit.extend(
            [
                ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "parts" / f"{filename}.step",
                ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "parts" / f"{filename}.stl",
            ]
        )
    with component_csv.open(newline="", encoding="utf-8-sig") as stream:
        for row in csv.DictReader(stream):
            explicit.append(NORMAL_ASM.parent / row["native_part"])
    def releasable(path: Path) -> bool:
        relative = path.relative_to(ROOT)
        return (
            path.is_file()
            and path.suffix.lower() not in {".log", ".pyc"}
            and "__pycache__" not in relative.parts
            and "tessellation_cache" not in relative.parts
            and not path.name.startswith(".")
        )

    files = set(path for path in explicit if releasable(path))
    for base in roots:
        if base.is_dir():
            files.update(path for path in base.rglob("*") if releasable(path))
    return sorted(files, key=lambda path: path.relative_to(ROOT).as_posix())


def main() -> int:
    sw = read_json(REPORT_DIR / "solidworks_gate.json")
    interference = read_json(REPORT_DIR / "solidworks_interference_gate.json")
    urdf_gate = read_json(REPORT_DIR / "urdf_mass_inertia_gate.json")
    mjcf_gate = read_json(REPORT_DIR / "mjcf_compile_gate.json")
    motion = read_json(REPORT_DIR / "coordinated_motion_evidence.json")
    torque = read_json(REPORT_DIR / "sts3250_quasistatic_torque_gate.json")
    actuator = read_json(ACTUATOR_CONFIG)
    rl = read_json(RL_CONFIG)
    manifest = read_json(ASSEMBLY_MANIFEST)
    connectivity = read_json(CONNECTIVITY_REPORT)
    manifest_sha256 = sha256(ASSEMBLY_MANIFEST)

    robot = ET.parse(URDF).getroot()
    revolute = [joint for joint in robot.findall("joint") if joint.get("type") == "revolute"]
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    compiled_mass = float(model.body_mass.sum())

    checks = {
        "solidworks_native_assembly": (
            sw.get("overall") == "PASS"
            and sw.get("assembly_component_count") == manifest.get("component_count")
            and sw.get("manifest_component_count") == manifest.get("component_count")
            and sw.get("manifest_sha256") == manifest_sha256
        ),
        "independent_sts3250_occurrences": sw.get("separate_blue_sts3250_count") == 18,
        "solidworks_height_le_500_mm": sw.get("standing_height_gate") == "PASS" and sw.get("standing_height_mm", 999.0) <= 500.0,
        "solidworks_cross_component_interference": (
            interference.get("overall") == "PASS"
            and interference.get("physical_interference_count") == 0
            and interference.get("manifest_sha256") == manifest_sha256
        ),
        "mechanical_connectivity_18": (
            connectivity.get("overall") == "PASS"
            and connectivity.get("joint_count") == 18
            and connectivity.get("exact_sts3250_count") == 18
            and connectivity.get("output_bridge_count") == 18
        ),
        "urdf_18dof": len(revolute) == 18 and urdf_gate.get("movable_joint_count") == 18,
        "nominal_mass_le_3kg": abs(compiled_mass - 2.85) < 1.0e-9 and urdf_gate.get("mass_gate") == "PASS",
        "mjcf_runtime_compile": mjcf_gate.get("runtime_compile_gate") == "PASS" and model.nu == 18,
        "coordinated_collision_sweep": motion.get("gate") == "PASS" and not motion.get("non_ground_penetrations"),
        "quasistatic_torque_below_continuous": torque.get("gate") == "PASS_NOMINAL_QUASISTATIC_CONTINUOUS",
        "actuator_ledger_18": actuator.get("count") == 18 and len(actuator.get("actuators", [])) == 18,
        "rl_handoff_points_to_v4": rl.get("movable_joint_count") == 18 and rl.get("nominal_total_mass_kg") == 2.85,
        "authoritative_files_present": all(path.is_file() for path in (URDF, MJCF, STEP, NORMAL_ASM, XRAY_ASM)),
    }
    digital_gate = "PASS" if all(checks.values()) else "FAIL"
    payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.release_gate.v1",
        "digital_rl_release_gate": digital_gate,
        "physical_first_article_gate": "HOLD",
        "checks": checks,
        "facts": {
            "solidworks_component_count": sw.get("assembly_component_count"),
            "sts3250_count": sw.get("separate_blue_sts3250_count"),
            "standing_height_mm": sw.get("standing_height_mm"),
            "nominal_mass_kg": compiled_mass,
            "revolute_joint_count": len(revolute),
            "quasistatic_peak_joint": torque.get("peak_joint"),
            "quasistatic_peak_torque_nm": torque.get("peak_quasistatic_torque_nm"),
        },
        "hold_items": [
            "print and assemble one load-bearing first article",
            "dimensionally inspect purchased STS3250 case, 4xM2 pattern, 25T horn and rear support",
            "scan bus IDs and jog-calibrate neutral counts and direction signs",
            "weigh every as-built link and identify inertia, damping, friction and motor strength",
            "run fused bench current/thermal tests and slow suspended motion before ground contact",
            "close RL rollout peak/RMS torque, impact, current and temperature policy traces",
        ],
        "truth_boundary": "PASS releases the digital CAD/URDF/MJCF baseline for RL; it is not a factory print-and-run or powered-dynamic signoff.",
    }
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    RELEASE_REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    files = collect_release_files()
    manifest = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.release_manifest.v1",
        "release": "v4-original-minimal-digital-rl",
        "digital_rl_release_gate": digital_gate,
        "physical_first_article_gate": "HOLD",
        "file_count": len(files),
        "files": [
            {
                "path": path.relative_to(ROOT).as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
            for path in files
        ],
    }
    RELEASE_MANIFEST.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"digital_rl_release_gate": digital_gate, "physical_first_article_gate": "HOLD", "file_count": len(files)}, indent=2))
    return 0 if digital_gate == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
