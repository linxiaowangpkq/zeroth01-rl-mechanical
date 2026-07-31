"""Validate the portable physical-mount-v2-minimal RL release."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


URDF_REL = Path("generated/urdf/physical_mount_v2_minimal/zeroth01_physical_mount_v2_minimal.urdf")
HANDOFF_REL = Path("generated/config/physical_mount_v2_minimal_rl_handoff.json")
ACTUATORS_REL = Path("generated/config/physical_mount_v1_actuators.json")
GEOMETRY_REL = Path("reports/physical_mount_v2_minimal/geometry_gate.json")
COLLISION_REL = Path("reports/physical_mount_v2_minimal/dynamic_collision_gate.json")
SOLIDWORKS_REL = Path("reports/physical_mount_v2_minimal/solidworks_gate.json")
PORTABLE_SOLIDWORKS_REL = Path("reports/physical_mount_v2_minimal/solidworks_portable_open_gate.json")
MASS_REL = Path("generated/config/physical_mount_v2_minimal_mass_properties.json")
OUTPUT_REL = Path("reports/physical_mount_v2_minimal/release_gate.json")
PUBLISHED_VTK_GIF_REL = Path("snapshots/physical_mount_v2_minimal/physical_mount_v2_minimal_16dof_motion.gif")
SOURCE_VTK_GIF_REL = Path("snapshots/cad/physical_mount_v2_minimal/physical_mount_v2_minimal_16dof_motion.gif")
SOLIDWORKS_GIF_REL = Path("snapshots/solidworks/physical_mount_v2_minimal/solidworks_physical_mount_v2_minimal_16dof_motion.gif")
PORTABLE_SW_ROOT_REL = Path("generated/solidworks/physical_mount_v2_minimal/portable_flat")
PORTABLE_SW_ASM_REL = PORTABLE_SW_ROOT_REL / "OPEN_FIRST_ZEROTH01_PHYSICAL_MOUNT_V2_MINIMAL_16_BLUE_SERVOS_XRAY.SLDASM"


def _json(root: Path, relative: Path) -> dict[str, object]:
    return json.loads((root / relative).read_text(encoding="utf-8"))


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate(root: Path) -> dict[str, object]:
    root = root.resolve()
    checks: dict[str, dict[str, object]] = {}
    vtk_gif_relative = PUBLISHED_VTK_GIF_REL if (root / PUBLISHED_VTK_GIF_REL).is_file() else SOURCE_VTK_GIF_REL

    def check(name: str, passed: bool, evidence: object) -> None:
        checks[name] = {"gate": "PASS" if passed else "FAIL", "evidence": evidence}

    required = [
        URDF_REL,
        HANDOFF_REL,
        ACTUATORS_REL,
        GEOMETRY_REL,
        COLLISION_REL,
        SOLIDWORKS_REL,
        PORTABLE_SOLIDWORKS_REL,
        MASS_REL,
        Path("config/physical_mount_v1_guarded_limits.json"),
        Path("generated/config/physical_mount_v1_hardware_calibration_template.csv"),
        vtk_gif_relative,
        SOLIDWORKS_GIF_REL,
        PORTABLE_SW_ASM_REL,
    ]
    missing = [relative.as_posix() for relative in required if not (root / relative).is_file()]
    check("required_files", not missing, missing)

    urdf = root / URDF_REL
    robot = ET.parse(urdf).getroot()
    revolute = [joint for joint in robot.findall("joint") if joint.get("type") == "revolute"]
    check("urdf_revolute_joint_count", len(revolute) == 16, len(revolute))

    inertial_failures: list[str] = []
    total_mass = 0.0
    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass = inertial.find("mass")
        tensor = inertial.find("inertia")
        if mass is None or tensor is None:
            inertial_failures.append(f"{link.get('name')}:incomplete")
            continue
        value = float(mass.get("value", "nan"))
        diagonal = [float(tensor.get(axis, "nan")) for axis in ("ixx", "iyy", "izz")]
        if not math.isfinite(value) or value <= 0.0 or any(not math.isfinite(v) or v <= 0.0 for v in diagonal):
            inertial_failures.append(str(link.get("name")))
        total_mass += value
    check("urdf_inertials", not inertial_failures, inertial_failures)
    check("urdf_total_mass_kg", abs(total_mass - 5.216675047851401) <= 1.0e-9, total_mass)

    missing_meshes: list[str] = []
    for mesh in robot.findall(".//mesh"):
        filename = mesh.get("filename", "")
        if filename.startswith("package://") or filename.startswith("file://"):
            missing_meshes.append(filename)
            continue
        if not (urdf.parent / filename).is_file():
            missing_meshes.append(filename)
    check("urdf_mesh_portability", not missing_meshes, sorted(set(missing_meshes)))

    hand_meshes: dict[str, list[str]] = {}
    for name in ("FINGER_1", "FINGER_1_2"):
        link = robot.find(f"./link[@name='{name}']")
        hand_meshes[name] = [mesh.get("filename", "") for mesh in link.findall(".//visual/geometry/mesh")]
    hand_ok = all(len(paths) == 1 and "minimal/" in paths[0] and "q_hand" in paths[0] for paths in hand_meshes.values())
    check("q_hands_replace_old_claws", hand_ok, hand_meshes)

    limits_ok = all(
        joint.find("limit") is not None
        and float(joint.find("limit").get("lower")) < float(joint.find("limit").get("upper"))
        and abs(float(joint.find("limit").get("effort")) - 1.569) <= 1.0e-9
        and abs(float(joint.find("limit").get("velocity")) - 3.0) <= 1.0e-9
        for joint in revolute
    )
    check("joint_limits_and_rated_actuation", limits_ok, "16 joints; effort=1.569 N*m; velocity=3 rad/s")

    actuators = _json(root, ACTUATORS_REL)
    ids = [row["id"] for row in actuators["servos"]]
    joints = [row["joint"] for row in actuators["servos"]]
    actuator_ok = (
        actuators["count"] == 16
        and len(set(ids)) == 16
        and len(set(joints)) == 16
        and actuators["target_model"] == "FEETECH STS3250-C001"
        and all(abs(float(row["rated_torque_nm"]) - 1.569) <= 1.0e-9 for row in actuators["servos"])
    )
    check("actuator_metadata", actuator_ok, {"ids": ids, "target": actuators["target_model"]})

    geometry = _json(root, GEOMETRY_REL)
    post = geometry["retained_head_post_visibility"]
    geometry_ok = geometry["overall"] == "PASS" and post["maximum_exposed_post_height_mm"] <= post["requirement_mm"] <= 5.0
    check("geometry_and_head_post", geometry_ok, post)

    collision = _json(root, COLLISION_REL)
    collision_ok = (
        collision["overall"] == "PASS"
        and collision["neutral_nonadjacent_failures"] == 0
        and all(row["gate"] == "PASS" for row in collision["joint_gates"])
        and collision["coordinated_motion_failure_pose_count"] == 0
    )
    check("mujoco_collision", collision_ok, {"engine": collision["mujoco_version"], "joint_gates": len(collision["joint_gates"]), "coordinated_samples": collision["coordinated_motion_sample_count"]})

    solidworks = _json(root, SOLIDWORKS_REL)
    solidworks_ok = (
        solidworks["overall"] == "PASS"
        and solidworks["component_count"] == 51
        and solidworks["separate_blue_source_servo_component_count"] == 16
        and solidworks["old_claw_component_count"] == 0
        and solidworks["external_neck_component_count"] == 0
    )
    check("solidworks_native_assembly", solidworks_ok, {key: solidworks[key] for key in ("solidworks_revision", "component_count", "separate_blue_source_servo_component_count", "old_claw_component_count", "external_neck_component_count")})

    portable_root = root / PORTABLE_SW_ROOT_REL
    portable_parts = [path for path in portable_root.glob("*.SLDPRT") if not path.name.startswith("~$")]
    portable_assemblies = [path for path in portable_root.glob("*.SLDASM") if not path.name.startswith("~$")]
    portable_servos = [path for path in portable_parts if len(path.name) > 3 and path.name[0] == "S" and path.name[1:3].isdigit()]
    oversized = [path.name for path in portable_root.glob("*") if path.is_file() and path.stat().st_size >= 100 * 1024 * 1024]
    portable_ok = (
        len(portable_parts) == 51
        and len(portable_assemblies) == 2
        and len(portable_servos) == 16
        and not oversized
        and len({path.name.casefold() for path in portable_parts + portable_assemblies}) == 53
    )
    portable_open = _json(root, PORTABLE_SOLIDWORKS_REL)
    portable_ok = portable_ok and portable_open["overall"] == "PASS" and portable_open["portable_resolved_component_count"] == 51
    check("solidworks_portable_pack", portable_ok, {"parts": len(portable_parts), "assemblies": len(portable_assemblies), "servo_parts": len(portable_servos), "oversized_github_files": oversized, "reopened_resolved_components": portable_open["portable_resolved_component_count"]})

    part_root = root / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
    step_count = len(list(part_root.glob("*.step")))
    stl_count = len(list(part_root.glob("*.stl")))
    check("cad_part_count", step_count == 17 and stl_count == 17, {"step": step_count, "stl": stl_count})

    mass = _json(root, MASS_REL)
    check("printed_mass_inertia_gate", mass["inertia_gate"] == "PASS" and len(mass["parts"]) == 9, {"mass_kg": mass["nominal_printed_mass_kg"], "parts": len(mass["parts"])})

    handoff = _json(root, HANDOFF_REL)
    handoff_ok = (
        handoff["canonical_urdf"] == URDF_REL.as_posix()
        and handoff["robot"]["actuated_dof"] == 16
        and abs(float(handoff["robot"]["total_mass_kg"]) - total_mass) <= 1.0e-9
        and len(handoff["joints"]) == 16
        and len(handoff["electronics_and_sensors"]) == 6
        and len(handoff["optical_frames"]) == 2
        and len(handoff["sole_contact_frames"]) == 4
    )
    check("rl_handoff", handoff_ok, {"mass_kg": handoff["robot"]["total_mass_kg"], "electronics": list(handoff["electronics_and_sensors"]), "status": handoff["training_readiness"]})

    gif_paths = [root / vtk_gif_relative, root / SOLIDWORKS_GIF_REL]
    gif_sizes = {path.relative_to(root).as_posix(): path.stat().st_size if path.is_file() else 0 for path in gif_paths}
    check("motion_gifs", all(size > 100_000 for size in gif_sizes.values()), gif_sizes)

    manifest_path = root / "RELEASE_MANIFEST.json"
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        manifest_failures: list[dict[str, object]] = []
        for row in manifest.get("files", []):
            path = root / str(row["path"])
            if not path.is_file():
                manifest_failures.append({"path": row["path"], "failure": "missing"})
                continue
            if path.stat().st_size != int(row["bytes"]):
                manifest_failures.append({"path": row["path"], "failure": "size"})
                continue
            if _sha256(path) != row["sha256"]:
                manifest_failures.append({"path": row["path"], "failure": "sha256"})
        manifest_ok = (
            manifest.get("schema") == "zeroth01.physical_mount_v2_minimal.release_manifest.v1"
            and manifest.get("canonical_urdf") == URDF_REL.as_posix()
            and int(manifest.get("file_count", -1)) == len(manifest.get("files", []))
            and not manifest_failures
        )
        check("release_manifest_integrity", manifest_ok, {"file_count": manifest.get("file_count"), "failures": manifest_failures[:20]})

    failures = [name for name, row in checks.items() if row["gate"] != "PASS"]
    report = {
        "schema": "zeroth01.physical_mount_v2_minimal.release_gate.v1",
        "root": ".",
        "canonical_urdf": URDF_REL.as_posix(),
        "checks": checks,
        "failures": failures,
        "overall": "PASS" if not failures else "FAIL",
        "claim_boundary": "Simulation and CAD release gate only; purchased STS3250 first article, harness routing and as-built mass/inertia identification remain required before full print/sim-to-real signoff.",
    }
    output = root / OUTPUT_REL
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    args = parser.parse_args()
    report = validate(args.root)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
