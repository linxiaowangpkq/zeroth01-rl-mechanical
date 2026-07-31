from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]
ROBOTO_XW_ROOT = SOURCE_ROOT.parents[1]
PUBLISH_ROOT = ROBOTO_XW_ROOT / "publish" / "zeroth01-rl-mechanical"
EXPECTED_PUBLISH = (
    Path(r"E:\Codex\Documents-Codex\roboto-lite-infra-portfolio")
    / "roboto_xw"
    / "publish"
    / "zeroth01-rl-mechanical"
)


def copy_file(source: Path, target: Path) -> None:
    if not source.is_file():
        raise FileNotFoundError(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def copy_tree(
    source: Path,
    target: Path,
    *,
    include=None,
) -> None:
    if not source.is_dir():
        raise FileNotFoundError(source)
    for path in sorted(source.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(source)
        if include is not None and not include(path, relative):
            continue
        copy_file(path, target / relative)


def load_validator():
    path = Path(__file__).with_name("validate_release_package.py")
    spec = importlib.util.spec_from_file_location(
        "physical_mount_release_validator",
        path,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def validate_targets() -> None:
    source = SOURCE_ROOT.resolve()
    publish = PUBLISH_ROOT.resolve()
    expected = EXPECTED_PUBLISH.resolve()
    if source.drive.upper() != "E:":
        raise RuntimeError(f"source must remain on E:, got {source}")
    if publish != expected:
        raise RuntimeError(
            f"refusing unexpected publish target {publish}; expected {expected}"
        )
    if not (publish / ".git").is_dir():
        raise RuntimeError(f"publish target is not a Git worktree: {publish}")


def preserve_licenses() -> dict[Path, bytes]:
    paths = [
        Path("LICENSE"),
        Path("LICENSES") / "ZEROTH_SIM_MIT.txt",
        Path("LICENSES") / "ZEROTH_BOT_MIT.txt",
        Path("LICENSES") / "KSCALE_ONSHAPE_MIT.txt",
    ]
    preserved: dict[Path, bytes] = {}
    for relative in paths:
        path = PUBLISH_ROOT / relative
        if not path.is_file():
            raise FileNotFoundError(path)
        preserved[relative] = path.read_bytes()
    return preserved


def clean_publish_root() -> None:
    for child in PUBLISH_ROOT.iterdir():
        if child.name == ".git":
            continue
        if child.is_dir() and not child.is_symlink():
            shutil.rmtree(child)
        else:
            child.unlink()


def copy_release() -> None:
    assets = SOURCE_ROOT / "release_assets"
    root_files = [
        (assets / "README.md", PUBLISH_ROOT / "README.md"),
        (
            SOURCE_ROOT / "PHYSICAL_MOUNT_V1_README_zh.md",
            PUBLISH_ROOT / "README_zh.md",
        ),
        (
            SOURCE_ROOT / "PHYSICAL_MOUNT_V1_README_zh.md",
            PUBLISH_ROOT / "PHYSICAL_MOUNT_V1_README_zh.md"
        ),
        (
            SOURCE_ROOT / "ASSEMBLY_FIRST_ARTICLE_PHYSICAL_MOUNT_V1_zh.md",
            PUBLISH_ROOT / "ASSEMBLY_GUIDE_zh.md",
        ),
        (SOURCE_ROOT / "one-seq.md", PUBLISH_ROOT / "one-seq.md"),
        (SOURCE_ROOT / "RL_PROMPT.txt", PUBLISH_ROOT / "RL_PROMPT.txt"),
        (assets / ".gitattributes", PUBLISH_ROOT / ".gitattributes"),
        (assets / ".gitignore", PUBLISH_ROOT / ".gitignore"),
        (
            assets / "requirements-validation.txt",
            PUBLISH_ROOT / "requirements-validation.txt",
        ),
        (
            assets / "requirements-generation.txt",
            PUBLISH_ROOT / "requirements-generation.txt",
        ),
        (
            assets / "THIRD_PARTY_NOTICES.md",
            PUBLISH_ROOT / "THIRD_PARTY_NOTICES.md",
        ),
        (
            assets / "validate.yml",
            PUBLISH_ROOT / ".github" / "workflows" / "validate.yml",
        ),
    ]
    for source, target in root_files:
        copy_file(source, target)

    source_lock_files = [
        "PHYSICAL_MOUNT_V1_SOURCE_LOCK.json",
    ]
    for name in source_lock_files:
        copy_file(
            SOURCE_ROOT / "source_assets" / name,
            PUBLISH_ROOT / "source_assets" / name,
        )
    for name in [
        "FEETECH_STS3250_SPEC.pdf",
        "FEETECH_STS3215_SPEC.pdf",
        "FEETECH_STS3215_DRAWING-6.png",
        "FEETECH_STS3250_DRAWING_1.png",
        "FEETECH_STS3250_DRAWING_2.png",
        "FEETECH_STS3250_DRAWING_3.png",
    ]:
        copy_file(
            SOURCE_ROOT / "source_assets" / "vendor" / name,
            PUBLISH_ROOT / "source_assets" / "vendor" / name,
        )

    copy_tree(
        SOURCE_ROOT / "upstream" / "kscale-assets" / "zbot",
        PUBLISH_ROOT / "upstream" / "kscale-assets" / "zbot",
    )

    for name in [
        "physical_mount_v1_source_regions.json",
        "physical_mount_v1_guarded_limits.json",
    ]:
        copy_file(
            SOURCE_ROOT / "config" / name,
            PUBLISH_ROOT / "config" / name,
        )

    cad_names = [
        "analyze_source_mesh.py",
        "build_physical_mount_v1.py",
        "validate_physical_mount_v1.py",
        "audit_physical_mount_kinematics.py",
        "build_sts3250_interface_gauge.py",
        "convert_split_stl_to_step.py",
        "create_solidworks_physical_mount_v1.py",
        "launch_solidworks_physical_mount_v1.py",
        "close_task_solidworks_session.py",
        "validate_release_package.py",
        "sync_publish_repo.py",
    ]
    for name in cad_names:
        copy_file(
            SOURCE_ROOT / "cad" / "physical_mount_v1" / name,
            PUBLISH_ROOT / "cad" / "physical_mount_v1" / name,
        )
    copy_file(
        SOURCE_ROOT / "scripts" / "create_solidworks_kinematic_review.py",
        PUBLISH_ROOT / "scripts" / "create_solidworks_kinematic_review.py",
    )
    copy_file(
        SOURCE_ROOT
        / "cad"
        / "physical_mount_v1"
        / "validate_release_package.py",
        PUBLISH_ROOT / "scripts" / "validate_release.py",
    )

    copy_tree(
        SOURCE_ROOT / "generated" / "urdf" / "physical_mount_v1",
        PUBLISH_ROOT / "generated" / "urdf" / "physical_mount_v1",
    )
    for name in [
        "physical_mount_v1_actuators.json",
        "physical_mount_v1_hardware_calibration_template.csv",
        "physical_mount_v1_rl_handoff.json",
    ]:
        copy_file(
            SOURCE_ROOT / "generated" / "config" / name,
            PUBLISH_ROOT / "generated" / "config" / name,
        )
    copy_tree(
        SOURCE_ROOT / "generated" / "cad" / "physical_mount_v1" / "skeleton",
        PUBLISH_ROOT / "generated" / "cad" / "physical_mount_v1" / "skeleton",
    )
    copy_tree(
        SOURCE_ROOT / "generated" / "cad" / "physical_mount_v1" / "servos",
        PUBLISH_ROOT / "generated" / "cad" / "physical_mount_v1" / "servos",
    )
    copy_tree(
        SOURCE_ROOT
        / "generated"
        / "cad"
        / "physical_mount_v1"
        / "sts3250_interface",
        PUBLISH_ROOT
        / "generated"
        / "cad"
        / "physical_mount_v1"
        / "sts3250_interface",
    )
    copy_tree(
        SOURCE_ROOT / "generated" / "print" / "physical_mount_v1",
        PUBLISH_ROOT / "generated" / "print" / "physical_mount_v1",
    )
    copy_tree(
        SOURCE_ROOT / "generated" / "solidworks" / "physical_mount_v1",
        PUBLISH_ROOT / "generated" / "solidworks" / "physical_mount_v1",
        include=lambda path, _relative: (
            path.suffix.lower() in {".sldprt", ".sldasm"}
            and not path.name.startswith("~$")
            and ".invalid_" not in path.name
        ),
    )

    report_names = [
        "source_component_gate.json",
        "servo_component_manifest.json",
        "servo_component_manifest.csv",
        "kinematic_mount_audit.json",
        "kinematic_mount_audit.csv",
        "dynamic_collision_gate.json",
        "dynamic_collision_contacts.csv",
        "solidworks_physical_mount_gate.json",
        "solidworks_physical_mount_parts.csv",
        "solidworks_physical_mount_link_subassemblies.csv",
        "solidworks_physical_mount_components.csv",
        "sts3250_interface_gauge.json",
        "sts3250_inertia_delta.json",
        "first_article_measurements.csv",
    ]
    for name in report_names:
        copy_file(
            SOURCE_ROOT / "reports" / "physical_mount_v1" / name,
            PUBLISH_ROOT / "reports" / "physical_mount_v1" / name,
        )
    copy_tree(
        SOURCE_ROOT / "reports" / "physical_mount_v1" / "source_mesh_audit",
        PUBLISH_ROOT / "reports" / "physical_mount_v1" / "source_mesh_audit",
    )

    for name in [
        "physical_mount_v1_16_blue_servos_front.png",
        "physical_mount_v1_16_blue_servos_xray.png",
        "physical_mount_v1_16dof_motion.gif",
    ]:
        copy_file(
            SOURCE_ROOT / "snapshots" / "physical_mount_v1" / name,
            PUBLISH_ROOT / "snapshots" / "physical_mount_v1" / name,
        )
    for name in [
        "solidworks_physical_mount_front.png",
        "solidworks_physical_mount_isometric.png",
        "solidworks_physical_mount_v1_16dof_motion.gif",
    ]:
        copy_file(
            SOURCE_ROOT
            / "snapshots"
            / "solidworks"
            / "physical_mount_v1"
            / name,
            PUBLISH_ROOT
            / "snapshots"
            / "solidworks"
            / "physical_mount_v1"
            / name,
        )


def normalize_release_text_files() -> int:
    """Make manifest hashes match the repository's eol=lf checkout bytes."""
    text_suffixes = {
        ".csv",
        ".json",
        ".md",
        ".mjcf",
        ".py",
        ".txt",
        ".urdf",
        ".xml",
        ".yaml",
        ".yml",
    }
    text_names = {".gitattributes", ".gitignore", "LICENSE"}
    changed = 0
    for path in sorted(PUBLISH_ROOT.rglob("*")):
        if (
            not path.is_file()
            or ".git" in path.parts
            or (
                path.suffix.lower() not in text_suffixes
                and path.name not in text_names
            )
        ):
            continue
        payload = path.read_bytes()
        normalized = payload.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
        if normalized != payload:
            path.write_bytes(normalized)
            changed += 1
    return changed


def write_manifest() -> None:
    rows = []
    for path in sorted(PUBLISH_ROOT.rglob("*")):
        if not path.is_file() or ".git" in path.parts:
            continue
        relative = path.relative_to(PUBLISH_ROOT).as_posix()
        if relative == "RELEASE_MANIFEST.json":
            continue
        rows.append(
            {
                "path": relative,
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    payload = {
        "schema": "zeroth01.physical_mount_v1.release_manifest.v1",
        "file_count": len(rows),
        "total_bytes": sum(row["bytes"] for row in rows),
        "files": rows,
    }
    (PUBLISH_ROOT / "RELEASE_MANIFEST.json").write_bytes(
        (json.dumps(payload, indent=2, ensure_ascii=False) + "\n").encode(
            "utf-8"
        )
    )


def main() -> int:
    validate_targets()
    validator = load_validator()
    source_gate = validator.validate(SOURCE_ROOT)
    if source_gate["failures"]:
        raise RuntimeError(
            f"source release gate failed: {source_gate['failures']}"
        )
    preserved = preserve_licenses()
    print(f"validated publish target: {PUBLISH_ROOT}")
    print("removing superseded round_v1/round_v3 diagnostic release files")
    clean_publish_root()
    for relative, payload in preserved.items():
        target = PUBLISH_ROOT / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
    copy_release()
    print(
        "normalized copied release text files to LF: "
        f"{normalize_release_text_files()}"
    )
    subprocess.run(
        [
            sys.executable,
            str(PUBLISH_ROOT / "scripts" / "validate_release.py"),
            "--root",
            str(PUBLISH_ROOT),
        ],
        cwd=PUBLISH_ROOT,
        check=True,
        timeout=120.0,
    )
    print(
        "normalized validator outputs to LF: "
        f"{normalize_release_text_files()}"
    )
    write_manifest()
    print(f"release sync complete: {PUBLISH_ROOT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
