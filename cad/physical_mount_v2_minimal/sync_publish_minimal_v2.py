"""Replace the GitHub publish worktree with the validated v2-minimal release."""

from __future__ import annotations

import hashlib
import importlib.util
import json
import shutil
import subprocess
import sys
from pathlib import Path


SOURCE_ROOT = Path(__file__).resolve().parents[2]
V1_SYNC_PATH = SOURCE_ROOT / "cad" / "physical_mount_v1" / "sync_publish_repo.py"
V2_VALIDATOR = SOURCE_ROOT / "scripts" / "validate_minimal_v2_release.py"


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v1 = _load(V1_SYNC_PATH, "zeroth01_v1_publish_sync")
validator = _load(V2_VALIDATOR, "zeroth01_v2_minimal_release_validator")
PUBLISH_ROOT: Path = v1.PUBLISH_ROOT


def copy_v2_release() -> None:
    root_files = [
        (SOURCE_ROOT / "release_assets" / "README_V2.md", PUBLISH_ROOT / "README.md"),
        (SOURCE_ROOT / "PHYSICAL_MOUNT_V2_MINIMAL_README_zh.md", PUBLISH_ROOT / "README_zh.md"),
        (SOURCE_ROOT / "PHYSICAL_MOUNT_V2_MINIMAL_README_zh.md", PUBLISH_ROOT / "PHYSICAL_MOUNT_V2_MINIMAL_README_zh.md"),
        (SOURCE_ROOT / "ASSEMBLY_PHYSICAL_MOUNT_V2_MINIMAL_zh.md", PUBLISH_ROOT / "ASSEMBLY_GUIDE_zh.md"),
        (SOURCE_ROOT / "one-seq.md", PUBLISH_ROOT / "one-seq.md"),
        (SOURCE_ROOT / "RL_PROMPT.txt", PUBLISH_ROOT / "RL_PROMPT.txt"),
        (V2_VALIDATOR, PUBLISH_ROOT / "scripts" / "validate_minimal_v2_release.py"),
    ]
    for source, target in root_files:
        v1.copy_file(source, target)

    v1.copy_file(
        SOURCE_ROOT / "config" / "round_v1_electronics_layout_source.json",
        PUBLISH_ROOT / "config" / "round_v1_electronics_layout_source.json",
    )

    v1.copy_tree(
        SOURCE_ROOT / "cad" / "physical_mount_v2_minimal",
        PUBLISH_ROOT / "cad" / "physical_mount_v2_minimal",
        include=lambda path, relative: (
            path.suffix.lower() in {".py", ".cs", ".csproj"}
            and "__pycache__" not in relative.parts
            and "bin" not in relative.parts
            and "obj" not in relative.parts
        ),
    )
    v1.copy_tree(
        SOURCE_ROOT / "generated" / "urdf" / "physical_mount_v2_minimal",
        PUBLISH_ROOT / "generated" / "urdf" / "physical_mount_v2_minimal",
    )
    for name in (
        "physical_mount_v2_minimal_mass_properties.json",
        "physical_mount_v2_minimal_rl_handoff.json",
    ):
        v1.copy_file(
            SOURCE_ROOT / "generated" / "config" / name,
            PUBLISH_ROOT / "generated" / "config" / name,
        )
    v1.copy_tree(
        SOURCE_ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts",
        PUBLISH_ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts",
        include=lambda path, _relative: path.suffix.lower() in {".step", ".stp", ".stl"},
    )
    v1.copy_tree(
        SOURCE_ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements",
        PUBLISH_ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements",
        include=lambda path, _relative: path.suffix.lower() in {".step", ".stp", ".stl"},
    )
    v1.copy_tree(
        SOURCE_ROOT / "generated" / "solidworks" / "physical_mount_v2_minimal" / "portable_flat",
        PUBLISH_ROOT / "generated" / "solidworks" / "physical_mount_v2_minimal" / "portable_flat",
        include=lambda path, _relative: (
            path.suffix.lower() in {".sldprt", ".sldasm"}
            and not path.name.startswith("~$")
            and ".invalid_" not in path.name
        ),
    )
    v1.copy_tree(
        SOURCE_ROOT / "reports" / "physical_mount_v2_minimal",
        PUBLISH_ROOT / "reports" / "physical_mount_v2_minimal",
        include=lambda path, _relative: path.suffix.lower() in {".json", ".csv"},
    )

    v1.copy_tree(
        SOURCE_ROOT / "snapshots" / "cad" / "physical_mount_v2_minimal",
        PUBLISH_ROOT / "snapshots" / "physical_mount_v2_minimal",
        include=lambda path, relative: len(relative.parts) == 1 and path.suffix.lower() in {".png", ".gif"},
    )
    v1.copy_tree(
        SOURCE_ROOT / "snapshots" / "solidworks" / "physical_mount_v2_minimal",
        PUBLISH_ROOT / "snapshots" / "solidworks" / "physical_mount_v2_minimal",
        include=lambda path, relative: len(relative.parts) == 1 and path.suffix.lower() in {".png", ".gif"},
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def write_manifest() -> None:
    rows: list[dict[str, object]] = []
    for path in sorted(PUBLISH_ROOT.rglob("*")):
        if not path.is_file() or ".git" in path.parts:
            continue
        relative = path.relative_to(PUBLISH_ROOT).as_posix()
        if relative in {
            "RELEASE_MANIFEST.json",
            "reports/physical_mount_v2_minimal/release_gate.json",
        }:
            continue
        rows.append({"path": relative, "bytes": path.stat().st_size, "sha256": _sha256(path)})
    payload = {
        "schema": "zeroth01.physical_mount_v2_minimal.release_manifest.v1",
        "canonical_urdf": "generated/urdf/physical_mount_v2_minimal/zeroth01_physical_mount_v2_minimal.urdf",
        "file_count": len(rows),
        "total_bytes": sum(int(row["bytes"]) for row in rows),
        "files": rows,
    }
    (PUBLISH_ROOT / "RELEASE_MANIFEST.json").write_bytes(
        (json.dumps(payload, indent=2, ensure_ascii=False) + "\n").encode("utf-8")
    )


def main() -> int:
    v1.validate_targets()
    source_gate = validator.validate(SOURCE_ROOT)
    if source_gate["failures"]:
        raise RuntimeError(f"v2 source release gate failed: {source_gate['failures']}")
    v1_validator = v1.load_validator()
    v1_gate = v1_validator.validate(SOURCE_ROOT)
    # The v2 release intentionally replaces the one-sentence canonical URDF
    # pointer; every mechanical v1 dependency gate must still pass.
    dependency_failures = [
        name for name in v1_gate["failures"]
        if name != "one_seq_is_one_sentence_line"
    ]
    if dependency_failures:
        raise RuntimeError(f"v1 dependency release gate failed: {dependency_failures}")

    preserved = v1.preserve_licenses()
    print(f"validated publish target: {PUBLISH_ROOT}")
    v1.clean_publish_root()
    for relative, payload in preserved.items():
        target = PUBLISH_ROOT / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(payload)
    v1.copy_release()
    legacy_solidworks = PUBLISH_ROOT / "generated" / "solidworks" / "physical_mount_v1"
    expected_legacy = (PUBLISH_ROOT / "generated" / "solidworks" / "physical_mount_v1").resolve()
    if legacy_solidworks.is_dir():
        if legacy_solidworks.resolve() != expected_legacy or PUBLISH_ROOT.resolve() not in legacy_solidworks.resolve().parents:
            raise RuntimeError(f"refusing unexpected legacy SolidWorks removal: {legacy_solidworks}")
        shutil.rmtree(legacy_solidworks)
    copy_v2_release()
    print(f"normalized release text files to LF: {v1.normalize_release_text_files()}")

    subprocess.run(
        [sys.executable, str(PUBLISH_ROOT / "scripts" / "validate_minimal_v2_release.py"), "--root", str(PUBLISH_ROOT)],
        cwd=PUBLISH_ROOT,
        check=True,
        timeout=120.0,
    )
    print(f"normalized validator outputs to LF: {v1.normalize_release_text_files()}")
    write_manifest()
    # The final validation adds the manifest-integrity result to release_gate.
    # release_gate is deliberately excluded from the manifest to avoid a
    # self-referential hash while every actual release input remains covered.
    subprocess.run(
        [sys.executable, str(PUBLISH_ROOT / "scripts" / "validate_minimal_v2_release.py"), "--root", str(PUBLISH_ROOT)],
        cwd=PUBLISH_ROOT,
        check=True,
        timeout=120.0,
        stdout=subprocess.DEVNULL,
    )
    v1.normalize_release_text_files()
    print(f"v2-minimal release sync complete: {PUBLISH_ROOT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
