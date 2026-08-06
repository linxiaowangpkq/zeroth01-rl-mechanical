"""Build and verify the publish repository's SHA-256 release manifest."""

from __future__ import annotations

import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
MANIFEST = ROOT / "RELEASE_MANIFEST.json"
TEXT_SUFFIXES = {
    ".csv",
    ".json",
    ".log",
    ".md",
    ".py",
    ".txt",
    ".urdf",
    ".xml",
    ".yaml",
    ".yml",
}
TEXT_FILENAMES = {".gitattributes", ".gitignore"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def release_files() -> list[Path]:
    return sorted(
        (
            path
            for path in ROOT.rglob("*")
            if path.is_file()
            and ".git" not in path.relative_to(ROOT).parts
            and path != MANIFEST
        ),
        key=lambda path: path.relative_to(ROOT).as_posix(),
    )


def normalize_text_files(files: list[Path]) -> None:
    """Match the repository's `eol=lf` checkout policy before hashing."""
    for path in files:
        if path.suffix.lower() not in TEXT_SUFFIXES and path.name not in TEXT_FILENAMES:
            continue
        content = path.read_bytes()
        normalized = content.replace(b"\r\n", b"\n")
        if normalized != content:
            path.write_bytes(normalized)


def build() -> dict[str, object]:
    files = release_files()
    normalize_text_files(files)
    rows = [
        {
            "path": path.relative_to(ROOT).as_posix(),
            "bytes": path.stat().st_size,
            "sha256": sha256(path),
        }
        for path in files
    ]
    return {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.release_manifest.v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "canonical_solidworks": (
            "generated/solidworks/physical_mount_v3_rl_fixed/portable_flat/"
            "OPEN_FIRST_ZEROTH01_V3_RL_FIXED_CONNECTED_WHITE_18_BLUE_STS3250.SLDASM"
        ),
        "canonical_urdf": (
            "generated/urdf/physical_mount_v3_rl_fixed/"
            "zeroth01_physical_mount_v3_rl_fixed_18dof.urdf"
        ),
        "canonical_mjcf": (
            "generated/mujoco/physical_mount_v3_rl_fixed/"
            "zeroth01_physical_mount_v3_rl_fixed_18dof_mjx.xml"
        ),
        "file_count": len(rows),
        "total_bytes": sum(row["bytes"] for row in rows),
        "files": rows,
    }


def verify(manifest: dict[str, object]) -> None:
    rows = manifest["files"]
    assert isinstance(rows, list)
    for row in rows:
        assert isinstance(row, dict)
        path = ROOT / str(row["path"])
        if not path.is_file():
            raise FileNotFoundError(path)
        if path.stat().st_size != row["bytes"]:
            raise RuntimeError(f"Size mismatch: {path}")
        if sha256(path) != row["sha256"]:
            raise RuntimeError(f"SHA-256 mismatch: {path}")


if __name__ == "__main__":
    payload = build()
    with MANIFEST.open("w", encoding="utf-8", newline="\n") as stream:
        stream.write(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")
    verify(payload)
    print(
        "RELEASE_MANIFEST_PASS "
        f"files={payload['file_count']} bytes={payload['total_bytes']}"
    )
