from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "RELEASE_MANIFEST.json"
EXCLUDED_PARTS = {".git", "__pycache__"}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> int:
    files = []
    for path in sorted(ROOT.rglob("*")):
        if not path.is_file() or path == OUTPUT:
            continue
        relative = path.relative_to(ROOT)
        if any(part in EXCLUDED_PARTS for part in relative.parts):
            continue
        files.append(
            {
                "path": relative.as_posix(),
                "bytes": path.stat().st_size,
                "sha256": sha256(path),
            }
        )
    payload = {
        "schema": "zeroth01.rl_mechanical.release_manifest.v1",
        "canonical_urdf": "generated/urdf/zeroth01_rl_round_v1.urdf",
        "canonical_mjcf": "generated/mujoco/zeroth01_rl_round_v1.xml",
        "file_count_excluding_manifest": len(files),
        "total_bytes_excluding_manifest": sum(item["bytes"] for item in files),
        "files": files,
    }
    OUTPUT.write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(
        f"MANIFEST={OUTPUT} FILES={len(files)} "
        f"BYTES={payload['total_bytes_excluding_manifest']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
