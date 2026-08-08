"""Verify every canonical file in the v5 delivery checksum ledger."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "config" / "physical_mount_v5_original_16dof_solidworks_motion_delivery_manifest.json"


def canonical_bytes(path: Path, mode: str) -> bytes:
    data = path.read_bytes()
    if mode == "sha256_lf_normalized_bytes":
        return data.replace(b"\r\n", b"\n").replace(b"\r", b"\n")
    if mode == "sha256_exact_bytes":
        return data
    raise ValueError(f"unsupported hash mode: {mode}")


def main() -> int:
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    failures: list[dict[str, object]] = []
    total_bytes = 0
    for row in payload["files"]:
        path = ROOT / row["path"]
        if not path.is_file():
            failures.append({"path": row["path"], "failure": "missing"})
            continue
        data = canonical_bytes(path, row["hash_mode"])
        total_bytes += len(data)
        digest = hashlib.sha256(data).hexdigest()
        if len(data) != row["bytes"] or digest != row["sha256"]:
            failures.append(
                {
                    "path": row["path"],
                    "failure": "checksum_or_size_mismatch",
                    "expected_bytes": row["bytes"],
                    "actual_bytes": len(data),
                    "expected_sha256": row["sha256"],
                    "actual_sha256": digest,
                }
            )
    if len(payload["files"]) != payload["file_count"]:
        failures.append({"failure": "file_count_mismatch"})
    if total_bytes != payload["total_bytes"]:
        failures.append(
            {
                "failure": "total_bytes_mismatch",
                "expected": payload["total_bytes"],
                "actual": total_bytes,
            }
        )
    result = {
        "overall": "PASS" if not failures else "FAIL",
        "file_count": payload["file_count"],
        "total_bytes": total_bytes,
        "failures": failures,
    }
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0 if not failures else 2


if __name__ == "__main__":
    raise SystemExit(main())
