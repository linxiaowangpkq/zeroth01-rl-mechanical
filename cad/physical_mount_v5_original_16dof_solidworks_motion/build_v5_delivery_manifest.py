"""Create a checksum ledger for the canonical v5 CAD/RL handoff."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
CAD_ROOT = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion"
URDF_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v5_original_16dof_solidworks_motion"
MJCF_ROOT = ROOT / "generated" / "mujoco" / "physical_mount_v5_original_16dof_solidworks_motion"
REPORT_ROOT = ROOT / "reports" / "v5_original_16dof_solidworks_motion"
OUTPUT = ROOT / "generated" / "config" / "physical_mount_v5_original_16dof_solidworks_motion_delivery_manifest.json"


TEXT_SUFFIXES = {".csv", ".json", ".urdf", ".xml"}


def canonical_bytes(path: Path) -> tuple[bytes, str]:
    """Return clone-stable bytes and document the checksum convention.

    Git enforces LF for text in this repository, while a Windows worktree can
    still contain CRLF before the file is staged.  Normalizing newline bytes
    (without decoding or stripping a possible BOM) keeps this ledger stable
    across Windows/Linux clones and preserves every other byte.
    """

    data = path.read_bytes()
    if path.suffix.lower() in TEXT_SUFFIXES:
        return data.replace(b"\r\n", b"\n").replace(b"\r", b"\n"), "sha256_lf_normalized_bytes"
    return data, "sha256_exact_bytes"


def checksum_row(path: Path) -> dict[str, object]:
    data, hash_mode = canonical_bytes(path)
    return {
        "path": path.relative_to(ROOT).as_posix(),
        "bytes": len(data),
        "sha256": hashlib.sha256(data).hexdigest(),
        "hash_mode": hash_mode,
    }


def main() -> int:
    files = [
        CAD_ROOT / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json",
        CAD_ROOT / "ZEROTH01_V5_ORIGINAL_16DOF_NEUTRAL_REVIEW.glb",
        ROOT / "generated" / "config" / "physical_mount_v5_original_16dof_solidworks_motion_actuator_layout.json",
        ROOT / "generated" / "config" / "physical_mount_v5_original_16dof_solidworks_motion_hardware_calibration.csv",
        ROOT / "generated" / "config" / "physical_mount_v5_original_16dof_solidworks_motion_rl_handoff.json",
        REPORT_ROOT / "release_gate.json",
        REPORT_ROOT / "offline_brep_component_interference.json",
        REPORT_ROOT / "motion_link_part_gate.json",
        REPORT_ROOT / "hip_yaw_dynamic_mount_screen.json",
        REPORT_ROOT / "hip_yaw_uhip_mount_clearance.json",
        REPORT_ROOT / "urdf_mass_inertia_gate.json",
        REPORT_ROOT / "mjcf_compile_gate.json",
    ]
    files.extend(sorted((CAD_ROOT / "motion_link_parts").glob("*.step")))
    files.extend(sorted(URDF_ROOT.rglob("*")))
    files.extend(sorted(MJCF_ROOT.rglob("*")))
    files = sorted({path.resolve() for path in files if path.is_file()})
    rows = [checksum_row(path) for path in files]
    release = json.loads((REPORT_ROOT / "release_gate.json").read_text(encoding="utf-8"))
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.delivery_manifest.v2",
        "checksum_convention": {
            "text": "SHA-256 over raw bytes after CRLF/CR newline normalization to LF; BOM is preserved",
            "binary": "SHA-256 over exact bytes",
        },
        "file_count": len(rows),
        "total_bytes": sum(row["bytes"] for row in rows),
        "digital_rl_baseline": release["digital_rl_baseline"],
        "native_solidworks_motion": release["native_solidworks_motion"],
        "physical_first_article": release["physical_first_article"],
        "files": rows,
        "overall": "PASS" if release["digital_rl_baseline"] == "PASS" and rows else "HOLD",
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: payload[key] for key in ("file_count", "total_bytes", "digital_rl_baseline", "native_solidworks_motion", "overall")}, indent=2))
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
