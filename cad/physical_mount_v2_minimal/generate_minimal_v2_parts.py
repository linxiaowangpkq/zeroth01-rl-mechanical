"""Export every Physical Mount v2-minimal part as STEP and STL."""

from __future__ import annotations

import json
from pathlib import Path

from build123d import export_step, export_stl, import_step

import minimal_v2_common as common


ROOT = Path(__file__).resolve().parents[2]
PART_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
REPORT_ROOT = ROOT / "reports" / "physical_mount_v2_minimal"
MANIFEST = REPORT_ROOT / "component_manifest.json"


def main() -> int:
    PART_ROOT.mkdir(parents=True, exist_ok=True)
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    records = common.part_records()
    for index, (key, record) in enumerate(records.items(), start=1):
        step_path = PART_ROOT / f"{key}.step"
        stl_path = PART_ROOT / f"{key}.stl"
        shape = record.shape
        if record.printable and (not shape.is_valid or len(shape.solids()) != 1):
            raise ValueError(
                f"{key}: printable geometry must be one valid solid; "
                f"valid={shape.is_valid} solids={len(shape.solids())}"
            )
        if not export_step(shape, step_path):
            raise RuntimeError(f"STEP export failed: {step_path}")
        if not export_stl(shape, stl_path, tolerance=0.05, angular_tolerance=0.08):
            raise RuntimeError(f"STL export failed: {stl_path}")
        if record.printable:
            reloaded = import_step(step_path)
            if len(reloaded.solids()) != 1:
                raise RuntimeError(f"{key}: STEP round-trip has {len(reloaded.solids())} solids")
        print(f"[{index:02d}/{len(records):02d}] {key}", flush=True)
    MANIFEST.write_text(
        json.dumps(common.manifest_payload(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(MANIFEST)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
