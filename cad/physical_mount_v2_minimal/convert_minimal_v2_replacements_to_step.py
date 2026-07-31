"""Convert the two metre-authored trimmed forearm meshes to AP214 STEP."""

from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "cad" / "physical_mount_v1" / "convert_split_stl_to_step.py"
REPLACEMENT_ROOT = (
    ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements"
)
REPORT = (
    ROOT
    / "reports"
    / "physical_mount_v2_minimal"
    / "forearm_replacement_step_gate.json"
)


def _load():
    spec = importlib.util.spec_from_file_location("minimal_v2_stl_step_base", SOURCE)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {SOURCE}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    base = _load()
    sources = sorted(REPLACEMENT_ROOT.glob("*_WRIST_TRIMMED.stl"))
    if len(sources) != 2:
        raise RuntimeError(f"expected two trimmed forearms, got {len(sources)}")
    rows = []
    for source in sources:
        target = source.with_suffix(".step")
        print(f"STEP conversion: {source.name}", flush=True)
        rows.append(base.convert(source, target, force=bool(args.force)))
    payload = {
        "schema": "zeroth01.physical_mount_v2_minimal.forearm_step_gate.v1",
        "source_units": "metre",
        "step_units": "millimetre",
        "rows": rows,
        "overall": "PASS" if all(row["status"] == "PASS" for row in rows) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(REPORT)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
