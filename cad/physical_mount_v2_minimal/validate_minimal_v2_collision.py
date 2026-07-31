"""Run the proven v1 MuJoCo sweep against the minimal v2 URDF."""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "cad" / "physical_mount_v1" / "validate_physical_mount_v1.py"


def _load():
    spec = importlib.util.spec_from_file_location(
        "physical_mount_v2_minimal_collision_base",
        SOURCE,
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load {SOURCE}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.URDF_PATH = (
        ROOT
        / "generated"
        / "urdf"
        / "physical_mount_v2_minimal"
        / "zeroth01_physical_mount_v2_minimal.urdf"
    )
    module.REPORT_ROOT = ROOT / "reports" / "physical_mount_v2_minimal"
    module.REPORT_PATH = module.REPORT_ROOT / "dynamic_collision_gate.json"
    module.CONTACT_PATH = module.REPORT_ROOT / "dynamic_collision_contacts.csv"
    return module


if __name__ == "__main__":
    raise SystemExit(_load().main())
