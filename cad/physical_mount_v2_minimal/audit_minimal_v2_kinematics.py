"""Reuse the v1 actuator-axis audit against the minimal v2 URDF tree."""

from __future__ import annotations

import importlib.util
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "cad" / "physical_mount_v1" / "audit_physical_mount_kinematics.py"


def _load():
    spec = importlib.util.spec_from_file_location(
        "physical_mount_v2_minimal_kinematic_base",
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
    module.JSON_REPORT = module.REPORT_ROOT / "kinematic_mount_audit.json"
    module.CSV_REPORT = module.REPORT_ROOT / "kinematic_mount_audit.csv"
    return module


if __name__ == "__main__":
    raise SystemExit(_load().main())
