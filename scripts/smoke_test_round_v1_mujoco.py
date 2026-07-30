from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
DEFAULT_MODEL = ROOT / "generated" / "mujoco" / "zeroth01_rl_round_v1.xml"
DEFAULT_URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_round_v1.urdf"
DEFAULT_REPORT = ROOT / "reports" / "mujoco_round_v1_smoke_gate.json"


def portable_path(path: Path) -> str:
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return str(path)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compile and step the canonical Zeroth-01 round-v1 MJCF."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--steps", type=int, default=1000)
    parser.add_argument("--report", type=Path, default=DEFAULT_REPORT)
    args = parser.parse_args()

    model_path = args.model.resolve()
    urdf_path = args.urdf.resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    for _ in range(args.steps):
        mujoco.mj_step(model, data)

    total_mass = float(model.body_mass.sum())
    urdf_root = ET.parse(urdf_path).getroot()
    expected_mass = sum(
        float(mass.get("value", "0"))
        for mass in urdf_root.findall("./link/inertial/mass")
    )
    expected_body_count = len(urdf_root.findall("link"))
    finite_state = all(math.isfinite(float(value)) for value in data.qpos) and all(
        math.isfinite(float(value)) for value in data.qvel
    )
    checks = {
        "actuator_count_16": model.nu == 16,
        "body_count_matches_urdf_tree": model.nbody == expected_body_count,
        "imu_four_pressure_and_tof_sensors": model.nsensor == 8,
        "head_camera_present": model.ncam == 1,
        "finite_state": finite_state,
        "mass_matches_urdf": abs(total_mass - expected_mass) < 1e-9,
    }
    payload = {
        "schema": "zeroth01.round_v1.mujoco_smoke.v1",
        "model": portable_path(model_path),
        "steps": args.steps,
        "nbody": model.nbody,
        "njnt": model.njnt,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "nsensor": model.nsensor,
        "total_mass_kg": total_mass,
        "expected_urdf_mass_kg": expected_mass,
        "checks": checks,
        "overall": "PASS" if all(checks.values()) else "FAIL",
    }
    report_path = args.report.resolve()
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(
        json.dumps(payload, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(payload, indent=2))
    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
