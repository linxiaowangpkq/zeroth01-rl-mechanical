from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import mujoco


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
DEFAULT_MODEL = ROOT / "generated" / "mujoco" / "zeroth01_rl_round_v1.xml"
EXPECTED_MASS_KG = 4.151924609464


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compile and step the canonical Zeroth-01 round-v1 MJCF."
    )
    parser.add_argument("--model", type=Path, default=DEFAULT_MODEL)
    parser.add_argument("--steps", type=int, default=1000)
    args = parser.parse_args()

    model_path = args.model.resolve()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)
    for _ in range(args.steps):
        mujoco.mj_step(model, data)

    total_mass = float(model.body_mass.sum())
    finite_state = all(math.isfinite(float(value)) for value in data.qpos) and all(
        math.isfinite(float(value)) for value in data.qvel
    )
    checks = {
        "actuator_count_16": model.nu == 16,
        "finite_state": finite_state,
        "mass_matches_manifest": abs(total_mass - EXPECTED_MASS_KG) < 1e-9,
    }
    payload = {
        "schema": "zeroth01.round_v1.mujoco_smoke.v1",
        "model": str(model_path),
        "steps": args.steps,
        "nbody": model.nbody,
        "njnt": model.njnt,
        "nq": model.nq,
        "nv": model.nv,
        "nu": model.nu,
        "nsensor": model.nsensor,
        "total_mass_kg": total_mass,
        "checks": checks,
        "overall": "PASS" if all(checks.values()) else "FAIL",
    }
    print(json.dumps(payload, indent=2))
    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
