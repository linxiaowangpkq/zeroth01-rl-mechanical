"""Evaluate nominal STS3250 gravity torque on the shipped v4 MJCF.

This is deliberately a quasi-static model gate. It cannot replace rollout
peak/RMS torque, current, temperature or a purchased-servo bench test.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import mujoco


ROOT = Path(__file__).resolve().parents[2]
MJCF = ROOT / "generated" / "mujoco" / "physical_mount_v4_original_minimal" / "zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml"
REPORT = ROOT / "reports" / "v4_original_minimal" / "sts3250_quasistatic_torque_gate.json"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def main() -> int:
    source = load(Path(__file__).with_name("build_v4_urdf.py"), "zeroth01_v4_torque_source")
    motion = load(Path(__file__).with_name("render_v4_motion.py"), "zeroth01_v4_torque_motion")
    model = mujoco.MjModel.from_xml_path(str(MJCF))
    data = mujoco.MjData(model)
    key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "official_standing")
    foot_ids = tuple(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in ("FOOT_collision", "FOOT_2_collision")
    )
    joint_ids = [
        joint_id
        for joint_id in range(model.njnt)
        if model.jnt_type[joint_id] == mujoco.mjtJoint.mjJNT_HINGE
    ]
    peaks = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id): {
            "abs_torque_nm": 0.0,
            "frame": 0,
            "position_rad": 0.0,
        }
        for joint_id in joint_ids
    }

    frame_count = 64
    for frame in range(frame_count):
        phase = 2.0 * math.pi * frame / frame_count
        mujoco.mj_resetDataKeyframe(model, data, key_id)
        values = motion.v3.set_pose(model, data, phase)
        motion.v3.reground(model, data, foot_ids)
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        mujoco.mj_inverse(model, data)
        for joint_id in joint_ids:
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            torque = abs(float(data.qfrc_inverse[model.jnt_dofadr[joint_id]]))
            if torque > peaks[name]["abs_torque_nm"]:
                peaks[name] = {
                    "abs_torque_nm": torque,
                    "frame": frame,
                    "position_rad": float(values[name]),
                }

    continuous = source.v3.CONTINUOUS_EFFORT_NM
    rated = source.v3.RATED_EFFORT_NM
    rows = []
    for name, peak in peaks.items():
        torque = peak["abs_torque_nm"]
        rows.append(
            {
                "joint": name,
                **peak,
                "continuous_fraction": torque / continuous,
                "rated_fraction": torque / rated,
                "continuous_gate": "PASS" if torque <= continuous else "HOLD",
                "rated_gate": "PASS" if torque <= rated else "FAIL",
            }
        )
    rows.sort(key=lambda row: row["abs_torque_nm"], reverse=True)
    peak_torque = rows[0]["abs_torque_nm"]
    if peak_torque <= continuous:
        gate = "PASS_NOMINAL_QUASISTATIC_CONTINUOUS"
    elif peak_torque <= rated:
        gate = "HOLD_RATED_ONLY_DYNAMIC_POLICY_TRACE_REQUIRED"
    else:
        gate = "FAIL_NOMINAL_QUASISTATIC_EXCEEDS_RATED"

    payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.sts3250_quasistatic_torque.v1",
        "source_mjcf": MJCF.relative_to(ROOT).as_posix(),
        "method": "MuJoCo mj_inverse at zero qvel/qacc over the same 64 coordinated poses; nominal gravity torque only",
        "actuator_model": "FEETECH STS3250-C001",
        "supply_voltage_v": 12.0,
        "continuous_design_torque_nm": continuous,
        "rated_torque_nm": rated,
        "official_stall_torque_nm": 50.0 * 9.80665 / 100.0,
        "frame_count": frame_count,
        "rows": rows,
        "peak_joint": rows[0]["joint"],
        "peak_quasistatic_torque_nm": peak_torque,
        "gate": gate,
        "required_next_gate": "RL rollout peak/RMS torque and thermal-current trace, then instrumented first-article bench",
        "truth_boundary": "Passing this report does not prove walking, running, impact capacity or servo thermal margin.",
        "official_source": "https://www.feetechrc.com/en/562636.html",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({key: payload[key] for key in ("peak_joint", "peak_quasistatic_torque_nm", "gate")}, indent=2))
    return 1 if gate.startswith("FAIL") else 0


if __name__ == "__main__":
    raise SystemExit(main())
