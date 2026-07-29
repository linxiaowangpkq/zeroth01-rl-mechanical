from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
AUDITED_GENERATOR = THIS_FILE.with_name("gen_zeroth01_rl_audited.py")
GLOBAL_BOX_REPORT = ROOT / "reports" / "global_collision_box_search.json"
ROBOT_NAME = "zeroth01_rl_ready_16dof"

# Values used by the official stompymicro MuJoCo model. They are simulation
# parameters, not identified physical constants. Keep them explicit so domain
# randomization and future system-identification updates are auditable.
OFFICIAL_SIM_EFFORT_LIMIT_NM = 2.0
OFFICIAL_SIM_VELOCITY_LIMIT_RAD_S = 5.0
OFFICIAL_SIM_JOINT_DAMPING_NM_S_RAD = 0.53
OFFICIAL_SIM_JOINT_FRICTION_NM = 0.001


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import generator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def gen_urdf() -> ET.Element:
    if not GLOBAL_BOX_REPORT.is_file():
        raise FileNotFoundError(
            "run search_global_collision_safe_box.py before generating the "
            f"RL-ready URDF: {GLOBAL_BOX_REPORT}"
        )
    audited = load_module(AUDITED_GENERATOR, "zeroth01_audited_generator")
    root = audited.gen_urdf()
    root.set("name", ROBOT_NAME)

    search = json.loads(GLOBAL_BOX_REPORT.read_text(encoding="utf-8"))
    selected = search.get("selected") or {}
    if selected.get("scale") is None:
        raise ValueError("global collision-box search has no passing selection")
    bounds = selected.get("bounds") or {}
    if len(bounds) != 16:
        raise ValueError(f"expected 16 guarded bounds, got {len(bounds)}")

    moving_joint_count = 0
    for joint in root.findall("joint"):
        if joint.get("type") not in {"revolute", "continuous"}:
            continue
        moving_joint_count += 1
        name = joint.get("name", "")
        if name not in bounds:
            raise ValueError(f"missing global guarded bound for {name}")
        limit = joint.find("limit")
        if limit is None:
            raise ValueError(f"moving joint lacks limit: {name}")
        limit.set("lower", f"{float(bounds[name]['lower_rad']):.9f}")
        limit.set("upper", f"{float(bounds[name]['upper_rad']):.9f}")
        limit.set("effort", f"{OFFICIAL_SIM_EFFORT_LIMIT_NM:.6f}")
        limit.set("velocity", f"{OFFICIAL_SIM_VELOCITY_LIMIT_RAD_S:.6f}")

        dynamics = joint.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(joint, "dynamics")
        dynamics.set(
            "damping", f"{OFFICIAL_SIM_JOINT_DAMPING_NM_S_RAD:.9f}"
        )
        dynamics.set("friction", f"{OFFICIAL_SIM_JOINT_FRICTION_NM:.9f}")

    if moving_joint_count != 16:
        raise ValueError(f"expected 16 moving joints, got {moving_joint_count}")

    root.insert(
        2,
        ET.Comment(
            "RL-ready startup envelope: limits use the largest tested uniform "
            "box (scale 0.3) that retains neutral and the official standing "
            "pose while passing 20,000 deterministic random poses and all "
            "65,536 box corners against non-allowed mesh collision pairs. "
            "This is sampled evidence, not a continuous mathematical proof."
        ),
    )
    root.insert(
        3,
        ET.Comment(
            "Actuator overlay: effort=2 N.m, velocity=5 rad/s, damping=0.53 "
            "N.m.s/rad and Coulomb friction=0.001 N.m follow the official "
            "stompymicro simulation baseline. Manufacturer rated/peak torque, "
            "servo IDs, zero offsets and confidence labels live in the adjacent "
            "actuator metadata; do not infer them from URDF limits."
        ),
    )
    audited.load_reference_module()._validate_tree(root)
    return root


if __name__ == "__main__":
    raise SystemExit(
        "Use the URDF skill launcher so generation-time validation is applied."
    )
