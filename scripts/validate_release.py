"""Validate the canonical Zeroth-01 v3 RL/mechanical publish package."""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "generated/urdf/physical_mount_v3_rl_fixed/zeroth01_physical_mount_v3_rl_fixed_18dof.urdf"
MJCF = ROOT / "generated/mujoco/physical_mount_v3_rl_fixed/zeroth01_physical_mount_v3_rl_fixed_18dof_mjx.xml"
LAYOUT = ROOT / "generated/config/physical_mount_v3_rl_fixed_actuator_layout.json"
HANDOFF = ROOT / "generated/config/physical_mount_v3_rl_fixed_rl_handoff.json"
GATES = ROOT / "reports/physical_mount_v3_rl_fixed/release_gates.json"
SW_GATE = ROOT / "reports/physical_mount_v3_rl_fixed/solidworks_gate.json"
INTERFERENCE = ROOT / "reports/physical_mount_v3_rl_fixed/solidworks_interference_gate.json"
SW_ASSEMBLY = ROOT / "generated/solidworks/physical_mount_v3_rl_fixed/portable_flat/OPEN_FIRST_ZEROTH01_V3_RL_FIXED_CONNECTED_WHITE_18_BLUE_STS3250.SLDASM"
TARGET_MASS_KG = 2.969171828


def read_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def main() -> int:
    failures: list[str] = []
    checks: dict[str, object] = {}

    required = [URDF, MJCF, LAYOUT, HANDOFF, GATES, SW_GATE, INTERFERENCE, SW_ASSEMBLY]
    missing = [path.relative_to(ROOT).as_posix() for path in required if not path.is_file()]
    checks["required_files"] = {"gate": "PASS" if not missing else "FAIL", "missing": missing}
    if missing:
        failures.append("required_files")
    else:
        robot = ET.parse(URDF).getroot()
        joints = [joint for joint in robot.findall("joint") if joint.get("type") == "revolute"]
        masses = [float(node.get("value")) for node in robot.findall("./link/inertial/mass")]
        inertia_bad = []
        for link in robot.findall("link"):
            inertia = link.find("./inertial/inertia")
            if inertia is None:
                if link.find("visual") is not None or link.find("collision") is not None:
                    inertia_bad.append(link.get("name"))
                continue
            diagonal = [float(inertia.get(axis)) for axis in ("ixx", "iyy", "izz")]
            if any(not math.isfinite(value) or value <= 0.0 for value in diagonal):
                inertia_bad.append(link.get("name"))
        mesh_missing = []
        for mesh in robot.findall(".//mesh"):
            path = (URDF.parent / str(mesh.get("filename"))).resolve()
            if not path.is_file():
                mesh_missing.append(str(mesh.get("filename")))
        urdf_ok = len(joints) == 18 and abs(sum(masses) - TARGET_MASS_KG) <= 1e-9 and not inertia_bad and not mesh_missing
        checks["urdf"] = {
            "gate": "PASS" if urdf_ok else "FAIL",
            "revolute_joints": len(joints),
            "mass_kg": sum(masses),
            "bad_inertials": inertia_bad,
            "missing_meshes": mesh_missing,
        }
        if not urdf_ok:
            failures.append("urdf")

        layout = read_json(LAYOUT)
        actuators = list(layout.get("actuators", []))
        ids = {str(row.get("id")) for row in actuators}
        layout_ok = len(actuators) == 18 and ids == {f"S{index:02d}" for index in range(1, 19)}
        checks["actuator_layout"] = {"gate": "PASS" if layout_ok else "FAIL", "count": len(actuators)}
        if not layout_ok:
            failures.append("actuator_layout")

        gates = read_json(GATES)
        sw = read_json(SW_GATE)
        interference = read_json(INTERFERENCE)
        digital_ok = (
            gates.get("overall_rl_nominal") == "PASS"
            and sw.get("overall") == "PASS"
            and sw.get("standing_height_mm", 1e9) <= 500.0
            and sw.get("separate_blue_sts3250_count") == 18
            and interference.get("overall") == "PASS"
            and interference.get("physical_interference_count") == 0
        )
        checks["digital_release"] = {
            "gate": "PASS" if digital_ok else "FAIL",
            "height_mm": sw.get("standing_height_mm"),
            "physical_interference_count": interference.get("physical_interference_count"),
            "rl_status": gates.get("overall_rl_nominal"),
        }
        if not digital_ok:
            failures.append("digital_release")

    one_seq_lines = [line.strip() for line in (ROOT / "one-seq.md").read_text(encoding="utf-8").splitlines() if line.strip()]
    one_seq_ok = len(one_seq_lines) == 1 and "zeroth01_physical_mount_v3_rl_fixed_18dof.urdf" in one_seq_lines[0]
    checks["one_seq"] = {"gate": "PASS" if one_seq_ok else "FAIL", "nonempty_lines": len(one_seq_lines)}
    if not one_seq_ok:
        failures.append("one_seq")

    payload = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.publish_validation.v1",
        "checks": checks,
        "failures": failures,
        "rl_training": "READY_WITH_DYNAMIC_TORQUE_HOLD" if not failures else "BLOCKED",
        "physical_release": "HOLD_FIRST_ARTICLE_AND_AS_BUILT_IDENTIFICATION",
        "overall": "PASS_WITH_EXPLICIT_HOLDS" if not failures else "FAIL",
    }
    output = ROOT / "reports/publish_validation.json"
    output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
