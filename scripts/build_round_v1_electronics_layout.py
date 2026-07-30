from __future__ import annotations

import json
import math
from pathlib import Path


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
SOURCE = ROOT / "config" / "round_v1_electronics_layout_source.json"
OUTPUT = (
    ROOT
    / "generated"
    / "config"
    / "round_v1_electronics_sensor_layout.json"
)


def box_inertia(
    mass: float,
    size: list[float],
) -> dict[str, float]:
    x_size, y_size, z_size = size
    return {
        "ixx": mass * (y_size**2 + z_size**2) / 12.0,
        "iyy": mass * (x_size**2 + z_size**2) / 12.0,
        "izz": mass * (x_size**2 + y_size**2) / 12.0,
        "ixy": 0.0,
        "ixz": 0.0,
        "iyz": 0.0,
    }


def main() -> int:
    payload = json.loads(SOURCE.read_text(encoding="utf-8"))
    modules = payload.get("modules", {})
    required_modules = {
        "eye_display_module",
        "camera_module",
        "tof_module",
        "imu_module",
        "compute_module",
        "battery_pack",
    }
    if set(modules) != required_modules:
        raise ValueError(
            "electronics source must define exactly: "
            + ", ".join(sorted(required_modules))
        )

    total_mass = 0.0
    for name, module in modules.items():
        size = [float(value) for value in module["size_xyz_m"]]
        center = [float(value) for value in module["center_xyz_m"]]
        mass = float(module["nominal_mass_kg"])
        mass_range = [float(value) for value in module["mass_range_kg"]]
        if (
            len(size) != 3
            or len(center) != 3
            or not all(math.isfinite(value) and value > 0 for value in size)
            or not all(math.isfinite(value) for value in center)
            or not (0 < mass_range[0] <= mass <= mass_range[1])
        ):
            raise ValueError(f"invalid module data: {name}")
        module["box_inertia_kg_m2_at_com"] = box_inertia(mass, size)
        module["inertial_source"] = (
            "analytic uniform box using assumed nominal mass and envelope"
        )
        total_mass += mass

    payload["schema"] = "zeroth01.round_v1.electronics_layout.v1"
    payload["source_file"] = SOURCE.relative_to(ROOT).as_posix()
    payload["nominal_electronics_mass_kg"] = total_mass
    payload["rl_use_gate"] = (
        "PASS_WITH_SELECTED_HEAD_MODULES_AND_ASSUMED_TORSO_PAYLOADS"
    )
    payload["hardware_use_gate"] = (
        "BLOCKED_UNTIL_EXACT_COMPONENTS_ARE_SELECTED_WEIGHED_AND_CALIBRATED"
    )
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
