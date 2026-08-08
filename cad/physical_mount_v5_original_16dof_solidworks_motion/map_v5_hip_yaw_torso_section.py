"""Print joint-local occupancy maps for the fixed torso around right hip yaw."""

from __future__ import annotations

import json

import numpy as np
from build123d import Compound, import_step
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepClass3d import BRepClass3d_SolidClassifier
from OCP.TopAbs import TopAbs_IN, TopAbs_ON
from OCP.gp import gp_Pnt, gp_Trsf

import diagnose_v5_offline_brep_interference as gate


ROOT = gate.ROOT
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "hip_yaw_torso_section_map.json"
X_VALUES = list(range(-120, 31, 5))
Y_VALUES = list(range(-60, 61, 5))
Z_VALUES = (-40.0, -38.0, -36.0, -34.0, -30.0)


def matrix_shape(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        *[float(matrix[row][column]) for row in range(3) for column in range(4)]
    )
    return Compound(BRepBuilderAPI_Transform(shape.wrapped, transform, True).Shape())


def main() -> int:
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    servo_row = by_id["S07_STS3250_right_hip_yaw"]
    servo_world = np.asarray(servo_row["transform_local_mm_to_world_mm"], dtype=float)
    joint_world = servo_world.copy()
    joint_world[:3, 3] -= joint_world[:3, 2] * 4.0
    world_to_joint = np.linalg.inv(joint_world)
    torso_row = by_id["CARRIER_Z_BOT2_MASTER_BODY_SKELETON"]
    torso = matrix_shape(
        import_step(ROOT / str(torso_row["source"])),
        world_to_joint @ np.asarray(torso_row["transform_local_mm_to_world_mm"], dtype=float),
    )
    solids = list(torso.solids())

    def occupied(x_mm: float, y_mm: float, z_mm: float) -> bool:
        point = gp_Pnt(x_mm, y_mm, z_mm)
        for solid in solids:
            classifier = BRepClass3d_SolidClassifier(solid.wrapped, point, 1.0e-5)
            if classifier.State() in (TopAbs_IN, TopAbs_ON):
                return True
        return False

    maps = []
    for z_mm in Z_VALUES:
        lines = []
        for y_mm in reversed(Y_VALUES):
            line = "".join("#" if occupied(x_mm, y_mm, z_mm) else "." for x_mm in X_VALUES)
            lines.append(line)
        maps.append(
            {
                "z_mm": z_mm,
                "x_min_mm": X_VALUES[0],
                "x_step_mm": 5,
                "y_max_mm": Y_VALUES[-1],
                "y_step_mm": -5,
                "rows": lines,
            }
        )
    payload = {
        "schema": "zeroth01.v5.hip_yaw_torso_section_map.v1",
        "legend": "# fixed torso solid, . free space; rows descend in Y, columns ascend in X",
        "maps": maps,
    }
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    for section in maps:
        print(f"z={section['z_mm']:.1f} mm", flush=True)
        print("\n".join(section["rows"]), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
