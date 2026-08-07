"""Search direct-drive STS3250 clocking at a sub-10 mm roll offset."""

from __future__ import annotations

import json
import math
from pathlib import Path

from build123d import Location, import_step, import_stl
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
REPORT = ROOT / "reports" / "v4_original_minimal" / "ankle_direct_clocking_screening.json"
HEAD_MAX_Z_MM = 74.0412372351


def transformed(row, rotation=None, translation=None):
    matrix = [list(values) for values in row["transform_local_mm_to_world_mm"]]
    if rotation is not None:
        for index in range(3):
            matrix[index][:3] = rotation[index]
    if translation is not None:
        for index in range(3):
            matrix[index][3] = translation[index]
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[r][c]) for r in range(3) for c in range(4)])
    source = ROOT / str(row["source"])
    shape = import_stl(source) if source.suffix.lower() == ".stl" else import_step(source)
    return shape.moved(Location(gp_trsf=transform))


def common_volume(first, second):
    common = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    common.Build()
    if not common.IsDone():
        # A failed Boolean on a released triangulated carrier is never allowed
        # to become a false PASS.
        return float("inf")
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(common.Shape(), props)
    return float(props.Mass())


def rotation(axis_sign: float, angle_deg: float):
    """Clock local +X from outboard toward world-up around local +Z."""

    angle = math.radians(angle_deg)
    z_axis = (axis_sign, 0.0, 0.0)
    outboard = (0.0, axis_sign, 0.0)
    up = (0.0, 0.0, 1.0)
    x_axis = tuple(
        math.cos(angle) * outboard[i] + math.sin(angle) * up[i]
        for i in range(3)
    )
    y_axis = (
        z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
        z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
        z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0],
    )
    return [[x_axis[row], y_axis[row], z_axis[row]] for row in range(3)]


def main() -> int:
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in payload["components"]}
    rows = []
    for side, axis_sign, pitch_id, roll_id, lower_id, foot_id, sole_id in (
        ("left", 1.0, "S16_STS3250_left_ankle_pitch", "S18_STS3250_left_ankle_roll", "CARRIER_3215_BothFlange_13", "CARRIER_FOOT", "LEFT_7MM_LIGHTWEIGHT_SOLE"),
        ("right", -1.0, "S15_STS3250_right_ankle_pitch", "S17_STS3250_right_ankle_roll", "CARRIER_3215_BothFlange_14", "CARRIER_FOOT_2", "RIGHT_7MM_LIGHTWEIGHT_SOLE"),
    ):
        pitch = transformed(by_id[pitch_id])
        lower = transformed(by_id[lower_id])
        pivot_base = [
            float(by_id[foot_id]["transform_local_mm_to_world_mm"][i][3])
            for i in range(3)
        ]
        for offset_mm in (5.0, 7.0, 8.0, 9.0, 9.5):
            pivot = (pivot_base[0], pivot_base[1], pivot_base[2] - offset_mm)
            foot_matrix_translation = pivot
            foot = transformed(by_id[foot_id], translation=foot_matrix_translation)
            sole = transformed(by_id[sole_id], translation=foot_matrix_translation)
            for angle_deg in range(0, 360, 30):
                servo = transformed(
                    by_id[roll_id],
                    rotation=rotation(axis_sign, float(angle_deg)),
                    translation=pivot,
                )
                pairs = []
                for name, obstacle in (
                    (pitch_id, pitch), (lower_id, lower), (foot_id, foot), (sole_id, sole)
                ):
                    overlap = common_volume(servo, obstacle)
                    pairs.append(
                        {
                            "components": [roll_id, name],
                            "overlap_mm3": overlap,
                            "gate": "PASS" if overlap <= 0.01 else "FAIL",
                        }
                    )
                minimum_z = min(servo.bounding_box().min.Z, sole.bounding_box().min.Z)
                height = HEAD_MAX_Z_MM - minimum_z
                rows.append(
                    {
                        "side": side,
                        "offset_mm": offset_mm,
                        "clocking_deg_from_outboard_toward_up": angle_deg,
                        "height_mm": height,
                        "height_gate": "PASS" if height <= 500.0 else "FAIL",
                        "pairs": pairs,
                        "screening_gate": "PASS" if (
                            height <= 500.0 and all(row["gate"] == "PASS" for row in pairs)
                        ) else "FAIL",
                    }
                )
    feasible = [row for row in rows if row["screening_gate"] == "PASS"]
    report = {
        "schema": "zeroth01.v4.ankle_direct_clocking_screening.v1",
        "rows": rows,
        "feasible": sorted(feasible, key=lambda row: (row["offset_mm"], row["height_mm"])),
        "overall": "PASS" if feasible else "FAIL",
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"overall": report["overall"], "feasible": report["feasible"]}, indent=2))
    return 0 if feasible else 1


if __name__ == "__main__":
    raise SystemExit(main())
