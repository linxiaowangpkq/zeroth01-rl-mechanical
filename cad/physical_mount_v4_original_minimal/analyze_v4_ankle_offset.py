"""Exact STS3250-to-STS3250 clearance sweep for a compact v4 ankle.

Only the RL-added ankle-roll centre is moved.  The released Zeroth-01 ankle
pitch servo and every original joint centre remain fixed.  Results are a
screening gate; the selected offset still requires a complete SolidWorks
interference and range-of-motion gate with a freshly generated carrier.
"""

from __future__ import annotations

import json
from pathlib import Path

from build123d import Location, import_step, import_stl
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal"
    / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)
REPORT = ROOT / "reports" / "v4_original_minimal" / "ankle_offset_screening.json"
CURRENT_OFFSET_MM = 30.0
CURRENT_HEIGHT_MM = 520.459277582387
HEIGHT_LIMIT_MM = 500.0
OFFSETS_MM = (5.0, 7.0, 8.0, 9.0, 9.5, 10.0, 12.0, 15.0, 20.0, 25.0, 30.0)


def moved(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        *[float(matrix[row][column]) for row in range(3) for column in range(4)]
    )
    return shape.moved(Location(gp_trsf=transform))


def occurrence(root: Path, row: dict[str, object], dz_mm: float = 0.0):
    matrix = [list(values) for values in row["transform_local_mm_to_world_mm"]]
    matrix[2][3] += dz_mm
    source = root / str(row["source"])
    shape = import_stl(source) if source.suffix.lower() == ".stl" else import_step(source)
    return moved(shape, matrix)


def occurrence_delta(
    root: Path, row: dict[str, object], dx_mm: float, dy_mm: float, dz_mm: float
):
    matrix = [list(values) for values in row["transform_local_mm_to_world_mm"]]
    matrix[0][3] += dx_mm
    matrix[1][3] += dy_mm
    matrix[2][3] += dz_mm
    source = root / str(row["source"])
    shape = import_stl(source) if source.suffix.lower() == ".stl" else import_step(source)
    return moved(shape, matrix)


def common_volume(first, second) -> float:
    common = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    common.Build()
    if not common.IsDone():
        raise RuntimeError("OCCT Boolean common failed")
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(common.Shape(), properties)
    return float(properties.Mass())


def clearance(first, second) -> float:
    distance = BRepExtrema_DistShapeShape(first.wrapped, second.wrapped)
    distance.Perform()
    if not distance.IsDone():
        raise RuntimeError("OCCT distance calculation failed")
    return float(distance.Value())


def optional_clearance(first, second) -> float | None:
    try:
        return clearance(first, second)
    except RuntimeError:
        # Some released STL-derived compound carriers do not support the OCCT
        # extremum solver.  Exact Boolean-common remains the release screen.
        return None


def main() -> int:
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in payload["components"]}
    rows = []
    for side, pitch_id, roll_id in (
        ("left", "S16_STS3250_left_ankle_pitch", "S18_STS3250_left_ankle_roll"),
        ("right", "S15_STS3250_right_ankle_pitch", "S17_STS3250_right_ankle_roll"),
    ):
        for offset_mm in OFFSETS_MM:
            pitch = occurrence(ROOT, by_id[pitch_id])
            # The released v4 roll centre is 30 mm below pitch.  A candidate
            # offset moves roll/foot/sole upward by (30 - offset) mm.
            dz_mm = CURRENT_OFFSET_MM - offset_mm
            roll = occurrence(ROOT, by_id[roll_id], dz_mm)
            overlap = common_volume(pitch, roll)
            gap = 0.0 if overlap > 1.0e-6 else clearance(pitch, roll)
            resulting_height = CURRENT_HEIGHT_MM - dz_mm
            rows.append(
                {
                    "side": side,
                    "offset_mm": offset_mm,
                    "servo_overlap_mm3": overlap,
                    "servo_clearance_mm": gap,
                    "resulting_height_mm": resulting_height,
                    "height_gate": "PASS" if resulting_height <= HEIGHT_LIMIT_MM else "FAIL",
                    "servo_pair_gate": "PASS" if overlap <= 0.01 else "FAIL",
                }
            )
    feasible = [
        row for row in rows
        if row["height_gate"] == "PASS" and row["servo_pair_gate"] == "PASS"
    ]
    remote_drive_rows = []
    for side, roll_id, foot_id, sole_id in (
        ("left", "S18_STS3250_left_ankle_roll", "CARRIER_FOOT", "LEFT_7MM_LIGHTWEIGHT_SOLE"),
        ("right", "S17_STS3250_right_ankle_roll", "CARRIER_FOOT_2", "RIGHT_7MM_LIGHTWEIGHT_SOLE"),
    ):
        # Remote 1:1 belt candidate: retain the proven 30 mm physical servo
        # separation, but return the foot and its roll pivot to the released
        # Zeroth-01 height.  The lower servo drives the upper coaxial pivot by
        # two equal pulleys instead of making its spline the foot pivot.
        roll = occurrence(ROOT, by_id[roll_id])
        foot = occurrence(ROOT, by_id[foot_id], CURRENT_OFFSET_MM)
        sole = occurrence(ROOT, by_id[sole_id], CURRENT_OFFSET_MM)
        pairs = []
        for name, candidate in ((foot_id, foot), (sole_id, sole)):
            overlap = common_volume(roll, candidate)
            gap = 0.0 if overlap > 1.0e-6 else optional_clearance(roll, candidate)
            pairs.append(
                {
                    "components": [roll_id, name],
                    "overlap_mm3": overlap,
                    "clearance_mm": gap,
                    "gate": "PASS" if overlap <= 0.01 else "FAIL",
                }
            )
        remote_drive_rows.append(
            {
                "side": side,
                "servo_center_offset_mm": CURRENT_OFFSET_MM,
                "foot_pivot_offset_mm": 0.0,
                "resulting_height_mm": CURRENT_HEIGHT_MM - CURRENT_OFFSET_MM,
                "pairs": pairs,
                "screening_gate": "PASS" if all(row["gate"] == "PASS" for row in pairs) else "FAIL",
            }
        )
    outboard_sweep = []
    for side, outward_sign, pitch_id, lower_id, roll_id, foot_id, sole_id in (
        (
            "left", 1.0, "S16_STS3250_left_ankle_pitch",
            "CARRIER_3215_BothFlange_13", "S18_STS3250_left_ankle_roll",
            "CARRIER_FOOT", "LEFT_7MM_LIGHTWEIGHT_SOLE",
        ),
        (
            "right", -1.0, "S15_STS3250_right_ankle_pitch",
            "CARRIER_3215_BothFlange_14", "S17_STS3250_right_ankle_roll",
            "CARRIER_FOOT_2", "RIGHT_7MM_LIGHTWEIGHT_SOLE",
        ),
    ):
        obstacles = {
            pitch_id: occurrence(ROOT, by_id[pitch_id]),
            lower_id: occurrence(ROOT, by_id[lower_id]),
            foot_id: occurrence(ROOT, by_id[foot_id], CURRENT_OFFSET_MM),
            sole_id: occurrence(ROOT, by_id[sole_id], CURRENT_OFFSET_MM),
        }
        sole_min_z = obstacles[sole_id].bounding_box().min.Z
        for outboard_mm in (15.0, 20.0, 25.0, 30.0, 35.0, 40.0):
            for raise_mm in (0.0, 5.0, 10.0, 15.0):
                roll = occurrence_delta(
                    ROOT, by_id[roll_id], 0.0, outward_sign * outboard_mm, raise_mm
                )
                pairs = []
                for name, obstacle in obstacles.items():
                    overlap = common_volume(roll, obstacle)
                    pairs.append(
                        {
                            "components": [roll_id, name],
                            "overlap_mm3": overlap,
                            "gate": "PASS" if overlap <= 0.01 else "FAIL",
                        }
                    )
                minimum_z = min(sole_min_z, roll.bounding_box().min.Z)
                resulting_height = 74.0412372351 - minimum_z
                outboard_sweep.append(
                    {
                        "side": side,
                        "outboard_mm": outboard_mm,
                        "servo_raise_mm": raise_mm,
                        "belt_center_distance_mm": (
                            outboard_mm ** 2 + (CURRENT_OFFSET_MM - raise_mm) ** 2
                        ) ** 0.5,
                        "resulting_height_mm": resulting_height,
                        "height_gate": "PASS" if resulting_height <= HEIGHT_LIMIT_MM else "FAIL",
                        "pairs": pairs,
                        "screening_gate": "PASS" if (
                            resulting_height <= HEIGHT_LIMIT_MM
                            and all(row["gate"] == "PASS" for row in pairs)
                        ) else "FAIL",
                    }
                )
    feasible_outboard = [
        row for row in outboard_sweep if row["screening_gate"] == "PASS"
    ]
    result = {
        "schema": "zeroth01.v4.ankle_offset_screening.v1",
        "truth_boundary": (
            "Exact B-Rep screening of the two adjacent STS3250 bodies only; "
            "a selected candidate is not released until its new carrier and "
            "full motion assembly pass SolidWorks interference detection."
        ),
        "current_offset_mm": CURRENT_OFFSET_MM,
        "current_height_mm": CURRENT_HEIGHT_MM,
        "height_limit_mm": HEIGHT_LIMIT_MM,
        "rows": rows,
        "feasible_servo_pair_and_height_offsets_mm": sorted(
            {float(row["offset_mm"]) for row in feasible}
        ),
        "remote_1to1_belt_candidate": remote_drive_rows,
        "outboard_remote_belt_sweep": outboard_sweep,
        "recommended_outboard_candidates": sorted(
            feasible_outboard,
            key=lambda row: (row["belt_center_distance_mm"], row["outboard_mm"]),
        )[:8],
        "overall": "PASS" if feasible or feasible_outboard or all(
            row["screening_gate"] == "PASS" for row in remote_drive_rows
        ) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
