"""Sweep the remote ankle's driven pivot against the fixed pitch servo."""

from __future__ import annotations

import json
from pathlib import Path

from build123d import Location, import_step, import_stl
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
REPORT = ROOT / "reports" / "v4_original_minimal" / "remote_pivot_screening.json"
PIVOT_OFFSETS_MM = (0.0, 3.0, 5.0, 7.0, 8.0, 9.0, 9.5)


def occurrence(row, dz_mm=0.0):
    matrix = [list(values) for values in row["transform_local_mm_to_world_mm"]]
    matrix[2][3] -= dz_mm
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[r][c]) for r in range(3) for c in range(4)])
    source = ROOT / str(row["source"])
    shape = import_stl(source) if source.suffix.lower() == ".stl" else import_step(source)
    return shape.moved(Location(gp_trsf=transform))


def common_volume(first, second):
    common = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    common.Build()
    if not common.IsDone():
        raise RuntimeError("OCCT common failed")
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(common.Shape(), properties)
    return float(properties.Mass())


def main() -> int:
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in payload["components"]}
    results = []
    for side, pitch_id in (
        ("LEFT", "S16_STS3250_left_ankle_pitch"),
        ("RIGHT", "S15_STS3250_right_ankle_pitch"),
    ):
        pitch = occurrence(by_id[pitch_id])
        critical = (
            f"{side}_ANKLE_BEARINGS",
            f"{side}_ANKLE_PULLEYS",
            f"{side}_ANKLE_OUTPUT_SHAFT",
            f"{side}_ANKLE_FOOT_HORN",
            f"{side}_ANKLE_FRONT_PLATE",
            f"{side}_ANKLE_REAR_PLATE",
        )
        for offset in PIVOT_OFFSETS_MM:
            pairs = []
            for component_id in critical:
                volume = common_volume(pitch, occurrence(by_id[component_id], offset))
                pairs.append(
                    {
                        "components": [pitch_id, component_id],
                        "overlap_mm3": volume,
                        "gate": "PASS" if volume <= 0.01 else "FAIL",
                    }
                )
            height = 490.459277582387 + offset
            results.append(
                {
                    "side": side.lower(),
                    "pivot_offset_below_pitch_mm": offset,
                    "height_mm": height,
                    "height_gate": "PASS" if height <= 500.0 else "FAIL",
                    "pairs": pairs,
                    "critical_hardware_gate": "PASS" if all(row["gate"] == "PASS" for row in pairs) else "FAIL",
                }
            )
    feasible = [
        row for row in results
        if row["height_gate"] == "PASS" and row["critical_hardware_gate"] == "PASS"
    ]
    report = {
        "schema": "zeroth01.v4.remote_pivot_screening.v1",
        "rows": results,
        "feasible": feasible,
        "overall": "PASS" if feasible else "FAIL",
    }
    REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))
    return 0 if feasible else 1


if __name__ == "__main__":
    raise SystemExit(main())
