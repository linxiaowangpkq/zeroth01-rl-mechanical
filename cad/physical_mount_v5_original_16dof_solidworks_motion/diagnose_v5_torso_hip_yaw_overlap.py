"""Export and measure the two torso/hip-yaw STS3250 common solids."""

from __future__ import annotations

import json
from pathlib import Path

from build123d import Compound, export_step, import_step
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
from OCP.BRepCheck import BRepCheck_Analyzer

import diagnose_v5_offline_brep_interference as gate


OUT = gate.ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "diagnostics"
REPORT = gate.ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "torso_hip_yaw_overlap.json"


def common_shape(first, second):
    operation = BRepAlgoAPI_Common(first.wrapped, second.wrapped)
    operation.SetNonDestructive(True)
    operation.SetRunParallel(True)
    operation.SetUseOBB(True)
    operation.SetFuzzyValue(1.0e-7)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT common failed")
    return Compound(operation.Shape())


def main() -> int:
    data = json.loads(gate.MANIFEST.read_text(encoding="utf-8"))
    by_id = {str(row["component_id"]): row for row in data["components"]}
    torso_row = by_id["CARRIER_Z_BOT2_MASTER_BODY_SKELETON"]
    torso = gate.transformed(
        import_step(gate.ROOT / str(torso_row["source"])),
        torso_row["transform_local_mm_to_world_mm"],
    )
    OUT.mkdir(parents=True, exist_ok=True)
    rows = []
    for servo_id in (
        "S07_STS3250_right_hip_yaw",
        "S08_STS3250_left_hip_yaw",
    ):
        servo_row = by_id[servo_id]
        servo = gate.transformed(
            import_step(gate.ROOT / str(servo_row["source"])),
            servo_row["transform_local_mm_to_world_mm"],
        )
        common = common_shape(torso, servo)
        volume_mm3 = gate.common_volume(torso, servo)
        box = common.bounding_box()
        target = OUT / f"{servo_id}_TORSO_COMMON.step"
        export_step(common, target)
        rows.append(
            {
                "servo_component_id": servo_id,
                "volume_mm3": volume_mm3,
                "bbox_world_mm": {
                    "min": [float(box.min.X), float(box.min.Y), float(box.min.Z)],
                    "max": [float(box.max.X), float(box.max.Y), float(box.max.Z)],
                    "size": [float(box.size.X), float(box.size.Y), float(box.size.Z)],
                },
                "valid_brep": bool(BRepCheck_Analyzer(common.wrapped).IsValid()),
                "intersection_step": target.relative_to(gate.ROOT).as_posix(),
            }
        )
    payload = {
        "schema": "zeroth01.v5.torso_hip_yaw_overlap.v1",
        "rows": rows,
        "overall": "PASS" if all(row["volume_mm3"] > gate.VOLUME_TOLERANCE_MM3 for row in rows) else "FAIL",
    }
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
