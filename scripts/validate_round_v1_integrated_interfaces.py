from __future__ import annotations

import json
import math
from pathlib import Path

from build123d import import_step
import trimesh


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
PARTS = ROOT / "generated" / "cad" / "round_v1" / "parts"
SERVO = (
    ROOT
    / "source_assets"
    / "vendor"
    / "sts3250"
    / "FEETECH_STS3250.step"
)
REPORT = ROOT / "reports" / "round_v1_integrated_interface_gate.json"
FIT_CHECK_DIR = (
    ROOT
    / "generated"
    / "print"
    / "round_v1"
    / "fit_check_non_load_bearing"
    / "final"
)

DEFINITIONS = {
    "parent_servo_cage": {
        "path": PARTS / "ZEROTH01_ROUND_V1_SERVO_CAGE.step",
        "expected_solids": 1,
        "role": "parent-side torque-reaction cage",
    },
    "child_output_adapter_front": {
        "path": PARTS / "ZEROTH01_ROUND_V1_OUTPUT_HUB_FRONT.step",
        "expected_solids": 1,
        "role": "child-side purchased-horn adapter",
    },
    "child_output_adapter_rear": {
        "path": PARTS / "ZEROTH01_ROUND_V1_OUTPUT_HUB_REAR.step",
        "expected_solids": 1,
        "role": "child-side purchased-horn adapter/opposite support",
    },
    "child_output_adapter_pair": {
        "path": PARTS / "ZEROTH01_ROUND_V1_OUTPUT_HUB.step",
        "expected_solids": 2,
        "role": "SolidWorks placement pair; manufacture as two files above",
    },
}
FIT_CHECK_STL = {
    "parent_servo_cage": (
        FIT_CHECK_DIR / "ZEROTH01_ROUND_V1_SERVO_CAGE_FIT_CHECK_ONLY.stl"
    ),
    "child_output_adapter_front": (
        FIT_CHECK_DIR
        / "ZEROTH01_ROUND_V1_OUTPUT_HUB_FRONT_FIT_CHECK_ONLY.stl"
    ),
    "child_output_adapter_rear": (
        FIT_CHECK_DIR
        / "ZEROTH01_ROUND_V1_OUTPUT_HUB_REAR_FIT_CHECK_ONLY.stl"
    ),
}


def bbox_payload(shape) -> dict[str, list[float]]:
    box = shape.bounding_box()
    return {
        "min_mm": [box.min.X, box.min.Y, box.min.Z],
        "max_mm": [box.max.X, box.max.Y, box.max.Z],
        "size_mm": [box.size.X, box.size.Y, box.size.Z],
    }


def main() -> int:
    rows: dict[str, dict[str, object]] = {}
    geometry_pass = True
    for name, definition in DEFINITIONS.items():
        path = definition["path"]
        shape = import_step(path)
        solids = list(shape.solids())
        positive = bool(solids) and all(
            math.isfinite(float(solid.volume)) and float(solid.volume) > 0.0
            for solid in solids
        )
        passed = positive and len(solids) == definition["expected_solids"]
        geometry_pass = geometry_pass and passed
        rows[name] = {
            "step": path.relative_to(ROOT).as_posix(),
            "role": definition["role"],
            "solid_count": len(solids),
            "expected_solid_count": definition["expected_solids"],
            "volume_mm3": sum(float(solid.volume) for solid in solids),
            "solids": [
                {
                    "volume_mm3": float(solid.volume),
                    "bbox": bbox_payload(solid),
                }
                for solid in solids
            ],
            "bbox": bbox_payload(shape),
            "gate": "PASS" if passed else "FAIL",
        }

    servo = import_step(SERVO)
    servo_size = bbox_payload(servo)["size_mm"]
    # The official 35 mm dimension is the body length; the vendor STEP's
    # 37.4 mm total includes mounting ears. The two other official envelope
    # axes remain directly comparable.
    servo_bbox_gate = (
        abs(float(servo_size[0]) - 45.22) <= 0.5
        and abs(float(servo_size[2]) - 24.72) <= 0.5
        and 35.0 <= float(servo_size[1]) <= 38.0
    )
    fit_rows: dict[str, dict[str, object]] = {}
    fit_check_pass = True
    for name, path in FIT_CHECK_STL.items():
        mesh = trimesh.load_mesh(path, process=True)
        if not isinstance(mesh, trimesh.Trimesh):
            raise TypeError(f"expected one fit-check mesh: {path}")
        components = list(mesh.split(only_watertight=False))
        step_volume = float(rows[name]["volume_mm3"])
        volume_error = abs(float(mesh.volume) - step_volume) / step_volume
        passed = bool(
            mesh.is_watertight
            and mesh.is_winding_consistent
            and len(components) == 1
            and volume_error <= 0.005
        )
        fit_check_pass = fit_check_pass and passed
        fit_rows[name] = {
            "stl": path.relative_to(ROOT).as_posix(),
            "component_count": len(components),
            "watertight": bool(mesh.is_watertight),
            "winding_consistent": bool(mesh.is_winding_consistent),
            "step_mesh_volume_error_ratio": volume_error,
            "scope": "dimensional fit check only; not load-bearing approval",
            "gate": "PASS" if passed else "FAIL",
        }
    payload = {
        "schema": "zeroth01.round_v1.integrated_interface_gate.v1",
        "servo_model": "FEETECH STS3250",
        "servo_source_step": SERVO.relative_to(ROOT).as_posix(),
        "official_envelope_mm": [45.22, 24.72, 35.0],
        "official_35mm_semantics": (
            "body length; imported total mounting-ear extent is 37.4 mm"
        ),
        "imported_servo_bbox_size_mm": servo_size,
        "servo_envelope_gate": "PASS" if servo_bbox_gate else "FAIL",
        "nominal_servo_cage_clearance_mm_per_side": 0.6,
        "parent_interface": (
            "servo housing plus cage is rigidly attached to parent joint frame"
        ),
        "child_interface": (
            "front/rear adapters are rigidly attached to child link through "
            "owned 29 mm PCD; purchased 25T horns connect them to the servo"
        ),
        "horn_interface": {
            "official_known": "25T, output spline OD5.9 mm, M3 retention",
            "unknown_not_invented": (
                "spline tooth form and purchased-horn accessory hole pattern"
            ),
            "adapter_strategy": (
                "four radial M3 slots accept measured horn PCD 11-20 mm; "
                "freeze only after the supplied horn is measured"
            ),
        },
        "manufacturing": {
            "production_candidate": "CNC 6061-T6 cage and output adapters",
            "included_stl_scope": "fit check only; not load-bearing approval",
            "hardware_gate": (
                "BLOCKED_UNTIL_PURCHASED_HORN_FASTENER_BEARING_CABLE_AND_"
                "TOLERANCE_RFQ_ARE_CONFIRMED"
            ),
        },
        "parts": rows,
        "fit_check_stl": fit_rows,
        "fit_check_stl_gate": "PASS" if fit_check_pass else "FAIL",
        "geometry_gate": "PASS" if geometry_pass else "FAIL",
        "overall": (
            "PASS_WITH_HARDWARE_LIMITATIONS"
            if geometry_pass and servo_bbox_gate and fit_check_pass
            else "FAIL"
        ),
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0 if payload["overall"] != "FAIL" else 1


if __name__ == "__main__":
    raise SystemExit(main())
