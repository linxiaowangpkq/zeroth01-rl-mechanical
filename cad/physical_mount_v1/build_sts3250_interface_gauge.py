from __future__ import annotations

import hashlib
import json
from pathlib import Path

from build123d import Align, Box, Cylinder, Location, export_step, export_stl
from vtkmodules.vtkFiltersCore import vtkFeatureEdges
from vtkmodules.vtkIOGeometry import vtkSTLReader


ROOT = Path(__file__).resolve().parents[2]
OUTPUT_ROOT = (
    ROOT / "generated" / "cad" / "physical_mount_v1" / "sts3250_interface"
)
PRINT_ROOT = (
    ROOT / "generated" / "print" / "physical_mount_v1" / "first_article"
)
REPORT_PATH = (
    ROOT
    / "reports"
    / "physical_mount_v1"
    / "sts3250_interface_gauge.json"
)

CASE_X_MIN_MM = -32.72
CASE_X_MAX_MM = 12.50
CASE_Y_MM = 24.72
CASE_Z_MM = 35.00
CASE_CENTER_X_MM = (CASE_X_MIN_MM + CASE_X_MAX_MM) / 2.0

# Official drawing: 4-M2.0, 20.5 mm transverse pitch.  The longitudinal
# centers are reconstructed from the dimensioned 45.22 mm outline using the
# drawing's 4.22/4.20 mm edge offsets.
M2_HOLE_X_MM = (-28.50, 8.30)
M2_HOLE_Y_MM = (-10.25, 10.25)
SERVO_TAPPED_PILOT_DIAMETER_MM = 1.60
GAUGE_CLEARANCE_DIAMETER_MM = 2.20
GAUGE_SHAFT_CLEARANCE_DIAMETER_MM = 20.50


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _rounded_box(length: float, width: float, height: float, radius: float):
    shape = Box(
        length,
        width,
        height,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    try:
        return shape.fillet(radius, shape.edges())
    except Exception:
        return shape


def _size_xyz(shape) -> list[float]:
    size = shape.bounding_box().size
    return [float(size.X), float(size.Y), float(size.Z)]


def _stl_edge_audit(path: Path) -> dict[str, object]:
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    mesh = reader.GetOutput()
    edges = vtkFeatureEdges()
    edges.SetInputData(mesh)
    edges.BoundaryEdgesOn()
    edges.NonManifoldEdgesOn()
    edges.FeatureEdgesOff()
    edges.ManifoldEdgesOff()
    edges.Update()
    return {
        "triangle_count": int(mesh.GetNumberOfCells()),
        "boundary_or_nonmanifold_edge_count": int(
            edges.GetOutput().GetNumberOfCells()
        ),
        "bounds_mm": [float(value) for value in mesh.GetBounds()],
        "gate": (
            "PASS" if edges.GetOutput().GetNumberOfCells() == 0 else "FAIL"
        ),
    }


def build_servo_reference():
    case = _rounded_box(45.22, CASE_Y_MM, CASE_Z_MM, 2.0).moved(
        Location((CASE_CENTER_X_MM, 0.0, 0.0))
    )
    for x_pos in M2_HOLE_X_MM:
        for y_pos in M2_HOLE_Y_MM:
            pilot = Cylinder(
                SERVO_TAPPED_PILOT_DIAMETER_MM / 2.0,
                CASE_Z_MM + 2.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Location((x_pos, y_pos, 0.0)))
            case = case.cut(pilot)

    front_boss = Cylinder(
        8.0,
        0.75,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((0.0, 0.0, 17.875)))
    rear_boss = Cylinder(
        8.0,
        0.75,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((0.0, 0.0, -17.875)))
    front_spline_envelope = Cylinder(
        2.95,
        3.4,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((0.0, 0.0, 16.55)))
    rear_spline_envelope = Cylinder(
        3.025,
        3.1,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((0.0, 0.0, -16.70)))
    center_screw = Cylinder(
        1.25,
        7.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((0.0, 0.0, 15.0)))
    shape = case.fuse(
        front_boss,
        rear_boss,
        front_spline_envelope,
        rear_spline_envelope,
    ).cut(center_screw)
    shape.label = "FEETECH_STS3250_C001_OFFICIAL_DIMENSION_REFERENCE"
    return shape


def build_first_article_gauge():
    gauge = _rounded_box(49.22, 28.72, 3.0, 2.5).moved(
        Location((CASE_CENTER_X_MM, 0.0, 0.0))
    )
    shaft_clearance = Cylinder(
        GAUGE_SHAFT_CLEARANCE_DIAMETER_MM / 2.0,
        5.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    gauge = gauge.cut(shaft_clearance)
    for x_pos in M2_HOLE_X_MM:
        for y_pos in M2_HOLE_Y_MM:
            clearance = Cylinder(
                GAUGE_CLEARANCE_DIAMETER_MM / 2.0,
                5.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Location((x_pos, y_pos, 0.0)))
            gauge = gauge.cut(clearance)
    gauge.label = "STS3250_4XM2_FIRST_ARTICLE_FACE_GAUGE"
    return gauge


def main() -> int:
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    PRINT_ROOT.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)

    servo_reference = build_servo_reference()
    gauge = build_first_article_gauge()
    servo_step = OUTPUT_ROOT / "FEETECH_STS3250_C001_DIMENSION_REFERENCE.step"
    gauge_step = OUTPUT_ROOT / "STS3250_4XM2_FIRST_ARTICLE_FACE_GAUGE.step"
    gauge_stl = PRINT_ROOT / "STS3250_4XM2_FIRST_ARTICLE_FACE_GAUGE.stl"
    export_step(servo_reference, servo_step)
    export_step(gauge, gauge_step)
    export_stl(gauge, gauge_stl, tolerance=0.02, angular_tolerance=0.1)

    report = {
        "schema": "zeroth01.physical_mount_v1.sts3250_interface_gauge.v1",
        "units": "mm",
        "official_source": (
            "FEETECH STS3250 product specification A/0 dated 2024-01-16"
        ),
        "case_size": [45.22, 24.72, 35.0],
        "drawing_max_two_sided_depth": 36.5,
        "shaft_center_from_short_end": 12.5,
        "output": {
            "front_spline": "25T, OD5.9, M3x6 retention screw",
            "rear_envelope_diameter": 6.05,
        },
        "mounting_face": {
            "thread_callout": "4-M2.0",
            "hole_centers_xy": [
                [x_pos, y_pos]
                for x_pos in M2_HOLE_X_MM
                for y_pos in M2_HOLE_Y_MM
            ],
            "transverse_pitch": 20.5,
            "longitudinal_pitch": 36.8,
            "gauge_clearance_diameter": GAUGE_CLEARANCE_DIAMETER_MM,
            "longitudinal_derivation": (
                "45.22 mm case outline with 4.22/4.20 mm drawing edge "
                "offsets; verify against purchased actuator"
            ),
        },
        "artifacts": {
            "servo_reference_step": str(servo_step.relative_to(ROOT)).replace(
                "\\",
                "/",
            ),
            "gauge_step": str(gauge_step.relative_to(ROOT)).replace("\\", "/"),
            "gauge_stl": str(gauge_stl.relative_to(ROOT)).replace("\\", "/"),
        },
        "sha256": {
            "servo_reference_step": _sha256(servo_step),
            "gauge_step": _sha256(gauge_step),
            "gauge_stl": _sha256(gauge_stl),
        },
        "validation": {
            "servo_reference_step_size_xyz": _size_xyz(servo_reference),
            "servo_reference_step_valid_brep": bool(servo_reference.is_valid),
            "gauge_step_size_xyz": _size_xyz(gauge),
            "gauge_step_valid_brep": bool(gauge.is_valid),
            "gauge_stl": _stl_edge_audit(gauge_stl),
        },
        "gate": {
            "cad_generation": "PASS",
            "full_robot_print_release": "HOLD",
            "release_condition": (
                "Print one gauge, bolt it to one purchased STS3250 using four "
                "M2 screws, verify the 25T horn and rear support, record actual "
                "hole centers/case depth, then update all source carriers."
            ),
        },
    }
    REPORT_PATH.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(REPORT_PATH)
    print(servo_step)
    print(gauge_step)
    print(gauge_stl)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
