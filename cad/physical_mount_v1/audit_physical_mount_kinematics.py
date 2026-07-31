from __future__ import annotations

import csv
import importlib.util
import json
import math
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BUILD_SCRIPT = Path(__file__).with_name("build_physical_mount_v1.py")
URDF_PATH = (
    ROOT
    / "generated"
    / "urdf"
    / "physical_mount_v1"
    / "zeroth01_physical_mount_v1.urdf"
)
MANIFEST_PATH = (
    ROOT
    / "reports"
    / "physical_mount_v1"
    / "servo_component_manifest.json"
)
REPORT_ROOT = ROOT / "reports" / "physical_mount_v1"
JSON_REPORT = REPORT_ROOT / "kinematic_mount_audit.json"
CSV_REPORT = REPORT_ROOT / "kinematic_mount_audit.csv"

EXPECTED_SHAFT_OFFSET_MM = 12.5
SHAFT_OFFSET_TOLERANCE_MM = 1.0
MIRROR_JOINT_ORIGIN_TOLERANCE_MM = 0.75
MIRROR_TRANSVERSE_HOUSING_TOLERANCE_MM = 0.75
MIRROR_SIZE_TOLERANCE_MM = 0.75
MIRROR_AXIS_ANGLE_TOLERANCE_DEG = 0.10

MIRROR_PAIRS = (
    ("shoulder_pitch", "S01", "S02"),
    ("shoulder_yaw", "S03", "S06"),
    ("hip_pitch", "S04", "S05"),
    ("hip_yaw", "S07", "S08"),
    ("elbow_yaw", "S09", "S10"),
    ("hip_roll", "S11", "S12"),
    ("knee_pitch", "S13", "S14"),
    ("ankle_pitch", "S15", "S16"),
)


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


physical = _load_module("zeroth01_physical_mount_build", BUILD_SCRIPT)


def _vec_sub(a, b):
    return tuple(float(a[index]) - float(b[index]) for index in range(3))


def _dot(a, b):
    return sum(float(a[index]) * float(b[index]) for index in range(3))


def _norm(vector):
    return math.sqrt(_dot(vector, vector))


def _normalize(vector):
    length = _norm(vector)
    if length <= 1e-12:
        raise ValueError("zero-length joint axis")
    return tuple(float(value) / length for value in vector)


def _mat_vec(matrix, vector):
    return tuple(
        sum(
            float(matrix[row][column]) * float(vector[column])
            for column in range(3)
        )
        for row in range(3)
    )


def _transform_point(transform, point):
    rotation, translation = transform
    rotated = _mat_vec(rotation, point)
    return tuple(rotated[index] + float(translation[index]) for index in range(3))


def _joint_world_frame(joint, transforms):
    parent_transform = transforms[str(joint["parent"])]
    parent_rotation, _parent_translation = parent_transform
    origin_rotation, origin_translation = joint["origin"]
    origin_world = _transform_point(parent_transform, origin_translation)
    axis_parent = _mat_vec(origin_rotation, joint["axis"])
    axis_world = _normalize(_mat_vec(parent_rotation, axis_parent))
    return origin_world, axis_world


def _reflect_world_y(vector):
    return (float(vector[0]), -float(vector[1]), float(vector[2]))


def _distance_to_axis(point, axis_origin, axis_direction):
    delta = _vec_sub(point, axis_origin)
    axis = _normalize(axis_direction)
    axial = _dot(delta, axis)
    radial = tuple(delta[index] - axial * axis[index] for index in range(3))
    return _norm(radial)


def _joint_axis_in_owning_link(joint, owning_link):
    if owning_link == joint["child"]:
        return (0.0, 0.0, 0.0), tuple(joint["axis"]), "child"
    if owning_link == joint["parent"]:
        rotation, translation = joint["origin"]
        return (
            tuple(float(value) for value in translation),
            _mat_vec(rotation, joint["axis"]),
            "parent",
        )
    raise ValueError(
        f"{joint['name']}: owning link {owning_link} is neither parent nor child"
    )


def main() -> int:
    if not URDF_PATH.is_file():
        raise FileNotFoundError(URDF_PATH)
    if not MANIFEST_PATH.is_file():
        raise FileNotFoundError(MANIFEST_PATH)

    urdf_root = physical.ET.parse(URDF_PATH).getroot()
    base_link, joints = physical._load_kinematic_model(urdf_root)
    joint_by_name = {str(joint["name"]): joint for joint in joints}
    transforms = physical._forward_kinematics(base_link, joints, {})
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))["servos"]
    servo_by_id = {str(servo["id"]): servo for servo in manifest}

    servo_rows: list[dict[str, object]] = []
    world_centers: dict[str, tuple[float, float, float]] = {}
    world_joint_frames: dict[str, tuple[tuple[float, ...], tuple[float, ...]]] = {}
    for servo in manifest:
        servo_id = str(servo["id"])
        joint_name = str(servo["joint"])
        owning_link = str(servo["owning_link"])
        joint = joint_by_name[joint_name]
        local_center = tuple(
            float(value) for value in servo["center_m_in_owning_link"]
        )
        axis_origin, axis_direction, carrier_side = _joint_axis_in_owning_link(
            joint,
            owning_link,
        )
        shaft_offset_mm = (
            _distance_to_axis(local_center, axis_origin, axis_direction) * 1000.0
        )
        shaft_error_mm = abs(shaft_offset_mm - EXPECTED_SHAFT_OFFSET_MM)
        world_center = _transform_point(transforms[owning_link], local_center)
        world_centers[servo_id] = world_center
        world_joint_frames[servo_id] = _joint_world_frame(joint, transforms)
        skeleton_path = (
            ROOT
            / "generated"
            / "cad"
            / "physical_mount_v1"
            / "skeleton"
            / Path(str(servo["source_mesh"])).name
        )
        servo_rows.append(
            {
                "row_type": "servo_axis",
                "id": servo_id,
                "joint": joint_name,
                "owning_link": owning_link,
                "servo_body_side_of_joint": carrier_side,
                "joint_parent": joint["parent"],
                "joint_child": joint["child"],
                "shaft_center_offset_mm": round(shaft_offset_mm, 6),
                "expected_offset_mm": EXPECTED_SHAFT_OFFSET_MM,
                "offset_error_mm": round(shaft_error_mm, 6),
                "carrier_mesh_present": skeleton_path.is_file(),
                "gate": (
                    "PASS"
                    if shaft_error_mm <= SHAFT_OFFSET_TOLERANCE_MM
                    and skeleton_path.is_file()
                    else "FAIL"
                ),
            }
        )

    pair_rows: list[dict[str, object]] = []
    for pair_name, right_id, left_id in MIRROR_PAIRS:
        right_center = world_centers[right_id]
        left_center = world_centers[left_id]
        # The source floating-base transform rotates torso X into world Y.
        # Therefore left/right symmetry at neutral is reflection across Y=0.
        residual = (
            left_center[0] - right_center[0],
            left_center[1] + right_center[1],
            left_center[2] - right_center[2],
        )
        center_error_mm = _norm(residual) * 1000.0
        right_origin, right_axis = world_joint_frames[right_id]
        left_origin, left_axis = world_joint_frames[left_id]
        mirrored_right_origin = _reflect_world_y(right_origin)
        mirrored_right_axis = _normalize(_reflect_world_y(right_axis))
        joint_origin_residual = _vec_sub(left_origin, mirrored_right_origin)
        joint_origin_error_mm = _norm(joint_origin_residual) * 1000.0
        joint_origin_axial = _dot(
            joint_origin_residual,
            mirrored_right_axis,
        )
        joint_line_residual = tuple(
            joint_origin_residual[index]
            - joint_origin_axial * mirrored_right_axis[index]
            for index in range(3)
        )
        joint_axis_line_error_mm = _norm(joint_line_residual) * 1000.0
        axis_dot = max(
            -1.0,
            min(1.0, abs(_dot(mirrored_right_axis, _normalize(left_axis)))),
        )
        axis_angle_error_deg = math.degrees(math.acos(axis_dot))
        axial_housing_error_mm = abs(_dot(residual, mirrored_right_axis)) * 1000.0
        axial_component = tuple(
            _dot(residual, mirrored_right_axis) * mirrored_right_axis[index]
            for index in range(3)
        )
        transverse_housing_error_mm = (
            _norm(_vec_sub(residual, axial_component)) * 1000.0
        )
        right_size = sorted(
            float(value) for value in servo_by_id[right_id]["size_mm"]
        )
        left_size = sorted(
            float(value) for value in servo_by_id[left_id]["size_mm"]
        )
        size_error_mm = max(
            abs(left_size[index] - right_size[index]) for index in range(3)
        )
        pair_rows.append(
            {
                "row_type": "mirror_pair",
                "pair": pair_name,
                "right_id": right_id,
                "left_id": left_id,
                "world_mirror_plane": "Y=0",
                "right_joint_origin_world_m": [
                    round(float(value), 9) for value in right_origin
                ],
                "left_joint_origin_world_m": [
                    round(float(value), 9) for value in left_origin
                ],
                "right_joint_axis_world": [
                    round(float(value), 9) for value in right_axis
                ],
                "left_joint_axis_world": [
                    round(float(value), 9) for value in left_axis
                ],
                "right_world_center_m": [
                    round(float(value), 9) for value in right_center
                ],
                "left_world_center_m": [
                    round(float(value), 9) for value in left_center
                ],
                "mirror_residual_mm": [
                    round(float(value) * 1000.0, 6) for value in residual
                ],
                "housing_center_mirror_error_mm": round(center_error_mm, 6),
                "housing_axial_offset_mm": round(axial_housing_error_mm, 6),
                "housing_transverse_mirror_error_mm": round(
                    transverse_housing_error_mm,
                    6,
                ),
                "joint_origin_mirror_error_mm": round(
                    joint_origin_error_mm,
                    6,
                ),
                "joint_axis_line_mirror_error_mm": round(
                    joint_axis_line_error_mm,
                    6,
                ),
                "joint_axis_mirror_angle_error_deg": round(
                    axis_angle_error_deg,
                    9,
                ),
                "max_sorted_size_error_mm": round(size_error_mm, 6),
                "strict_symmetry_gate": (
                    "PASS"
                    if joint_axis_line_error_mm
                    <= MIRROR_JOINT_ORIGIN_TOLERANCE_MM
                    and transverse_housing_error_mm
                    <= MIRROR_TRANSVERSE_HOUSING_TOLERANCE_MM
                    and axis_angle_error_deg
                    <= MIRROR_AXIS_ANGLE_TOLERANCE_DEG
                    and size_error_mm <= MIRROR_SIZE_TOLERANCE_MM
                    else "REVIEW"
                ),
            }
        )

    axis_gate = (
        "PASS"
        if len(servo_rows) == 16
        and all(row["gate"] == "PASS" for row in servo_rows)
        else "FAIL"
    )
    strict_symmetry_gate = (
        "PASS"
        if len(pair_rows) == 8
        and all(
            row["strict_symmetry_gate"] == "PASS" for row in pair_rows
        )
        else "FAIL"
    )
    overall_status = (
        "PASS"
        if axis_gate == strict_symmetry_gate == "PASS"
        else (
            "PASS_WITH_SOURCE_ASYMMETRY_REVIEW"
            if axis_gate == "PASS"
            else "FAIL"
        )
    )

    report = {
        "schema": "zeroth01.physical_mount_v1.kinematic_mount_audit.v1",
        "source": (
            "servo bodies extracted in place from the original Zeroth-01 "
            "assembled link meshes; joint frames retained from source URDF"
        ),
        "servo_count": len(servo_rows),
        "mirror_pair_count": len(pair_rows),
        "expected_sts32xx_shaft_to_case_center_offset_mm": (
            EXPECTED_SHAFT_OFFSET_MM
        ),
        "shaft_offset_tolerance_mm": SHAFT_OFFSET_TOLERANCE_MM,
        "mirror_joint_origin_tolerance_mm": MIRROR_JOINT_ORIGIN_TOLERANCE_MM,
        "mirror_transverse_housing_tolerance_mm": (
            MIRROR_TRANSVERSE_HOUSING_TOLERANCE_MM
        ),
        "mirror_axis_angle_tolerance_deg": MIRROR_AXIS_ANGLE_TOLERANCE_DEG,
        "mirror_size_tolerance_mm": MIRROR_SIZE_TOLERANCE_MM,
        "axis_alignment_gate": axis_gate,
        "strict_left_right_symmetry_gate": strict_symmetry_gate,
        "overall_mount_provenance_gate": axis_gate,
        "overall_status": overall_status,
        "servos": servo_rows,
        "mirror_pairs": pair_rows,
        "manufacturing_scope": {
            "source_sts3215_family_fit": (
                "PASS: component identity, source carrier ownership, shaft "
                "axis, and original in-place transforms; strict bilateral "
                "source deviations remain separately reported"
            ),
            "sts3250_hole_interface": (
                "NOT_SIGNED_OFF: official STS3250 drawing calls out 4-M2.0; "
                "first-article hole and horn fit must be checked against a "
                "purchased actuator before printing the full set"
            ),
        },
    }
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    JSON_REPORT.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    rows = servo_rows + pair_rows
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with CSV_REPORT.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(
        json.dumps(
            {
                "overall_mount_provenance_gate": axis_gate,
                "axis_alignment_gate": axis_gate,
                "strict_left_right_symmetry_gate": strict_symmetry_gate,
                "overall_status": overall_status,
                "json": str(JSON_REPORT),
                "csv": str(CSV_REPORT),
            },
            ensure_ascii=False,
        )
    )
    return 0 if axis_gate == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
