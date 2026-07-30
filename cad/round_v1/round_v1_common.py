from __future__ import annotations

import copy
import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

from build123d import (
    Align,
    Box,
    Color,
    Compound,
    Cylinder,
    Location,
    Plane,
    Shape,
    Solid,
    Sphere,
    Vector,
    import_step,
)
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
URDF_PATH = ROOT / "generated" / "urdf" / "zeroth01_rl_ready.urdf"
SERVO_STEP = (
    ROOT
    / "source_assets"
    / "vendor"
    / "sts3250"
    / "FEETECH_STS3250.step"
)
SERVO_AXIS_REPORT = ROOT / "reports" / "round_v1_servo_axis_alignment.csv"
SERVO_PHASE_CONFIG = (
    ROOT
    / "generated"
    / "config"
    / "zeroth01_sts3250_mount_phase.json"
)
ELECTRONICS_LAYOUT_SOURCE = (
    ROOT / "config" / "round_v1_electronics_layout_source.json"
)

CREAM = Color("#E8D2B3")
TAN = Color("#B7875E")
DARK = Color("#2A2D32")
TEAL = Color("#55C9C6")
SERVO_METAL = Color("#636B73")

WALL_MM = 2.4
SEAM_GAP_MM = 0.35
SOLE_THICKENING_MM = 8.0


def _mat_mul(a: list[list[float]], b: list[list[float]]) -> list[list[float]]:
    return [
        [
            sum(a[row][index] * b[index][column] for index in range(3))
            for column in range(3)
        ]
        for row in range(3)
    ]


def _mat_vec(matrix: list[list[float]], vector: list[float]) -> list[float]:
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def _vec_add(a: list[float], b: list[float]) -> list[float]:
    return [a[index] + b[index] for index in range(3)]


def _tf_mul(
    a: tuple[list[list[float]], list[float]],
    b: tuple[list[list[float]], list[float]],
) -> tuple[list[list[float]], list[float]]:
    ar, at = a
    br, bt = b
    return _mat_mul(ar, br), _vec_add(_mat_vec(ar, bt), at)


def _rpy_matrix(rpy: list[float]) -> list[list[float]]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return _mat_mul(rz, _mat_mul(ry, rx))


def _parse_vec(text: str | None, default: tuple[float, float, float]) -> list[float]:
    return [float(value) for value in text.split()] if text else list(default)


def load_neutral_kinematics() -> tuple[
    list[dict[str, object]],
    dict[str, tuple[list[list[float]], list[float]]],
]:
    root = ET.parse(URDF_PATH).getroot()
    joints: list[dict[str, object]] = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        joints.append(
            {
                "name": joint.get("name", ""),
                "type": joint.get("type", ""),
                "parent": joint.find("parent").get("link", ""),
                "child": joint.find("child").get("link", ""),
                "origin": (
                    _rpy_matrix(
                        _parse_vec(
                            origin.get("rpy") if origin is not None else None,
                            (0.0, 0.0, 0.0),
                        )
                    ),
                    _parse_vec(
                        origin.get("xyz") if origin is not None else None,
                        (0.0, 0.0, 0.0),
                    ),
                ),
                "axis": _parse_vec(
                    joint.find("axis").get("xyz")
                    if joint.find("axis") is not None
                    else None,
                    (0.0, 0.0, 1.0),
                ),
            }
        )

    identity = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
    transforms: dict[str, tuple[list[list[float]], list[float]]] = {
        "base": (identity, [0.0, 0.0, 0.0])
    }
    pending = list(joints)
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = str(joint["parent"])
            if parent not in transforms:
                continue
            transforms[str(joint["child"])] = _tf_mul(
                transforms[parent], joint["origin"]
            )
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise RuntimeError(
                f"URDF tree did not resolve: {[item['name'] for item in pending]}"
            )
    return joints, transforms


def location_from_transform(
    transform: tuple[list[list[float]], list[float]]
) -> Location:
    rotation, translation_m = transform
    transform_ocp = gp_Trsf()
    transform_ocp.SetValues(
        rotation[0][0],
        rotation[0][1],
        rotation[0][2],
        translation_m[0] * 1000.0,
        rotation[1][0],
        rotation[1][1],
        rotation[1][2],
        translation_m[1] * 1000.0,
        rotation[2][0],
        rotation[2][1],
        rotation[2][2],
        translation_m[2] * 1000.0,
    )
    return Location(gp_trsf=transform_ocp)


def rounded_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    radius: float,
) -> Shape:
    shape = Box(
        *size,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    shape = shape.fillet(radius, shape.edges())
    return shape.moved(Location(center))


def _shape_from_list(items) -> Shape:
    shapes = list(items)
    if not shapes:
        raise RuntimeError("boolean operation returned no geometry")
    return shapes[0] if len(shapes) == 1 else Compound(shapes)


def _half(
    shape: Shape,
    split_y: float,
    side: str,
    seam_gap: float = SEAM_GAP_MM,
) -> Shape:
    if side == "front":
        clip = Box(
            500.0,
            250.0,
            500.0,
            align=(Align.CENTER, Align.MAX, Align.CENTER),
        ).moved(Location((0.0, split_y - seam_gap / 2.0, 0.0)))
    elif side == "back":
        clip = Box(
            500.0,
            250.0,
            500.0,
            align=(Align.CENTER, Align.MIN, Align.CENTER),
        ).moved(Location((0.0, split_y + seam_gap / 2.0, 0.0)))
    else:
        raise ValueError(side)
    return _shape_from_list(shape.intersect(clip))


def _add_seam_bosses(
    shell: Shape,
    split_y: float,
    positions_xz: list[tuple[float, float]],
) -> Shape:
    result = shell
    for x_pos, z_pos in positions_xz:
        plane = Plane(
            origin=(x_pos, split_y - 7.0, z_pos),
            x_dir=(1.0, 0.0, 0.0),
            z_dir=(0.0, 1.0, 0.0),
        )
        boss = Solid.make_cylinder(6.0, 14.0, plane)
        result = result.fuse(boss)
        hole_plane = Plane(
            origin=(x_pos, split_y - 9.0, z_pos),
            x_dir=(1.0, 0.0, 0.0),
            z_dir=(0.0, 1.0, 0.0),
        )
        result = result.cut(Solid.make_cylinder(1.7, 18.0, hole_plane))
    return result


def chest_shell(side: str) -> Shape:
    # Keep an 8 mm waist gap above the pelvis shell. The earlier -79 mm
    # bottom edge entered the hip-roll envelope by up to 0.24 mm at guarded
    # hip-pitch limits.
    center = (0.0, 5.0, -6.0)
    outer = rounded_box((170.0, 76.0, 130.0), center, 30.0)
    inner = rounded_box((160.6, 66.6, 116.0), center, 25.5)
    shell = outer.cut(inner)
    for x_pos in (-78.15, 78.15):
        shell = shell.cut(Sphere(31.0).moved(Location((x_pos, -0.34, 43.05))))
    shell = _add_seam_bosses(
        shell,
        center[1],
        [(-83.0, -42.0), (83.0, -42.0), (-83.0, 18.0), (83.0, 18.0)],
    )
    piece = _half(shell, center[1], side)
    piece.label = f"ROUND_V1_CHEST_{side.upper()}"
    piece.color = CREAM
    return piece


def head_shell(side: str) -> Shape:
    center = (0.0, -3.0, 99.0)
    outer = rounded_box((132.0, 66.0, 78.0), center, 31.0)
    inner = rounded_box((114.0, 48.0, 64.0), (0.0, -1.0, 99.0), 22.0)
    shell = outer.cut(inner)
    for x_pos in (-47.0, 47.0):
        # Use a solid, low-profile rounded ear boss. The earlier hollow sphere
        # produced a valid in-memory B-Rep whose rear-half inner-shell
        # orientation changed during STEP round-trip, corrupting mass
        # properties. This form preserves symmetry and exact STEP volume.
        ear = rounded_box(
            (42.0, 20.0, 30.0),
            (x_pos, -3.0, 130.0),
            9.0,
        )
        shell = shell.fuse(ear)
    shell = _add_seam_bosses(
        shell,
        center[1],
        [(-62.0, 86.0), (62.0, 86.0), (-58.0, 119.0), (58.0, 119.0)],
    )
    piece = _half(shell, center[1], side)
    piece.label = f"ROUND_V1_HEAD_{side.upper()}"
    piece.color = CREAM
    return piece


def pelvis_shell(side: str) -> Shape:
    center = (0.0, 5.0, -91.0)
    outer = rounded_box((108.0, 68.0, 56.0), center, 22.0)
    inner = rounded_box((95.0, 56.0, 47.0), center, 17.5)
    shell = outer.cut(inner)
    for x_pos in (-45.65, 45.65):
        shell = shell.cut(Sphere(21.0).moved(Location((x_pos, -0.34, -96.95))))
    shell = _add_seam_bosses(
        shell,
        center[1],
        [(-28.0, -116.0), (28.0, -116.0), (-28.0, -66.0), (28.0, -66.0)],
    )
    piece = _half(shell, center[1], side)
    piece.label = f"ROUND_V1_PELVIS_{side.upper()}"
    piece.color = CREAM
    return piece


def muzzle_badge() -> Shape:
    shape = rounded_box((68.0, 12.0, 33.0), (0.0, -39.0, 86.0), 5.0)
    shape.label = "ROUND_V1_MUZZLE_BADGE"
    shape.color = TAN
    return shape


def visor_badge() -> Shape:
    shape = rounded_box((92.0, 8.0, 31.0), (0.0, -38.0, 107.0), 3.5)
    shape.label = "ROUND_V1_VISOR_BADGE"
    shape.color = DARK
    return shape


def camera_lenses() -> Shape:
    lenses: list[Shape] = []
    for x_pos in (-27.0, 27.0):
        plane = Plane(
            origin=(x_pos, -43.0, 108.0),
            x_dir=(1.0, 0.0, 0.0),
            z_dir=(0.0, -1.0, 0.0),
        )
        lens = Solid.make_cylinder(9.5, 3.0, plane)
        pupil = Solid.make_cylinder(5.8, 3.6, plane)
        lens = lens.cut(pupil)
        lens.label = f"ROUND_V1_CAMERA_LENS_{'LEFT' if x_pos < 0 else 'RIGHT'}"
        lens.color = TEAL
        lenses.append(lens)
    shape = Compound(label="ROUND_V1_CAMERA_LENSES", children=lenses)
    shape.color = TEAL
    return shape


def torso_spine() -> Shape:
    vertical = rounded_box((26.0, 16.0, 190.0), (0.0, 22.0, 0.0), 5.0)
    shoulder = rounded_box((144.0, 16.0, 18.0), (0.0, 22.0, 43.0), 5.0)
    hip = rounded_box((90.0, 16.0, 18.0), (0.0, 22.0, -92.0), 5.0)
    neck = rounded_box((42.0, 16.0, 22.0), (0.0, 22.0, 67.0), 5.0)
    shape = vertical.fuse(shoulder, hip, neck)
    shape.label = "ROUND_V1_TORSO_INTERNAL_SPINE"
    shape.color = TAN
    return shape


def _electronics_module_box(name: str, radius_mm: float) -> Shape:
    layout = json.loads(
        ELECTRONICS_LAYOUT_SOURCE.read_text(encoding="utf-8")
    )
    module = layout["modules"][name]
    size_mm = tuple(
        1000.0 * float(value) for value in module["size_xyz_m"]
    )
    center_mm = tuple(
        1000.0 * float(value) for value in module["center_xyz_m"]
    )
    return rounded_box(size_mm, center_mm, radius_mm)


def camera_module() -> Shape:
    shape = _electronics_module_box("camera_module", 4.0)
    shape.label = "ROUND_V1_CAMERA_MODULE_ENVELOPE"
    shape.color = DARK
    return shape


def imu_module() -> Shape:
    shape = _electronics_module_box("imu_module", 2.0)
    shape.label = "ROUND_V1_IMU_MODULE_ENVELOPE"
    shape.color = TEAL
    return shape


def compute_module() -> Shape:
    shape = _electronics_module_box("compute_module", 4.0)
    shape.label = "ROUND_V1_COMPUTE_MODULE_ENVELOPE"
    shape.color = TEAL
    return shape


def battery_pack() -> Shape:
    shape = _electronics_module_box("battery_pack", 6.0)
    shape.label = "ROUND_V1_3S2P_BATTERY_BMS_ENVELOPE"
    shape.color = TAN
    return shape


def servo_cage() -> Shape:
    """Parent-side CNC carrier around the exact STS3250 local envelope.

    Local +Z is the servo output axis.  The front opening keeps the purchased
    25T horn serviceable; the rear ring supports the second shaft.
    """

    center = (-13.0, -9.5, 0.0)
    outer = rounded_box((52.0, 44.0, 31.0), center, 5.0)
    inner = rounded_box((46.42, 38.60, 25.92), center, 3.6)
    cage = outer.cut(inner)
    service_opening = Box(
        14.0,
        34.0,
        23.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((12.0, -9.5, 0.0)))
    cage = cage.cut(service_opening)

    # Overlap the rear ring 1 mm into the cage wall so STEP export preserves
    # one connected torque-reaction solid rather than a visually touching
    # second body.
    rear_outer = Cylinder(14.5, 4.0).moved(Location((0.0, 0.0, -16.5)))
    rear_inner = Cylinder(6.4, 5.0).moved(Location((0.0, 0.0, -16.5)))
    rear_seat = rear_outer.cut(rear_inner)
    front_outer = Cylinder(14.5, 4.0).moved(Location((0.0, 0.0, 14.0)))
    front_inner = Cylinder(11.8, 5.0).moved(Location((0.0, 0.0, 13.5)))
    front_guard = front_outer.cut(front_inner)
    anchor = rounded_box((13.0, 24.0, 20.0), (-39.0, -9.5, 0.0), 4.0)
    shape = cage.fuse(rear_seat, front_guard, anchor)
    shape.label = "ROUND_V1_STS3250_PARENT_SERVO_CAGE"
    shape.color = CREAM
    return shape


def _output_adapter_disc(z_start: float, recess_from_top: bool) -> Shape:
    """Owned slotted adapter used with a purchased 25T servo horn."""

    disc = Cylinder(18.5, 4.0).moved(Location((0.0, 0.0, z_start)))
    disc = disc.cut(
        Cylinder(1.7, 6.0).moved(Location((0.0, 0.0, z_start - 1.0)))
    )
    # Four radial M3 slots accept a measured horn PCD from 11 to 20 mm.
    # They are deliberately slots, not a claimed vendor bolt pattern.
    for x_pos in (-7.75, 7.75):
        slot = rounded_box(
            (4.5, 3.4, 6.0),
            (x_pos, 0.0, z_start + 2.0),
            1.65,
        )
        disc = disc.cut(slot)
    for y_pos in (-7.75, 7.75):
        slot = rounded_box(
            (3.4, 4.5, 6.0),
            (0.0, y_pos, z_start + 2.0),
            1.65,
        )
        disc = disc.cut(slot)
    # The owned child-bracket interface is a frozen 29 mm PCD at 45 degrees.
    for angle_deg in (45.0, 135.0, 225.0, 315.0):
        angle = math.radians(angle_deg)
        x_pos = 14.5 * math.cos(angle)
        y_pos = 14.5 * math.sin(angle)
        disc = disc.cut(
            Cylinder(1.7, 6.0).moved(
                Location((x_pos, y_pos, z_start - 1.0))
            )
        )
    recess_start = z_start + 2.5 if recess_from_top else z_start - 0.1
    return disc.cut(
        Cylinder(7.0, 1.6).moved(Location((0.0, 0.0, recess_start)))
    )


def output_hub_front() -> Shape:
    shape = _output_adapter_disc(13.0, False)
    shape.label = "ROUND_V1_STS3250_CHILD_OUTPUT_ADAPTER_FRONT"
    shape.color = TEAL
    return shape


def output_hub_rear() -> Shape:
    shape = _output_adapter_disc(-17.0, True)
    shape.label = "ROUND_V1_STS3250_CHILD_OUTPUT_ADAPTER_REAR"
    shape.color = TEAL
    return shape


def output_hub() -> Shape:
    """Two child-side adapters around purchased front/rear 25T horns.

    The official spline is OD5.9 with M3 retention.  Tooth form and accessory
    horn hole pattern are unavailable, so neither is fabricated here.  The
    owned adapters clamp purchased horns through radial slots and expose a
    frozen 29 mm PCD to the child-side fork.
    """

    shape = Compound(
        label="ROUND_V1_STS3250_CHILD_OUTPUT_HUB_PAIR",
        children=[output_hub_front(), output_hub_rear()],
    )
    shape.color = TEAL
    return shape


def sole(side: str) -> Shape:
    if side not in {"left", "right"}:
        raise ValueError(side)
    z_center = 19.025 if side == "left" else -19.025
    outer_center = (-10.0, -22.11, z_center)
    outer = rounded_box((112.0, 16.0, 64.0), outer_center, 4.0)
    cavity_center = (-10.0, -7.0, z_center)
    cavity = Box(
        101.2,
        30.0,
        53.25,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location(cavity_center))
    result = outer.cut(cavity)
    for x_pos in (-42.0, -10.0, 22.0):
        groove = Box(
            4.0,
            1.2,
            54.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Location((x_pos, -29.65, z_center)))
        result = result.cut(groove)
    # Two 12 mm hook-and-loop strap stations provide reversible retention
    # without drilling the unknown upstream foot body. The cuts cross only
    # the upper side walls (the internal cavity is already open) and preserve
    # the full 8 mm outsole beneath y=-22 mm.
    for x_pos in (-32.0, 18.0):
        strap_slot = Box(
            14.0,
            6.0,
            80.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Location((x_pos, -17.0, z_center)))
        result = result.cut(strap_slot)
    result.label = f"ROUND_V1_{side.upper()}_THICK_SOLE"
    result.color = DARK
    return result


def joint_ring() -> Shape:
    outer = Cylinder(21.0, 8.0)
    inner = Cylinder(14.5, 10.0)
    ring = outer.cut(inner)
    ring.label = "ROUND_V1_GENERIC_JOINT_RING"
    ring.color = DARK
    return ring


def _moving_joint_map(
    joints: list[dict[str, object]],
) -> dict[str, dict[str, object]]:
    return {
        str(item["name"]): item
        for item in joints
        if item["type"] in {"revolute", "continuous"}
    }


def _rotation_z(angle_rad: float) -> list[list[float]]:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]


def _joint_frame(
    joint: dict[str, object],
    link_transforms: dict[str, tuple[list[list[float]], list[float]]],
) -> tuple[list[list[float]], list[float]]:
    """Return the parent-side joint frame before the commanded rotation."""

    return _tf_mul(
        link_transforms[str(joint["parent"])],
        joint["origin"],
    )


def _servo_mount_rotation(
    joint: dict[str, object],
    joint_frame_rotation: list[list[float]],
    phase_config: dict[str, dict[str, object]],
) -> tuple[list[list[float]], int, float]:
    """Orient local +Z to the configured STS3250 output-axis direction."""

    flip_x = [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ]
    axis_local = [float(value) for value in joint["axis"]]
    positive_rotation = (
        _mat_mul(joint_frame_rotation, flip_x)
        if axis_local[2] < 0.0
        else joint_frame_rotation
    )
    entry = phase_config.get(str(joint["name"]), {})
    output_axis_sign = int(entry.get("output_axis_sign", 1))
    phase_deg = float(entry.get("phase_deg", 0.0))
    rotation = (
        positive_rotation
        if output_axis_sign == 1
        else _mat_mul(positive_rotation, flip_x)
    )
    return (
        _mat_mul(rotation, _rotation_z(math.radians(phase_deg))),
        output_axis_sign,
        phase_deg,
    )


def servo_instances() -> tuple[list[Shape], list[dict[str, object]]]:
    joints, transforms = load_neutral_kinematics()
    moving = _moving_joint_map(joints)
    source = import_step(SERVO_STEP)
    instances: list[Shape] = []
    rows: list[dict[str, object]] = []
    phase_config: dict[str, dict[str, object]] = {}
    if SERVO_PHASE_CONFIG.is_file():
        phase_config = json.loads(
            SERVO_PHASE_CONFIG.read_text(encoding="utf-8")
        ).get("joint_mount_phase", {})

    for name, joint in moving.items():
        joint_frame_rotation, translation = _joint_frame(joint, transforms)
        axis_local = [float(value) for value in joint["axis"]]
        joint_axis_world = _mat_vec(joint_frame_rotation, axis_local)
        phase_entry = phase_config.get(name, {})
        (
            servo_rotation,
            output_axis_sign,
            phase_deg,
        ) = _servo_mount_rotation(
            joint,
            joint_frame_rotation,
            phase_config,
        )
        servo_axis_world = _mat_vec(servo_rotation, [0.0, 0.0, 1.0])
        dot = max(
            -1.0,
            min(
                1.0,
                sum(
                    joint_axis_world[index] * servo_axis_world[index]
                    for index in range(3)
                ),
            ),
        )
        axis_error_deg = math.degrees(math.acos(abs(dot)))
        instance = copy.copy(source).moved(
            location_from_transform((servo_rotation, translation))
        )
        instance.label = f"STS3250_{name}"
        instance.color = SERVO_METAL
        instances.append(instance)
        rows.append(
            {
                "joint": name,
                "servo_model": "Feetech STS3250",
                "source_step": SERVO_STEP.relative_to(ROOT).as_posix(),
                "shaft_xyz_world_mm": " ".join(
                    f"{value * 1000.0:.6f}" for value in translation
                ),
                "joint_positive_axis_world": " ".join(
                    f"{value:.9f}" for value in joint_axis_world
                ),
                "servo_step_positive_z_world": " ".join(
                    f"{value:.9f}" for value in servo_axis_world
                ),
                "output_axis_sign_to_joint_positive": output_axis_sign,
                "housing_phase_deg": f"{phase_deg:.3f}",
                "housing_phase_confidence": phase_entry.get(
                    "confidence", "UNSET"
                ),
                "housing_attachment": str(joint["parent"]),
                "output_attachment": str(joint["child"]),
                "transmission_semantics": (
                    "housing_and_cage_follow_parent_joint_frame; "
                    "output_hub_follows_child_link"
                ),
                "axis_collinearity_error_deg": f"{axis_error_deg:.9f}",
                "shaft_origin_error_mm": "0.000000",
                "gate": "PASS" if axis_error_deg <= 1e-5 else "FAIL",
            }
        )
    return instances, rows


def joint_interface_instances() -> tuple[list[Shape], list[Shape]]:
    """Place neutral parent cages and child output hubs at all 16 joints."""

    joints, transforms = load_neutral_kinematics()
    moving = _moving_joint_map(joints)
    phase_config: dict[str, dict[str, object]] = {}
    if SERVO_PHASE_CONFIG.is_file():
        phase_config = json.loads(
            SERVO_PHASE_CONFIG.read_text(encoding="utf-8")
        ).get("joint_mount_phase", {})

    cage_source = servo_cage()
    hub_source = output_hub()
    cages: list[Shape] = []
    hubs: list[Shape] = []
    for name, joint in moving.items():
        frame_rotation, translation = _joint_frame(joint, transforms)
        housing_rotation, _, _ = _servo_mount_rotation(
            joint,
            frame_rotation,
            phase_config,
        )
        cage = copy.copy(cage_source).moved(
            location_from_transform((housing_rotation, translation))
        )
        cage.label = f"STS3250_PARENT_CAGE_{name}"
        cage.color = CREAM
        cages.append(cage)

        child_rotation, child_translation = transforms[str(joint["child"])]
        # At neutral the child and parent joint frames are coincident.  Keep
        # the hub's fixed clocking identical to the housing; during motion the
        # SolidWorks generator applies the child's revolute rotation.
        relative_mount = _mat_mul(
            [
                [frame_rotation[column][row] for column in range(3)]
                for row in range(3)
            ],
            housing_rotation,
        )
        hub_rotation = _mat_mul(child_rotation, relative_mount)
        hub = copy.copy(hub_source).moved(
            location_from_transform((hub_rotation, child_translation))
        )
        hub.label = f"STS3250_CHILD_HUB_{name}"
        hub.color = TEAL
        hubs.append(hub)
    return cages, hubs


def write_servo_axis_report(rows: list[dict[str, object]]) -> None:
    SERVO_AXIS_REPORT.parent.mkdir(parents=True, exist_ok=True)
    with SERVO_AXIS_REPORT.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def _capsule_between(
    start: tuple[float, float, float],
    end: tuple[float, float, float],
    radius: float,
    label: str,
) -> Shape:
    vector = tuple(end[index] - start[index] for index in range(3))
    length = math.sqrt(sum(value * value for value in vector))
    if length <= 1e-6:
        raise ValueError(f"zero-length capsule {label}")
    unit = tuple(value / length for value in vector)
    candidate = (1.0, 0.0, 0.0)
    if abs(sum(unit[index] * candidate[index] for index in range(3))) > 0.9:
        candidate = (0.0, 1.0, 0.0)
    plane = Plane(origin=start, x_dir=candidate, z_dir=unit)
    shape = Solid.make_cylinder(radius, length, plane)
    shape = shape.fuse(
        Solid.make_sphere(radius, Plane(origin=start)),
        Solid.make_sphere(radius, Plane(origin=end)),
    )
    shape.label = label
    shape.color = CREAM
    return shape


def concept_armor() -> list[Shape]:
    joints, transforms = load_neutral_kinematics()
    moving = _moving_joint_map(joints)
    points = {
        name: tuple(value * 1000.0 for value in transforms[str(item["child"])][1])
        for name, item in moving.items()
    }
    chains = [
        (
            "LEFT_ARM",
            ["left_shoulder_pitch", "left_shoulder_yaw", "left_elbow_yaw"],
            14.0,
        ),
        (
            "RIGHT_ARM",
            ["right_shoulder_pitch", "right_shoulder_yaw", "right_elbow_yaw"],
            14.0,
        ),
        (
            "LEFT_LEG",
            [
                "left_hip_pitch",
                "left_hip_yaw",
                "left_hip_roll",
                "left_knee_pitch",
                "left_ankle_pitch",
            ],
            15.5,
        ),
        (
            "RIGHT_LEG",
            [
                "right_hip_pitch",
                "right_hip_yaw",
                "right_hip_roll",
                "right_knee_pitch",
                "right_ankle_pitch",
            ],
            15.5,
        ),
    ]
    parts: list[Shape] = []
    for chain_name, names, radius in chains:
        for index, (first, second) in enumerate(zip(names, names[1:]), start=1):
            parts.append(
                _capsule_between(
                    points[first],
                    points[second],
                    radius,
                    f"ROUND_V1_{chain_name}_ARMOR_CONCEPT_{index}",
                )
            )
    return parts


def round_v1_assembly() -> Compound:
    joints, transforms = load_neutral_kinematics()
    servo_parts, servo_rows = servo_instances()
    cage_parts, hub_parts = joint_interface_instances()
    write_servo_axis_report(servo_rows)
    parts: list[Shape] = [
        chest_shell("front"),
        chest_shell("back"),
        head_shell("front"),
        head_shell("back"),
        pelvis_shell("front"),
        pelvis_shell("back"),
        muzzle_badge(),
        visor_badge(),
        camera_lenses(),
        torso_spine(),
        camera_module(),
        imu_module(),
        compute_module(),
        battery_pack(),
    ]
    for side, link_name in (("left", "foot_left"), ("right", "foot_right")):
        placed = copy.copy(sole(side)).moved(location_from_transform(transforms[link_name]))
        placed.label = f"ROUND_V1_{side.upper()}_THICK_SOLE"
        parts.append(placed)
    parts.extend(concept_armor())
    parts.extend(servo_parts)
    parts.extend(cage_parts)
    parts.extend(hub_parts)
    assembly = Compound(
        label="ZEROTH01_ROUND_V1_ASSEMBLY",
        children=parts,
    )
    return assembly


def validation_facts(shape: Shape) -> dict[str, object]:
    solids = shape.solids()
    bbox = shape.bounding_box()
    return {
        "solid_count": len(solids),
        "all_positive_volume": all(item.volume > 0 for item in solids),
        "volume_mm3": sum(item.volume for item in solids),
        "bbox_size_mm": [bbox.size.X, bbox.size.Y, bbox.size.Z],
        "bbox_center_mm": [
            (bbox.min.X + bbox.max.X) / 2.0,
            (bbox.min.Y + bbox.max.Y) / 2.0,
            (bbox.min.Z + bbox.max.Z) / 2.0,
        ],
    }
