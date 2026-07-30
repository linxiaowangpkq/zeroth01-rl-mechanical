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
QUARANTINED_SERVO_STEP = (
    ROOT
    / "source_assets"
    / "vendor"
    / "sts3250"
    / "FEETECH_STS3250.step"
)
SERVO_STEP = QUARANTINED_SERVO_STEP
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
SERVO_INTERFACE_CONFIG = (
    ROOT / "config" / "round_v2_servo_interface_geometry.json"
)
COMPONENT_IDENTITY_CONFIG = (
    ROOT / "config" / "round_v2_component_identity.json"
)
DUAL_EYE_STEP = (
    ROOT
    / "source_assets"
    / "vendor"
    / "head_electronics"
    / "Waveshare_DualEye_LCD_Module.step"
)
CAMERA_WIDE_STEP = (
    ROOT
    / "source_assets"
    / "vendor"
    / "head_electronics"
    / "Raspberry_Pi_Camera_Module_3_Wide.step"
)

CREAM = Color("#F7F8FA")
TAN = Color("#D7DBE2")
DARK = Color("#101820")
TEAL = Color("#52D6FF")
SERVO_METAL = Color("#1677FF")
SERVO_BLUE = Color("#1677FF")

WALL_MM = 2.4
SEAM_GAP_MM = 0.35
SOLE_THICKENING_MM = 8.0


def servo_interface_config() -> dict[str, object]:
    return json.loads(SERVO_INTERFACE_CONFIG.read_text(encoding="utf-8"))


def servo_identity_map() -> dict[str, dict[str, object]]:
    payload = json.loads(
        COMPONENT_IDENTITY_CONFIG.read_text(encoding="utf-8")
    )
    return {
        str(item["joint"]): item for item in payload["servos"]
    }


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
    # Poppy Eva head topology, rebuilt parametrically around Zeroth-01 rather
    # than scaling its STL: a removable white egg shell, one continuous black
    # screen aperture, and a separate internal display carrier.  The larger
    # upper volume houses a 4.3-inch Waveshare display plus camera/ToF while
    # the tapered underside preserves the frozen shoulder mechanism.
    center = (0.0, -3.0, 108.0)
    outer = Sphere(1.0).scale((78.0, 50.0, 60.0)).moved(Location(center))
    inner = Sphere(1.0).scale((74.8, 46.8, 56.8)).moved(Location(center))
    # Two small solid ear tabs make the silhouette friendlier without
    # widening the existing shoulder envelope.  Keeping these low-volume
    # cosmetic tabs solid avoids a fragile shell-within-shell intersection at
    # the front/back seam and gives the printed halves a reliable watertight
    # union.  They remain part of the removable head shell.
    shell = outer.cut(inner)
    for x_pos in (-54.0, 54.0):
        ear_center = (x_pos, center[1], 154.0)
        shell = shell.fuse(Sphere(14.0).moved(Location(ear_center)))
    # Large continuous visor: no ears, muzzle, exposed eyeballs, or protruding
    # sensor blocks remain in the exterior design.
    visor_opening = Box(
        112.0,
        60.0,
        70.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    visor_depth_edges = [
        edge
        for edge in visor_opening.edges()
        if abs(edge.length - 60.0) <= 1e-6
    ]
    visor_opening = visor_opening.fillet(
        18.0, visor_depth_edges
    ).moved(Location((0.0, -45.0, 105.0)))
    shell = shell.cut(visor_opening)
    # Underside keep-outs are centered on the frozen shoulder-pitch axes.
    for x_pos in (-78.15, 78.15):
        shell = shell.cut(
            Sphere(32.0).moved(Location((x_pos, -0.34, 43.05)))
        )
    shell = _add_seam_bosses(
        shell,
        center[1],
        [(-66.0, 76.0), (66.0, 76.0), (-60.0, 139.0), (60.0, 139.0)],
    )
    piece = _half(shell, center[1], side)
    piece.label = f"ROUND_V3_POPPY_EVA_DERIVED_HEAD_{side.upper()}"
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


def arm_sleeve(side: str, segment: str) -> Shape:
    """Printable, open-ended cosmetic sleeve in the source link frame."""

    if side not in {"left", "right"}:
        raise ValueError(side)
    if segment == "upper":
        center = (
            (15.0, -32.0, -19.7)
            if side == "left"
            else (-15.0, 32.0, 19.7)
        )
        # The upstream shoulder mesh spans about 58 x 41 mm in this
        # cross-section.  A 64 x 49 mm rounded keep-out leaves at least
        # 3-4 mm nominal clearance before the printable wall.
        outer_size = (70.0, 52.0, 57.0)
        inner_size = (64.0, 64.0, 49.0)
        outer_radius = 15.0
        inner_radius = 12.0
    elif segment == "forearm":
        # Both mirrored hand-link meshes share the same local envelope.
        center = (0.0, 27.0, 18.4)
        outer_size = (46.0, 52.0, 60.0)
        inner_size = (34.0, 64.0, 54.0)
        outer_radius = 13.0
        inner_radius = 8.0
    else:
        raise ValueError(segment)
    shape = rounded_box(outer_size, center, outer_radius).cut(
        rounded_box(inner_size, center, inner_radius)
    )
    shape.label = (
        f"ROUND_V3_{side.upper()}_{segment.upper()}_ARM_PRINTED_SLEEVE"
    )
    shape.color = CREAM
    return shape


def chibi_hand(side: str) -> Shape:
    """A printable rounded mitten/palm cap; no articulated fingers."""

    if side not in {"left", "right"}:
        raise ValueError(side)
    center = (0.0, 77.0, 18.4)
    outer = rounded_box((52.0, 46.0, 60.0), center, 14.0)
    # A subtle solid thumb bump: local +X is inward on the right hand and
    # local -X is inward on the left hand in the frozen neutral frames.
    thumb_sign = -1.0 if side == "left" else 1.0
    thumb = Sphere(1.0).scale((8.0, 10.0, 8.0)).moved(
        Location((thumb_sign * 23.0, 76.0, 15.0))
    )
    outer = outer.fuse(thumb)
    # The cavity opens through the proximal end and captures the existing
    # aggregate hand-link tip.  Its 34 x 54 mm cross-section provides a
    # conservative 3-4 mm keep-out around the frozen 26 x 48 mm link while a
    # 6 mm distal wall keeps the fixed mitten robust.
    cavity = rounded_box((34.0, 48.0, 54.0), (0.0, 70.0, 18.4), 9.0)
    shape = outer.cut(cavity)
    shape.label = f"ROUND_V3_{side.upper()}_CHIBI_PRINTED_HAND"
    shape.color = CREAM
    return shape


def face_graphics() -> Shape:
    """Flat screen UI reference, not printed or mass-bearing geometry."""

    eyes: list[Shape] = []
    for x_pos in (-24.0, 24.0):
        # A filled, shallow screen pixel avoids the deep Boolean eye rings
        # that produced spoke-like reflections in SOLIDWORKS.  It reads as a
        # friendly display graphic rather than a protruding mechanical eye.
        eye = Sphere(1.0).scale((8.0, 0.55, 11.5)).moved(
            Location((x_pos, -40.45, 109.0))
        )
        eye.label = (
            f"ROUND_V3_SCREEN_EYE_"
            f"{'LEFT' if x_pos < 0 else 'RIGHT'}"
        )
        eye.color = TEAL
        eyes.append(eye)
    shape = Compound(label="ROUND_V3_NONPHYSICAL_SCREEN_UI", children=eyes)
    shape.color = TEAL
    return shape


def muzzle_badge() -> Shape:
    # Backward-compatible source entry point.  The old bear muzzle is gone.
    return face_graphics()


def visor_badge() -> Shape:
    # A manufacturable recessed screen carrier: round only the four edges
    # parallel to Y so the 20 mm face radius does not violate the 4 mm depth.
    # The head aperture crops this plate into a continuous curved-shell visor
    # silhouette while the plate itself remains a robust printable solid.
    panel = Box(
        120.0,
        4.0,
        78.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    )
    depth_edges = [
        edge for edge in panel.edges() if abs(edge.length - 4.0) <= 1e-6
    ]
    shape = panel.fillet(20.0, depth_edges).moved(
        Location((0.0, -38.0, 105.0))
    )
    shape.label = "ROUND_V3_CONTINUOUS_BLACK_SCREEN_VISOR"
    shape.color = DARK
    return shape


def camera_lenses() -> Shape:
    lenses: list[Shape] = []
    camera_plane = Plane(
        origin=(0.0, -39.5, 151.0),
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, -1.0, 0.0),
    )
    camera_ring = Solid.make_cylinder(4.5, 1.5, camera_plane).cut(
        Solid.make_cylinder(2.7, 2.0, camera_plane)
    )
    camera_ring.label = "RPI_CAMERA_MODULE_3_WIDE_WINDOW"
    camera_ring.color = TEAL
    lenses.append(camera_ring)
    # Keep the edge radius below half of the 1.5 mm optical-window depth.
    # A 1.5 mm radius on every edge is geometrically impossible here.
    tof_window = rounded_box((8.0, 1.5, 5.0), (26.0, -39.0, 151.0), 0.6)
    tof_window.label = "VL53L5CX_TOF_WINDOW"
    tof_window.color = Color("#AA00FF")
    lenses.append(tof_window)
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


def _recenter_step(shape: Shape) -> Shape:
    bbox = shape.bounding_box()
    center = (
        (bbox.min.X + bbox.max.X) / 2.0,
        (bbox.min.Y + bbox.max.Y) / 2.0,
        (bbox.min.Z + bbox.max.Z) / 2.0,
    )
    return shape.moved(Location(tuple(-value for value in center)))


def eye_display_module() -> Shape:
    # step.parts has no exact 4.3-inch DSI/QLED module.  Use the documented
    # envelope selected in round_v1_electronics_layout_source.json; the
    # Poppy Eva source STL remains the open mounting-topology reference.
    shape = _electronics_module_box("eye_display_module", 2.0)
    shape.label = "WAVESHARE_4_3IN_DSI_QLED_CONTROLLED_ENVELOPE"
    shape.color = Color("#00B8D9")
    return shape


def camera_module() -> Shape:
    # The official reference STEP contains 631 solids and stalls SolidWorks
    # import without improving package-clearance evidence.  Use its measured
    # 25 x 23.862 x 11.4 mm overall envelope in the working assembly; retain
    # the untouched vendor STEP under source_assets/vendor/head_electronics.
    shape = _electronics_module_box("camera_module", 2.0)
    shape.label = "RASPBERRY_PI_CAMERA_MODULE_3_WIDE_VENDOR_ENVELOPE"
    shape.color = Color("#FF1744")
    return shape


def tof_module() -> Shape:
    shape = _electronics_module_box("tof_module", 1.0)
    shape.label = "VL53L5CX_CUSTOM_CARRIER_ENVELOPE"
    shape.color = Color("#AA00FF")
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


def sts3250_controlled_case() -> Shape:
    """Dimension-controlled STS3250-C001 case in the canonical shaft frame.

    The previously downloaded STEP identifies itself as ST-3235M and its
    shaft is +Y, while the assembly treated +Z as the shaft. It remains
    quarantined for provenance only. This controlled reference instead uses
    the current FEETECH 45.22 x 24.72 x 35 mm case and the drawing's 12.5 mm
    shaft-center offset. Purchased splines and horns are child-side parts.
    """

    controlled = servo_interface_config()["controlled_servo_reference"]
    bounds_min = [float(value) for value in controlled["case_bounds_min_xyz"]]
    bounds_max = [float(value) for value in controlled["case_bounds_max_xyz"]]
    size = tuple(
        bounds_max[index] - bounds_min[index] for index in range(3)
    )
    center = tuple(
        (bounds_max[index] + bounds_min[index]) / 2.0
        for index in range(3)
    )
    case = rounded_box(size, center, 2.0)
    # Four drawing-visible M2 mounting locations make the review part
    # recognisable while keeping the body envelope dimension-controlled.
    for x_pos in (-28.5, 8.3):
        for y_pos in (-9.5, 9.5):
            case = case.cut(
                Cylinder(1.1, 40.0).moved(
                    Location((x_pos, y_pos, -20.0))
                )
            )

    # Shallow face bosses show the two-sided output location without claiming
    # an unverified internal bearing/gear tooth profile.
    # The drawing's complete two-sided depth is 36.5 mm.  Keep the 35 mm
    # case body exact and constrain both review bosses to +/-18.25 mm.
    front_boss = Cylinder(8.0, 1.5).moved(Location((0.0, 0.0, 17.5)))
    rear_boss = Cylinder(8.0, 1.5).moved(Location((0.0, 0.0, -17.5)))
    shape = case.fuse(front_boss, rear_boss)
    shape.label = "FEETECH_STS3250_C001_BLUE_DIAGNOSTIC_ENVELOPE"
    shape.color = SERVO_BLUE
    return shape


def _purchased_horn(
    center_z: float,
    thickness: float,
    label: str,
) -> Shape:
    interface = servo_interface_config()
    horn = interface["purchased_horns"]
    radius = float(horn["outer_diameter"]) / 2.0
    shape = Cylinder(radius, thickness).moved(Location((0.0, 0.0, center_z)))
    spline_radius = float(horn["center_spline_envelope_diameter"]) / 2.0
    shape = shape.cut(
        Cylinder(spline_radius, thickness + 1.0).moved(
            Location((0.0, 0.0, center_z))
        )
    )
    bolt_radius = float(horn["bolt_circle_diameter"]) / 2.0
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle = math.radians(angle_deg)
        shape = shape.cut(
            Cylinder(1.6, thickness + 1.0).moved(
                Location(
                    (
                        bolt_radius * math.cos(angle),
                        bolt_radius * math.sin(angle),
                        center_z,
                    )
                )
            )
        )
    shape.label = label
    shape.color = SERVO_METAL
    return shape


def purchased_horn_pair() -> Shape:
    horn = servo_interface_config()["purchased_horns"]
    front = _purchased_horn(
        float(horn["front_center_z"]),
        float(horn["front_disc_thickness"]),
        "PURCHASED_STS3250_25T_FRONT_HORN_4XM3_PCD14",
    )
    rear = _purchased_horn(
        float(horn["rear_center_z"]),
        float(horn["rear_disc_thickness"]),
        "PURCHASED_STS3250_25T_REAR_HORN_4XM3_PCD14",
    )
    return Compound(
        label="PURCHASED_STS3250_25T_HORN_PAIR",
        children=[front, rear],
    )


def servo_cage() -> Shape:
    """Parent-side CNC carrier around the exact STS3250 local envelope.

    Local +Z is the servo output axis.  The front opening keeps the purchased
    25T horn serviceable; the rear ring supports the second shaft.
    """

    cage_config = servo_interface_config()["cage"]
    center = tuple(
        float(value) for value in cage_config["outer_box_center_xyz"]
    )
    outer = rounded_box(
        tuple(float(value) for value in cage_config["outer_box_size_xyz"]),
        center,
        4.0,
    )
    inner = rounded_box(
        tuple(float(value) for value in cage_config["inner_box_size_xyz"]),
        tuple(float(value) for value in cage_config["inner_box_center_xyz"]),
        3.0,
    )
    cage = outer.cut(inner)
    service_opening = Box(
        18.0,
        24.0,
        29.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((13.0, 0.0, 0.0)))
    cage = cage.cut(service_opening)

    rear_center = float(cage_config["rear_guard_center_z"])
    front_center = float(cage_config["front_guard_center_z"])
    rear_outer = Cylinder(
        float(cage_config["rear_guard_outer_radius"]), 3.0
    ).moved(Location((0.0, 0.0, rear_center)))
    rear_inner = Cylinder(
        float(cage_config["rear_guard_inner_radius"]), 4.0
    ).moved(Location((0.0, 0.0, rear_center)))
    rear_seat = rear_outer.cut(rear_inner)
    front_outer = Cylinder(
        float(cage_config["front_guard_outer_radius"]), 3.0
    ).moved(Location((0.0, 0.0, front_center)))
    front_inner = Cylinder(
        float(cage_config["front_guard_inner_radius"]), 4.0
    ).moved(Location((0.0, 0.0, front_center)))
    front_guard = front_outer.cut(front_inner)
    anchor = rounded_box(
        tuple(float(value) for value in cage_config["anchor_size_xyz"]),
        tuple(float(value) for value in cage_config["anchor_center_xyz"]),
        3.0,
    )
    shape = cage.fuse(rear_seat, front_guard, anchor)
    shape.label = "ROUND_V1_STS3250_PARENT_SERVO_CAGE"
    shape.color = CREAM
    return shape


def _output_adapter_disc(center_z: float) -> Shape:
    """Owned adapter matching the purchased horn's drawing-defined M3 PCD."""

    adapter = servo_interface_config()["child_output_adapters"]
    thickness = float(adapter["disc_thickness"])
    disc = Cylinder(
        float(adapter["disc_outer_radius"]), thickness
    ).moved(Location((0.0, 0.0, center_z)))
    disc = disc.cut(
        Cylinder(3.1, thickness + 1.0).moved(
            Location((0.0, 0.0, center_z))
        )
    )
    bolt_radius = float(adapter["bolt_circle_diameter"]) / 2.0
    clearance_radius = float(adapter["bolt_clearance_diameter"]) / 2.0
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle = math.radians(angle_deg)
        x_pos = bolt_radius * math.cos(angle)
        y_pos = bolt_radius * math.sin(angle)
        disc = disc.cut(
            Cylinder(clearance_radius, thickness + 1.0).moved(
                Location((x_pos, y_pos, center_z))
            )
        )
    return disc


def output_hub_front() -> Shape:
    center_z = float(
        servo_interface_config()["child_output_adapters"]["front_center_z"]
    )
    shape = _output_adapter_disc(center_z)
    shape.label = "ROUND_V1_STS3250_CHILD_OUTPUT_ADAPTER_FRONT"
    shape.color = TEAL
    return shape


def output_hub_rear() -> Shape:
    center_z = float(
        servo_interface_config()["child_output_adapters"]["rear_center_z"]
    )
    shape = _output_adapter_disc(center_z)
    shape.label = "ROUND_V1_STS3250_CHILD_OUTPUT_ADAPTER_REAR"
    shape.color = TEAL
    return shape


def output_hub() -> Shape:
    """Purchased 25T horn pair plus owned PCD14 child adapters.

    The supplied drawing resolves the previous unknown interface: both horns
    expose four M3 holes on a 14 mm bolt circle. The 25T tooth form remains a
    purchased feature and is deliberately represented only by its envelope.
    """

    interface = servo_interface_config()
    controlled = interface["controlled_servo_reference"]
    front_spline = controlled["front_spline"]
    rear_spline = controlled["rear_spline"]
    front_shaft = Cylinder(
        float(front_spline["diameter"]) / 2.0,
        float(front_spline["axial_length"]),
    ).moved(Location((0.0, 0.0, float(front_spline["axial_center_z"]))))
    rear_shaft = Cylinder(
        float(rear_spline["diameter"]) / 2.0,
        float(rear_spline["axial_length"]),
    ).moved(Location((0.0, 0.0, float(rear_spline["axial_center_z"]))))
    shape = Compound(
        label="ROUND_V2_STS3250_CHILD_OUTPUT_STACK",
        children=[
            front_shaft,
            rear_shaft,
            *list(purchased_horn_pair().children),
            output_hub_front(),
            output_hub_rear(),
        ],
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
    # Review-only marker: deliberately smaller than a horn or cage so the
    # colored annotation cannot be mistaken for physical transmission CAD.
    outer = Cylinder(10.0, 1.8)
    inner = Cylinder(7.2, 2.2)
    ring = outer.cut(inner)
    ring.label = "ROUND_V2_NONPHYSICAL_JOINT_POSITION_MARKER"
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


def joint_marker_instances() -> tuple[list[Shape], list[dict[str, object]]]:
    """Place 16 colored markers without altering the assembled mechanism."""

    joints, transforms = load_neutral_kinematics()
    moving = _moving_joint_map(joints)
    identities = servo_identity_map()
    source = joint_ring()
    markers: list[Shape] = []
    rows: list[dict[str, object]] = []
    for name, joint in moving.items():
        identity = identities[name]
        servo_id = str(identity["id"])
        frame_rotation, translation = _joint_frame(joint, transforms)
        marker = copy.copy(source).moved(
            location_from_transform((frame_rotation, translation))
        )
        marker.label = f"{servo_id}_JOINT_POSITION_{name}"
        marker.color = Color(str(identity["color_hex"]))
        markers.append(marker)
        axis_world = _mat_vec(
            frame_rotation,
            [float(value) for value in joint["axis"]],
        )
        rows.append(
            {
                "joint": name,
                "servo_id": servo_id,
                "color_hex": identity["color_hex"],
                "candidate_actuator": "Feetech STS3250",
                "marker_semantics": (
                    "NONPHYSICAL_POSITION_MARKER; frozen assembled Zeroth-01 "
                    "link geometry remains authoritative"
                ),
                "shaft_xyz_world_mm": " ".join(
                    f"{value * 1000.0:.6f}" for value in translation
                ),
                "joint_positive_axis_world": " ".join(
                    f"{value:.9f}" for value in axis_world
                ),
                "gate": "PASS",
            }
        )
    return markers, rows


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
    source = sts3250_controlled_case()
    identities = servo_identity_map()
    instances: list[Shape] = []
    rows: list[dict[str, object]] = []
    phase_config: dict[str, dict[str, object]] = {}
    if SERVO_PHASE_CONFIG.is_file():
        phase_config = json.loads(
            SERVO_PHASE_CONFIG.read_text(encoding="utf-8")
        ).get("joint_mount_phase", {})

    for name, joint in moving.items():
        identity = identities[name]
        servo_id = str(identity["id"])
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
        instance.label = f"{servo_id}_STS3250_C001_BLUE_REVIEW_{name}"
        instance.color = SERVO_BLUE
        instances.append(instance)
        rows.append(
            {
                "joint": name,
                "servo_id": servo_id,
                "id_color_hex": identity["color_hex"],
                "visual_color_hex": "#1677FF",
                "servo_model": "Feetech STS3250-C001",
                "geometry_source": (
                    "dimension-controlled STS3250-C001 from current official "
                    "product size and supplied drawing"
                ),
                "quarantined_step": (
                    QUARANTINED_SERVO_STEP.relative_to(ROOT).as_posix()
                ),
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
                    "DIAGNOSTIC_HOUSING_ONLY_FOLLOWS_PARENT_JOINT_FRAME; "
                    "NOT_A_PHYSICAL_REPLACEMENT_OR_EXTRA_URDF_BODY"
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
    identities = servo_identity_map()
    cages: list[Shape] = []
    hubs: list[Shape] = []
    for name, joint in moving.items():
        identity = identities[name]
        servo_id = str(identity["id"])
        identity_color = Color(str(identity["color_hex"]))
        frame_rotation, translation = _joint_frame(joint, transforms)
        housing_rotation, _, _ = _servo_mount_rotation(
            joint,
            frame_rotation,
            phase_config,
        )
        cage = copy.copy(cage_source).moved(
            location_from_transform((housing_rotation, translation))
        )
        cage.label = f"{servo_id}_STS3250_PARENT_CAGE_{name}"
        cage.color = identity_color
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
        hub.label = f"{servo_id}_STS3250_CHILD_OUTPUT_STACK_{name}"
        hub.color = identity_color
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
    write_servo_axis_report(servo_rows)
    parts: list[Shape] = [
        chest_shell("front"),
        chest_shell("back"),
        head_shell("front"),
        head_shell("back"),
        pelvis_shell("front"),
        pelvis_shell("back"),
        face_graphics(),
        visor_badge(),
        camera_lenses(),
        torso_spine(),
        eye_display_module(),
        camera_module(),
        tof_module(),
        imu_module(),
        compute_module(),
        battery_pack(),
    ]
    for side, link_name in (("left", "foot_left"), ("right", "foot_right")):
        placed = copy.copy(sole(side)).moved(location_from_transform(transforms[link_name]))
        placed.label = f"ROUND_V1_{side.upper()}_THICK_SOLE"
        parts.append(placed)
    moving_cosmetics = [
        (
            arm_sleeve("right", "upper"),
            "right_shoulder_yaw_motor",
        ),
        (
            arm_sleeve("left", "upper"),
            "left_shoulder_yaw_motor",
        ),
        (arm_sleeve("right", "forearm"), "Left_Hand"),
        (arm_sleeve("left", "forearm"), "hand_right"),
        (chibi_hand("right"), "Left_Hand"),
        (chibi_hand("left"), "hand_right"),
    ]
    for local_shape, link_name in moving_cosmetics:
        placed = copy.copy(local_shape).moved(
            location_from_transform(transforms[link_name])
        )
        placed.label = local_shape.label
        placed.color = CREAM
        parts.append(placed)
    parts.extend(servo_parts)
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
