"""Minimal exterior revision on top of the proven Physical Mount v1 chain.

Only four mechanical areas are changed:
* the body-owned head shell/display/sensor package,
* a shallow rounded chest front panel,
* two complete claw-link replacements that retain the horn datum, and
* two 9 mm replaceable soles.

All dimensions are millimetres in the owning URDF link frame.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from build123d import (
    Align,
    Box,
    Color,
    Compound,
    Location,
    Plane,
    Shape,
    Solid,
    Sphere,
)


ROOT = Path(__file__).resolve().parents[2]
BASE_URDF = (
    ROOT
    / "generated"
    / "urdf"
    / "physical_mount_v1"
    / "zeroth01_physical_mount_v1.urdf"
)
BODY_LINK = "Z_BOT2_MASTER_BODY_SKELETON"

WHITE = Color("#F7F8FA")
BLACK = Color("#101820")
CYAN = Color("#52D6FF")
CAMERA_GREEN = Color("#2E7D32")
TOF_PURPLE = Color("#AA00FF")
DISPLAY_CYAN = Color("#00B8D9")
IMU_GREEN = Color("#64DD17")
COMPUTE_ORANGE = Color("#FF9100")
BATTERY_MAGENTA = Color("#D500F9")
SOLE_DARK = Color("#252A30")

SEAM_GAP_MM = 0.45
HEAD_Z_SHIFT_MM = -55.0


def _head_z(value: float) -> float:
    return value + HEAD_Z_SHIFT_MM

# Measured from the released v1 link meshes.  The local Y direction is
# mirrored between the two hands/feet; local +Z is medial for both feet.
LINK_BOUNDS_MM: dict[str, tuple[float, float, float, float, float, float]] = {
    "FINGER_1": (-16.9364, 9.9748, -66.2847, 9.9674, -1.5207, 39.1193),
    "FINGER_1_2": (-16.9364, 9.9748, -9.9674, 66.2847, -1.5207, 39.1193),
    "FOOT": (-61.0, 29.0, -9.9, 35.6, -1.5, 38.9),
    "FOOT_2": (-61.0, 29.0, -35.6, 9.9, -1.5, 38.9),
}


@dataclass(frozen=True)
class PartRecord:
    key: str
    shape: Shape
    installed_link: str
    color_hex: str
    classification: str
    printable: bool = True
    material: str = "PETG_WHITE"
    urdf_collision: bool = False
    replaces_baseline_visual: bool = False


def rounded_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    radius: float,
) -> Shape:
    maximum = max(0.5, min(size) / 2.0 - 0.25)
    base = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    solid = Solid(base.wrapped)
    result = solid.fillet(min(radius, maximum), solid.edges())
    return result.moved(Location(center))


def _shape_from_list(items) -> Shape:
    shapes = list(items)
    if not shapes:
        raise RuntimeError("Boolean operation returned no geometry")
    return shapes[0] if len(shapes) == 1 else Compound(shapes)


def largest_solid(shape: Shape) -> Shape:
    solids = list(shape.solids())
    if not solids:
        raise RuntimeError("shape has no solid")
    return max(solids, key=lambda item: float(item.volume))


def _split_front(shape: Shape, split_y: float) -> Shape:
    clip = Box(
        500.0,
        250.0,
        500.0,
        align=(Align.CENTER, Align.MAX, Align.CENTER),
    ).moved(Location((0.0, split_y - SEAM_GAP_MM / 2.0, 0.0)))
    return _shape_from_list(shape.intersect(clip))


def _chest_panel() -> Shape:
    """Shallow front skin only; the v1 torso sides/back remain unchanged."""

    # A compact seven-millimetre front skin.  Its 150 mm width stays inside
    # the source shoulder service volumes, so it needs no large side reliefs
    # and retains a clean rounded-rectangle silhouette.
    center = (0.0, -18.0, -14.0)
    outer = rounded_box((150.0, 48.0, 112.0), center, 22.0)
    inner = Box(
        138.0,
        40.0,
        120.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((0.0, -15.0, -14.0)))
    shell = outer.cut(inner)
    # Keep only the front skin and a short return: y=-42..-33 mm.  The
    # resulting installed depth is about 9 mm, rather than half of the
    # construction volume.
    result = largest_solid(_split_front(shell, -33.0))
    # Nest the head directly into the chest without overlapping printable
    # solids.  The 0.6 mm radial allowance is the assembly seam; visually it
    # reads as zero neck while remaining printable and removable.
    head_clearance = Sphere(1.0).scale((78.6, 56.6, 65.6)).moved(
        Location((0.0, 7.0, _head_z(126.0)))
    )
    result = largest_solid(result.cut(head_clearance))
    result.label = "PHYSICAL_MOUNT_V2_MINIMAL_ROUNDED_CHEST_FRONT_PANEL"
    result.color = WHITE
    return result


def _head_base_shell() -> Shape:
    center = (0.0, 7.0, _head_z(126.0))
    outer = Sphere(1.0).scale((78.0, 56.0, 65.0)).moved(Location(center))
    inner = Sphere(1.0).scale((73.2, 51.2, 60.2)).moved(Location(center))
    shell = outer.cut(inner)
    # Only open the 4.8 mm bottom wall around the retained source head post.
    # The earlier 102 x 52 x 42 mm notch removed far too much of the lower
    # shell and visually exposed the post as a long rectangular neck.
    shell = shell.cut(
        rounded_box((62.0, 46.0, 20.0), (0.0, 9.0, _head_z(56.0)), 8.0)
    )
    shell = shell.cut(
        rounded_box(
            (116.0, 32.0, 76.0),
            (0.0, -51.0, _head_z(125.0)),
            20.0,
        )
    )
    camera_plane = Plane(
        origin=(0.0, -70.0, _head_z(166.0)),
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, 1.0, 0.0),
    )
    tof_plane = Plane(
        origin=(29.0, -70.0, _head_z(166.0)),
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, 1.0, 0.0),
    )
    return shell.cut(
        Solid.make_cylinder(5.2, 40.0, camera_plane),
        Solid.make_cylinder(4.2, 40.0, tof_plane),
    )


def _head_shell(side: str) -> Shape:
    if side == "front":
        result = largest_solid(_split_front(_head_base_shell(), 7.0))
    elif side == "back":
        clip = Box(
            500.0,
            250.0,
            500.0,
            align=(Align.CENTER, Align.MIN, Align.CENTER),
        ).moved(Location((0.0, 7.0 + SEAM_GAP_MM / 2.0, 0.0)))
        result = largest_solid(_shape_from_list(_head_base_shell().intersect(clip)))
        for x_pos in (-56.0, 56.0):
            ear = Sphere(1.0).scale((14.0, 11.0, 16.0)).moved(
                Location((x_pos, 18.0, _head_z(181.0)))
            )
            result = largest_solid(result.fuse(ear))
    else:
        raise ValueError(side)
    result.label = f"PHYSICAL_MOUNT_V2_MINIMAL_ROUNDED_HEAD_{side.upper()}"
    result.color = WHITE
    return result


def _visor() -> Shape:
    panel = Box(118.0, 4.0, 78.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    depth_edges = [edge for edge in panel.edges() if abs(edge.length - 4.0) <= 1e-6]
    shape = Solid(panel.wrapped).fillet(21.0, depth_edges).moved(
        Location((0.0, -52.0, _head_z(125.0)))
    )
    for x_pos, radius in ((0.0, 4.5), (29.0, 3.6)):
        plane = Plane(
            origin=(x_pos, -60.0, _head_z(166.0)),
            x_dir=(1.0, 0.0, 0.0),
            z_dir=(0.0, 1.0, 0.0),
        )
        shape = shape.cut(Solid.make_cylinder(radius, 16.0, plane))
    shape.label = "PHYSICAL_MOUNT_V2_MINIMAL_CONTINUOUS_BLACK_VISOR"
    shape.color = BLACK
    return shape


def _face_ui() -> Shape:
    eyes: list[Shape] = []
    for x_pos in (-24.0, 24.0):
        eye = Sphere(1.0).scale((9.0, 0.8, 13.0)).moved(
            Location((x_pos, -55.0, _head_z(125.0)))
        )
        eye.color = CYAN
        eyes.append(eye)
    result = Compound(label="PHYSICAL_MOUNT_V2_MINIMAL_DISPLAY_UI", children=eyes)
    result.color = CYAN
    return result


def _camera_window() -> Shape:
    camera_plane = Plane(
        origin=(0.0, -62.5, _head_z(166.0)),
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, 1.0, 0.0),
    )
    outer = Solid.make_cylinder(6.2, 2.4, camera_plane)
    inner = Solid.make_cylinder(4.2, 2.8, camera_plane)
    ring = outer.cut(inner)
    lens = Solid.make_cylinder(4.1, 2.2, camera_plane)
    tof_plane = Plane(
        origin=(29.0, -62.5, _head_z(166.0)),
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, 1.0, 0.0),
    )
    tof = Solid.make_cylinder(3.5, 2.4, tof_plane)
    ring.color = CYAN
    lens.color = BLACK
    tof.color = TOF_PURPLE
    return Compound(
        label="PHYSICAL_MOUNT_V2_MINIMAL_VISIBLE_CAMERA_TOF_WINDOWS",
        children=[ring, lens, tof],
    )


def _camera_bracket() -> Shape:
    back = rounded_box((38.0, 4.0, 24.0), (0.0, -25.0, _head_z(162.0)), 4.0)
    rails = [
        rounded_box((5.0, 18.0, 20.0), (x_pos, -33.0, _head_z(162.0)), 2.0)
        for x_pos in (-16.0, 16.0)
    ]
    bridge = rounded_box((38.0, 17.0, 5.0), (0.0, -32.0, _head_z(151.0)), 2.0)
    shape = largest_solid(back.fuse(*rails, bridge))
    for x_pos in (-14.0, 14.0):
        for z_pos in (_head_z(154.0), _head_z(170.0)):
            plane = Plane(
                origin=(x_pos, -45.0, z_pos),
                x_dir=(1.0, 0.0, 0.0),
                z_dir=(0.0, 1.0, 0.0),
            )
            shape = shape.cut(Solid.make_cylinder(1.4, 30.0, plane))
    shape.label = "PHYSICAL_MOUNT_V2_MINIMAL_CAMERA_M2P5_BRACKET"
    shape.color = WHITE
    return shape


def _module_box(
    size: tuple[float, float, float],
    center: tuple[float, float, float],
    radius: float,
    label: str,
    color: Color,
) -> Shape:
    shape = rounded_box(size, center, radius)
    shape.label = label
    shape.color = color
    return shape


def _q_hand(link: str, side: str) -> Shape:
    """Replace the complete U-claw while retaining its proximal twin-ear datum."""

    if link not in {"FINGER_1", "FINGER_1_2"}:
        raise ValueError(link)
    direction = -1.0 if link == "FINGER_1" else 1.0
    center_x = -3.4808
    center_z = 18.7993

    # In the source link the actual wrist datum is the open pair of tabs at
    # local Y=0; the four-hole round face is at the distal claw end.  Retain
    # the measured 26.9 x 40.6 mm root envelope as a U-saddle and use a
    # transverse M3 shoulder fastener through the unchanged fixed-joint
    # origin.  This is the physical load path; there is no cosmetic overlay.
    palm_center_y = direction * 28.0
    palm = rounded_box(
        (42.0, 42.0, 38.0),
        (center_x, palm_center_y, center_z),
        17.0,
    )
    ear_center_y = direction * 2.0
    bottom_ear = rounded_box(
        (26.8, 22.0, 6.0),
        (center_x, ear_center_y, 1.5),
        2.5,
    )
    top_ear = rounded_box(
        (26.8, 22.0, 6.0),
        (center_x, ear_center_y, 36.1),
        2.5,
    )
    shape = largest_solid(palm.fuse(bottom_ear, top_ear))
    wrist_gap = rounded_box(
        (21.8, 26.0, 28.6),
        (center_x, 0.0, center_z),
        5.0,
    )
    shape = largest_solid(shape.cut(wrist_gap))
    fastener_plane = Plane(
        origin=(center_x, 0.0, -4.0),
        x_dir=(1.0, 0.0, 0.0),
        z_dir=(0.0, 0.0, 1.0),
    )
    shape = largest_solid(
        shape.cut(Solid.make_cylinder(1.7, 46.0, fastener_plane))
    )
    # A small, mostly embedded thumb bump gives a Q-style silhouette without
    # the previous 62x82x54 mm oversized mitten.
    thumb = Sphere(1.0).scale((5.0, 7.0, 6.0)).moved(
        Location((center_x - 19.0, palm_center_y, center_z - 4.0))
    )
    shape = largest_solid(shape.fuse(thumb))
    shape.label = f"PHYSICAL_MOUNT_V2_MINIMAL_{side.upper()}_Q_HAND_TWIN_EAR_WRIST_MOUNT"
    shape.color = WHITE
    return shape


def _sole(link: str, side: str) -> Shape:
    x0, x1, y0, y1, z0, z1 = LINK_BOUNDS_MM[link]
    if link == "FOOT":
        sole_y0, sole_y1 = y1, y1 + 9.0
    elif link == "FOOT_2":
        sole_y0, sole_y1 = y0 - 9.0, y0
    else:
        raise ValueError(link)
    # Local +Z is the medial side on both mirrored feet.  Keep the sole
    # inside that edge by 4 mm so hip yaw/roll cannot sweep the two soles
    # into each other.  Add only a modest 6 mm lateral extension; fore/aft
    # stability still comes from the 10 mm extension at each end.
    sole_z0 = z0 - 6.0
    sole_z1 = z1 - 4.0
    shape = rounded_box(
        (110.0, 9.0, sole_z1 - sole_z0),
        (
            (x0 + x1) / 2.0,
            (sole_y0 + sole_y1) / 2.0,
            (sole_z0 + sole_z1) / 2.0,
        ),
        4.0,
    )
    shape.label = f"PHYSICAL_MOUNT_V2_MINIMAL_{side.upper()}_9MM_REPLACEABLE_SOLE"
    shape.color = SOLE_DARK
    return shape


def part_records() -> dict[str, PartRecord]:
    records: dict[str, PartRecord] = {}

    def add(
        key: str,
        shape: Shape,
        link: str,
        color_hex: str,
        classification: str,
        *,
        printable: bool = True,
        material: str = "PETG_WHITE",
        collision: bool = False,
        replacement: bool = False,
    ) -> None:
        if key in records:
            raise KeyError(key)
        records[key] = PartRecord(
            key,
            shape,
            link,
            color_hex,
            classification,
            printable,
            material,
            collision,
            replacement,
        )

    add("chest_panel", _chest_panel(), BODY_LINK, "#F7F8FA", "shallow_rounded_chest_front_panel", collision=True)
    add("head_front", _head_shell("front"), BODY_LINK, "#F7F8FA", "rounded_head_front_clamshell", collision=True)
    add("head_back", _head_shell("back"), BODY_LINK, "#F7F8FA", "rounded_head_back_clamshell_with_ears", collision=True)
    add("visor", _visor(), BODY_LINK, "#101820", "display_front_panel", material="BLACK_PETG_OR_DISPLAY")
    add("face_ui", _face_ui(), BODY_LINK, "#52D6FF", "screen_ui_reference", printable=False, material="DISPLAY_PIXELS")
    add("camera_window", _camera_window(), BODY_LINK, "#52D6FF", "visible_camera_and_tof_windows", printable=False, material="OPTICAL_WINDOW_REFERENCE")
    add("camera_bracket", _camera_bracket(), BODY_LINK, "#F7F8FA", "camera_m2p5_removable_bracket")

    modules = (
        ("display_module", (105.5, 8.0, 67.2), (0.0, -43.0, _head_z(125.0)), 2.0, "WAVESHARE_4_3IN_DSI_QLED_ENVELOPE", DISPLAY_CYAN, "#00B8D9"),
        ("camera_module", (25.0, 11.4, 23.862), (0.0, -33.0, _head_z(160.0)), 2.0, "RPI_CAMERA_MODULE_3_WIDE_ENVELOPE", CAMERA_GREEN, "#2E7D32"),
        ("tof_module", (12.0, 3.0, 10.0), (29.0, -38.0, _head_z(163.0)), 1.0, "VL53L5CX_CARRIER_ENVELOPE", TOF_PURPLE, "#AA00FF"),
        ("compute_module", (105.0, 20.0, 70.0), (0.0, 42.0, -2.0), 4.0, "TORSO_COMPUTE_REGULATOR_ENVELOPE", COMPUTE_ORANGE, "#FF9100"),
        ("battery_pack", (75.0, 38.0, 38.0), (0.0, 28.0, -52.0), 6.0, "3S2P_BATTERY_BMS_ENVELOPE", BATTERY_MAGENTA, "#D500F9"),
        ("torso_imu", (32.0, 25.0, 8.0), (0.0, 12.0, 18.0), 2.0, "TORSO_IMU_ENVELOPE", IMU_GREEN, "#64DD17"),
    )
    for key, size, center, radius, label, color, color_hex in modules:
        add(
            key,
            _module_box(size, center, radius, label, color),
            BODY_LINK,
            color_hex,
            "internal_payload_controlled_envelope",
            printable=False,
            material="PURCHASED_OR_SELECTED_ELECTRONICS",
        )

    add(
        "left_q_hand",
        _q_hand("FINGER_1", "left"),
        "FINGER_1",
        "#F7F8FA",
        "complete_claw_link_replacement_twin_ear_m3_crossbolt",
        collision=True,
        replacement=True,
    )
    add(
        "right_q_hand",
        _q_hand("FINGER_1_2", "right"),
        "FINGER_1_2",
        "#F7F8FA",
        "complete_claw_link_replacement_twin_ear_m3_crossbolt",
        collision=True,
        replacement=True,
    )
    add(
        "left_sole",
        _sole("FOOT", "left"),
        "FOOT",
        "#252A30",
        "replaceable_9mm_thickened_contact_sole",
        material="TPU_OR_PETG_CONTACT_PROTOTYPE",
        collision=True,
    )
    add(
        "right_sole",
        _sole("FOOT_2", "right"),
        "FOOT_2",
        "#252A30",
        "replaceable_9mm_thickened_contact_sole",
        material="TPU_OR_PETG_CONTACT_PROTOTYPE",
        collision=True,
    )
    return records


def manifest_payload() -> dict[str, object]:
    rows: list[dict[str, object]] = []
    for record in part_records().values():
        bounds = record.shape.bounding_box()
        rows.append(
            {
                "key": record.key,
                "label": record.shape.label,
                "installed_link": record.installed_link,
                "color_hex": record.color_hex,
                "classification": record.classification,
                "printable": record.printable,
                "material": record.material,
                "urdf_collision": record.urdf_collision,
                "replaces_baseline_visual": record.replaces_baseline_visual,
                "solid_count": len(record.shape.solids()),
                "volume_mm3": float(record.shape.volume),
                "bbox_size_mm": [bounds.size.X, bounds.size.Y, bounds.size.Z],
            }
        )
    return {
        "schema": "zeroth01.physical_mount_v2_minimal.component_manifest.v1",
        "source_mechanism": BASE_URDF.relative_to(ROOT).as_posix(),
        "change_policy": "head + claw-link replacements + sole + chest-front only",
        "unchanged_revolute_joint_count": 16,
        "parts": rows,
    }
