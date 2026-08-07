"""STEP-first compact v3.1 manufacturing/review geometry.

Individual parts are the manufacturing handoff.  ``gen_step`` returns a
labelled diagnostic assembly that makes all 18 actuator axes and electronics
locations visible; it is not a substitute for the v2 source-carrier B-Reps.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

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
    export_step,
    export_stl,
    import_step,
)
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed"
PARTS = OUT / "parts"
REPORT = ROOT / "reports" / "physical_mount_v3_rl_fixed" / "cad_build.json"
V2_PARTS = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
V1_BODY_STEP = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v1"
    / "step"
    / "skeleton"
    / "Z_BOT2_MASTER_BODY_SKELETON.step"
)

WHITE = Color("#F7F8FA")
BLUE = Color("#1677FF")
BLACK = Color("#101820")
ORANGE = Color("#FF9100")
MAGENTA = Color("#D500F9")
GREEN = Color("#64DD17")
CYAN = Color("#00B8D9")
PURPLE = Color("#AA00FF")

CASE_X_MIN_MM = -32.72
CASE_X_MAX_MM = 12.50
CASE_Y_MM = 24.72
CASE_Z_MM = 35.00
CASE_CENTER_X_MM = (CASE_X_MIN_MM + CASE_X_MAX_MM) / 2.0
SERVO_ORIGIN_Z_SHIFT_MM = -20.90
M2_X_MM = (-28.50, 8.30)
M2_Y_MM = (-10.25, 10.25)
CORES3_TORSO_HOLE_Y_MM = (-18.0, 18.0)
CORES3_TORSO_HOLE_Z_MM = (-6.0, 6.0)
ANKLE_ROLL_COMPACT_OFFSET_MM = 30.0
BODY_TOP_LIMIT_MM = 45.0
BODY_LOCAL_TO_WORLD_Z_DEG = math.degrees(
    math.atan2(0.9999996829318346, 0.0007963267107332633)
)
CORES3_BODY_CLEARANCE_MM = 0.30
CORES3_CRADLE_CLEARANCE_MM = 0.20
STS3250_BODY_CLEARANCE_MM = 0.30


def rounded_box(size, center=(0.0, 0.0, 0.0), radius=2.0) -> Shape:
    base = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    solid = Solid(base.wrapped)
    maximum = max(0.25, min(size) / 2.0 - 0.2)
    try:
        result = solid.fillet(min(radius, maximum), solid.edges())
    except Exception:
        result = solid
    return result.moved(Location(center))


def cut_full_product_tree(shape: Shape, *tools: Shape) -> Shape:
    """Cut every solid in a legacy STEP compound, preserving one component."""

    current = shape.wrapped
    for tool in tools:
        operation = BRepAlgoAPI_Cut(current, tool.wrapped)
        operation.Build()
        if not operation.IsDone():
            raise RuntimeError("OCCT failed to cut a v3 body installation feature")
        current = operation.Shape()
    return Compound(current)


def world_tool_to_body_local(shape: Shape) -> Shape:
    """Map a world-authored installation cutter into the legacy body frame."""

    return shape.moved(Location((0.0, 0.0, 0.0), (0.0, 0.0, -BODY_LOCAL_TO_WORLD_Z_DEG)))


def controlled_sts3250() -> Shape:
    case = rounded_box((45.22, CASE_Y_MM, CASE_Z_MM), (CASE_CENTER_X_MM, 0.0, 0.0), 2.0)
    for x_pos in M2_X_MM:
        for y_pos in M2_Y_MM:
            case = case.cut(
                Cylinder(
                    0.80,
                    CASE_Z_MM + 2.0,
                    align=(Align.CENTER, Align.CENTER, Align.CENTER),
                ).moved(Location((x_pos, y_pos, 0.0)))
            )
    front = Cylinder(8.0, 0.75, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location((0.0, 0.0, 17.875))
    )
    rear = Cylinder(8.0, 0.75, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location((0.0, 0.0, -17.875))
    )
    spline_front = Cylinder(2.95, 3.4, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location((0.0, 0.0, 19.2))
    )
    spline_rear = Cylinder(3.025, 3.1, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location((0.0, 0.0, -19.05))
    )
    # Place the joint frame at the outer output-spline plane, matching the
    # released installed-servo meshes. The previous centred Z origin displaced
    # every case by about 20.9 mm into its carrier.
    result = case.fuse(front, rear, spline_front, spline_rear).moved(
        Location((0.0, 0.0, SERVO_ORIGIN_Z_SHIFT_MM))
    )
    result.label = "FEETECH_STS3250_C001_DIMENSION_CONTROLLED_REFERENCE"
    result.color = BLUE
    return result


def first_article_gauge() -> Shape:
    plate = rounded_box((49.22, 28.72, 3.0), (CASE_CENTER_X_MM, 0.0, 0.0), 2.5)
    plate = plate.cut(Cylinder(10.25, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)))
    for x_pos in M2_X_MM:
        for y_pos in M2_Y_MM:
            plate = plate.cut(
                Cylinder(1.10, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                    Location((x_pos, y_pos, 0.0))
                )
            )
    plate.label = "STS3250_4XM2_25T_FIRST_ARTICLE_GAUGE"
    return plate


def ankle_carrier(side: str) -> Shape:
    if side not in {"left", "right"}:
        raise ValueError(side)
    cage_z = -20.75
    outer = rounded_box((52.0, 28.0, 43.0), (CASE_CENTER_X_MM, 0.0, cage_z), 3.0)
    inner = rounded_box((46.42, 25.92, 42.0), (CASE_CENTER_X_MM, 0.0, cage_z), 2.0)
    cage = outer.cut(inner)
    # Open the horn side; the opposite wall remains the rear support face.
    cage = cage.cut(
        Box(50.0, 40.0, 18.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((CASE_CENTER_X_MM, 0.0, -4.5))
        )
    )
    # With the roll servos mirrored, local +X is outboard and local +Y is
    # world up on both sides. This short bridge
    # reaches the released ankle-pitch output while the 45.22 mm case axis
    # remains horizontal.
    anchor = rounded_box(
        (3.0, ANKLE_ROLL_COMPACT_OFFSET_MM, 20.0),
        (
            14.5,
            ANKLE_ROLL_COMPACT_OFFSET_MM / 2.0,
            0.0,
        ),
        1.2,
    )
    cage = cage.fuse(anchor)
    # The PCD14 horn/foot adapter is a rotating child. Cut radial clearance
    # through the parent carrier instead of accepting coincident overlap.
    cage = cage.cut(
        Cylinder(14.2, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    )
    for z_pos in (-6.0, 6.0):
        hole = Cylinder(
            1.7,
            6.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(
            Location(
                (
                    14.5,
                    ANKLE_ROLL_COMPACT_OFFSET_MM - 3.0,
                    z_pos,
                ),
                (0.0, 90.0, 0.0),
            )
        )
        cage = cage.cut(hole)
    # Keep the part authored in the STS3250 local frame. The assembly manifest
    # applies the same proper rotation to servo, cage and horn on each side.
    cage.label = f"{side.upper()}_STS3250_ANKLE_ROLL_PARENT_CARRIER_4XM2"
    cage.color = WHITE
    return cage


def trimmed_body_skeleton() -> Shape:
    """Trim the old head plate and machine the real v3 installation pockets.

    The released v1 body STEP is a multibody carrier with a few intentional
    same-component union overlaps. Preserve that product structure here; the
    assembly gate classifies those internal overlaps separately from true
    cross-component interference. The following cuts are physical
    installation features, not display-only suppression: a flush CoreS3
    pocket, the hidden U-cradle recess/bolt passages, and 0.30 mm case
    clearance around the two hip-yaw STS3250 servos. Servo joint frames and
    all retained v1 interfaces stay unchanged.
    """

    source = import_step(V1_BODY_STEP)
    keep = Box(
        400.0,
        400.0,
        600.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((0.0, 0.0, BODY_TOP_LIMIT_MM - 300.0)))
    common = BRepAlgoAPI_Common(source.wrapped, keep.wrapped)
    common.Build()
    if not common.IsDone():
        raise RuntimeError("failed to trim source body skeleton at Z=45 mm")
    result = Compound(common.Shape())

    # Official CoreS3 K128 main-unit envelope: 54 x 54 x 15.5 mm.  Add only
    # 0.30 mm diametral/axial assembly clearance so the purchased module is
    # flush-sunk into the torso instead of intersecting or becoming a neck.
    cores3_pocket = rounded_box(
        (
            15.5 + 2.0 * CORES3_BODY_CLEARANCE_MM,
            54.0 + 2.0 * CORES3_BODY_CLEARANCE_MM,
            54.0 + 2.0 * CORES3_BODY_CLEARANCE_MM,
        ),
        (38.75, 0.0, 18.0),
        3.0 + CORES3_BODY_CLEARANCE_MM,
    )

    # Recess the reversible 2 mm aluminium U-cradle with 0.20 mm clearance.
    # Matching M3 passages continue through the remaining torso wall so the
    # cradle is retained by a real bolt/nut-plate stack rather than floating.
    cradle_clearance = rounded_box(
        (2.0 + 2.0 * CORES3_CRADLE_CLEARANCE_MM, 50.4, 22.4),
        (30.0, 0.0, 18.0),
        1.0,
    ).fuse(
        rounded_box((7.9, 2.4, 22.4), (34.75, -28.0, 18.0), 0.8),
        rounded_box((7.9, 2.4, 22.4), (34.75, 28.0, 18.0), 0.8),
        rounded_box((7.9, 50.4, 2.4), (34.75, 0.0, -10.0), 0.8),
    )
    result = cut_full_product_tree(
        result,
        world_tool_to_body_local(cores3_pocket),
        world_tool_to_body_local(cradle_clearance),
    )
    for y_pos in CORES3_TORSO_HOLE_Y_MM:
        for z_pos in CORES3_TORSO_HOLE_Z_MM:
            result = cut_full_product_tree(
                result,
                world_tool_to_body_local(
                    Cylinder(
                        1.7,
                        40.0,
                        align=(Align.CENTER, Align.CENTER, Align.CENTER),
                    ).moved(
                        Location((18.0, y_pos, 18.0 + z_pos), (0.0, 90.0, 0.0))
                    )
                ),
            )

    # The two hip-yaw cases keep the released v2 shaft frames.  A bounded
    # rectangular machining envelope clears the full purchased case/spline
    # extent while preserving all material outside a 0.30 mm service gap.
    hip_clearance_local = rounded_box(
        (
            45.22 + 2.0 * STS3250_BODY_CLEARANCE_MM,
            CASE_Y_MM + 2.0 * STS3250_BODY_CLEARANCE_MM,
            42.10 + 2.0 * STS3250_BODY_CLEARANCE_MM,
        ),
        (CASE_CENTER_X_MM, 0.0, -20.75),
        2.0 + STS3250_BODY_CLEARANCE_MM,
    )
    result = cut_full_product_tree(
        result,
        world_tool_to_body_local(
            hip_clearance_local.moved(
                Location((-7.700908, 42.814146, -83.418), (0.0, 0.0, 90.0))
            )
        ),
        world_tool_to_body_local(
            hip_clearance_local.moved(
                Location((-7.769134, -42.861827, -83.418), (0.0, 0.0, -90.0))
            )
        ),
    )

    result.label = "Z_BOT2_MASTER_BODY_SKELETON_V3_INSTALL_POCKETS_Z45"
    result.color = WHITE
    return result


def horn_adapter(side: str) -> Shape:
    disc = Cylinder(13.5, 3.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    disc = disc.cut(Cylinder(3.1, 4.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)))
    for index in range(4):
        angle = index * 90.0
        x_pos = 7.0 * __import__("math").cos(__import__("math").radians(angle))
        y_pos = 7.0 * __import__("math").sin(__import__("math").radians(angle))
        disc = disc.cut(
            Cylinder(1.7, 4.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((x_pos, y_pos, 0.0))
            )
        )
    disc.label = f"{side.upper()}_PURCHASED_HORN_TO_FOOT_ADAPTER_PCD14"
    disc.color = WHITE
    return disc


def hip_yaw_inboard_adapter(side: str) -> Shape:
    """Reversible plate moving each hip shaft 5.838 mm toward Y=0."""

    if side not in {"left", "right"}:
        raise ValueError(side)
    offset = 5.838
    plate = rounded_box((55.0, 32.0, 3.0), (0.0, 0.0, 0.0), 3.0)
    # Source-carrier and new-servo patterns remain separate; elongated slots
    # make the offset measurable and reversible without modifying v2 carriers.
    for x_base in (-18.4, 18.4):
        for y_pos in (-10.25, 10.25):
            source_hole = Cylinder(1.7, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((x_base, y_pos, 0.0))
            )
            servo_x = x_base + (-offset if side == "left" else offset)
            servo_hole = Cylinder(1.15, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((servo_x, y_pos, 0.0))
            )
            plate = plate.cut(source_hole, servo_hole)
    plate.label = f"{side.upper()}_HIP_YAW_INBOARD_ADAPTER_5P838MM"
    plate.color = WHITE
    return plate


def lightweight_sole(side: str) -> Shape:
    if side == "left":
        y_contact = 44.6
        sign = 1.0
    elif side == "right":
        y_contact = -44.6
        sign = -1.0
    else:
        raise ValueError(side)
    center_x = -16.0
    center_z = 13.7
    skin_y = y_contact - sign * 1.0
    # 2 mm ground skin + 5 mm structural rib stack = 7 mm total. This remains
    # thicker than the source foot plate while recovering height margin.
    rib_y = y_contact - sign * 4.5
    skin = rounded_box((110.0, 2.0, 42.4), (center_x, skin_y, center_z), 4.0)
    long_rails = [
        rounded_box((102.0, 5.0, 4.0), (center_x, rib_y, z), 1.5)
        for z in (-5.0, 32.4)
    ]
    end_rails = [
        rounded_box((4.0, 5.0, 34.4), (x, rib_y, center_z), 1.5)
        for x in (-67.0, 35.0)
    ]
    cross_x = rounded_box((7.0, 5.0, 34.4), (center_x, rib_y, center_z), 1.5)
    cross_z = rounded_box((96.0, 5.0, 5.0), (center_x, rib_y, center_z), 1.5)
    result = skin.fuse(*long_rails, *end_rails, cross_x, cross_z)
    result.label = f"{side.upper()}_7MM_LIGHTWEIGHT_REPLACEABLE_SOLE_RING_RIB"
    result.color = BLACK
    return result


def cores3_purchased_envelope() -> Shape:
    """Official-size CoreS3 main-unit envelope, not a printable replacement."""

    body = rounded_box((15.5, 54.0, 54.0), (0.0, 0.0, 0.0), 3.0)
    body.label = "M5STACK_CORES3_K128_PURCHASED_MAIN_UNIT_ENVELOPE"
    body.color = WHITE
    return body


def cores3_face_glass_reference() -> Shape:
    glass = rounded_box((1.0, 48.0, 48.0), (8.25, 0.0, 0.0), 3.0)
    glass.label = "CORES3_2INCH_TOUCH_FACE_GLASS"
    glass.color = BLACK
    return glass


def cores3_camera_reference() -> Shape:
    camera = Cylinder(2.6, 1.5, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location((8.9, 0.0, -20.0), (0.0, 90.0, 0.0))
    )
    camera.label = "CORES3_GC0308_CAMERA_WINDOW"
    camera.color = CYAN
    return camera


def cores3_expression_reference() -> Shape:
    eyes = []
    for y_pos in (-10.5, 10.5):
        eye = Cylinder(4.2, 1.4, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((8.95, y_pos, 3.0), (0.0, 90.0, 0.0))
        )
        eye.color = CYAN
        eye.label = "CORES3_SCREEN_EXPRESSION_EYE"
        eyes.append(eye)
    return Compound(
        label="CORES3_PURCHASED_SCREEN_EXPRESSION_REFERENCE",
        children=eyes,
    )


def cores3_internal_torso_cradle() -> Shape:
    """Compact 2 mm 6061 U-cradle hidden behind the purchased CoreS3."""

    plate = rounded_box((2.0, 50.0, 22.0), (0.0, 0.0, 0.0), 1.0)
    # Side and bottom retention lips touch the CoreS3 perimeter without
    # entering its controlled 54 x 54 x 15.5 mm envelope.
    plate = plate.fuse(
        rounded_box((7.5, 2.0, 22.0), (4.75, -28.0, 0.0), 0.8),
        rounded_box((7.5, 2.0, 22.0), (4.75, 28.0, 0.0), 0.8),
        rounded_box((7.5, 50.0, 2.0), (4.75, 0.0, -28.0), 0.8),
    )
    # Four M3 clearance holes are a controlled first-article interface to the
    # torso-side nut plate; CoreS3 itself is retained by the U-cradle.
    for y_pos in CORES3_TORSO_HOLE_Y_MM:
        for z_pos in CORES3_TORSO_HOLE_Z_MM:
            plate = plate.cut(
                Cylinder(1.7, 4.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                    Location((0.0, y_pos, z_pos), (0.0, 90.0, 0.0))
                )
            )
    plate.label = "CORES3_INTERNAL_2MM_6061_U_CRADLE"
    plate.color = Color("#BFC7D1")
    return plate


def _load_urdf_module():
    path = Path(__file__).with_name("build_v3_urdf.py")
    spec = importlib.util.spec_from_file_location("zeroth_v3_urdf", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _axis_location(origin_mm, axis) -> Location:
    z_dir = tuple(float(value) for value in axis)
    x_dir = (1.0, 0.0, 0.0) if abs(z_dir[0]) < 0.9 else (0.0, 1.0, 0.0)
    return Plane(origin=origin_mm, x_dir=x_dir, z_dir=z_dir).location


def _rod_between(first, second, radius=7.0) -> Shape | None:
    import math

    delta = tuple(second[i] - first[i] for i in range(3))
    length = math.sqrt(sum(value * value for value in delta))
    if length < 2.0:
        return None
    direction = tuple(value / length for value in delta)
    x_dir = (1.0, 0.0, 0.0) if abs(direction[0]) < 0.9 else (0.0, 1.0, 0.0)
    rod = Solid.make_cylinder(
        radius,
        length,
        Plane(origin=first, x_dir=x_dir, z_dir=direction),
    )
    rod.color = WHITE
    return rod


def gen_step() -> Shape:
    module = _load_urdf_module()
    old_robot = __import__("xml.etree.ElementTree", fromlist=["ElementTree"]).parse(module.V2_URDF).getroot()
    old_tf = module.old_fk(old_robot)
    neutral_tf = module.neutral_transforms(old_tf)
    pos_m = {name: transform[1] for name, transform in neutral_tf.items()}
    pos = {name: tuple(value * 1000.0 for value in xyz) for name, xyz in pos_m.items()}

    children: list[Shape] = []
    # v2 printable parts are authored with their face normal along -Y.  The
    # RL/world convention is X-forward, Y-left, Z-up, so rotate only those
    # raw cosmetic assets +90 deg about Z.  Joint/servo positions below are
    # already expressed in the RL/world convention and must not be rotated.
    raw_to_world = Location((0.0, 0.0, 0.0), (0.0, 0.0, 90.0))
    torso = rounded_box((76.0, 155.0, 176.0), (0.0, 0.0, -25.0), 20.0)
    torso.label = "V3_LIGHTWEIGHT_TORSO_ENVELOPE"
    torso.color = WHITE
    children.append(torso)
    # The v2 cosmetic chest panel is deliberately absent. Its upper edge
    # conflicts with the purchased CoreS3 and blocks the USB-C service
    # volume; the original load-bearing torso remains below.

    # This bounded STEP is the light diagnostic envelope.  The full assembly
    # with released carrier B-Reps is external-part based and is written by
    # build_v3_assembly_manifest.py / the SolidWorks generator.
    for _, parent, child, _, _ in module.JOINT_SPECS:
        rod = _rod_between(pos[parent], pos[child], 6.0 if "shoulder" in child.lower() else 7.0)
        if rod is not None:
            rod.label = f"DIAGNOSTIC_AXIS_LINK_{parent}_TO_{child}"
            children.append(rod)

    servo = controlled_sts3250()
    servo_id_by_joint = {
        "right_shoulder_pitch": "S01", "left_shoulder_pitch": "S02",
        "right_shoulder_yaw": "S03", "right_hip_pitch": "S04",
        "left_hip_pitch": "S05", "left_shoulder_yaw": "S06",
        "right_hip_yaw": "S07", "left_hip_yaw": "S08",
        "right_elbow_yaw": "S09", "left_elbow_yaw": "S10",
        "right_hip_roll": "S11", "left_hip_roll": "S12",
        "right_knee_pitch": "S13", "left_knee_pitch": "S14",
        "right_ankle_pitch": "S15", "left_ankle_pitch": "S16",
        "right_ankle_roll": "S17", "left_ankle_roll": "S18",
    }
    for name, _, child, axis, _ in module.JOINT_SPECS:
        instance = servo.moved(_axis_location(pos[child], axis))
        instance.label = f"{servo_id_by_joint[name]}_STS3250_{name}"
        instance.color = BLUE
        children.append(instance)

    for side, link, foot, axis in (
        ("left", module.LEFT_ANKLE_CARRIER, "FOOT", (1.0, 0.0, 0.0)),
        ("right", module.RIGHT_ANKLE_CARRIER, "FOOT_2", (-1.0, 0.0, 0.0)),
    ):
        carrier = ankle_carrier(side).moved(_axis_location(pos[foot], axis))
        carrier.label = f"{side.upper()}_ANKLE_ROLL_CARRIER_INSTALLED"
        children.append(carrier)
    electronics = (
        ((0.0, 42.0, -2.0), (105.0, 20.0, 70.0), ORANGE, "COMPUTE"),
        ((0.0, 28.0, -52.0), (75.0, 38.0, 38.0), MAGENTA, "BATTERY"),
        ((0.0, 12.0, 18.0), (32.0, 25.0, 8.0), GREEN, "IMU"),
    )
    for center, size, color, name in electronics:
        world_center = (-center[1], center[0], center[2])
        world_size = (size[1], size[0], size[2])
        shape = rounded_box(world_size, world_center, 2.0)
        shape.label = f"{name}_CONTROLLED_INSTALL_ENVELOPE"
        shape.color = color
        children.append(shape)

    adapter_center = tuple(value * 1000.0 for value in module.CORES3_ADAPTER_CENTER_M)
    head_center = tuple(value * 1000.0 for value in module.CORES3_HEAD_CENTER_M)
    adapter = cores3_internal_torso_cradle().moved(Location(adapter_center))
    adapter.label = "CORES3_INTERNAL_TORSO_CRADLE_INSTALLED"
    children.append(adapter)
    purchased_head = cores3_purchased_envelope().moved(Location(head_center))
    purchased_head.label = "PURCHASED_M5STACK_CORES3_K128_HEAD_INSTALLED"
    children.append(purchased_head)
    for feature in (
        cores3_face_glass_reference(),
        cores3_camera_reference(),
        cores3_expression_reference(),
    ):
        children.append(feature.moved(Location(head_center)))

    assembly = Compound(label="ZEROTH01_V3_RL_FIXED_18DOF_DIAGNOSTIC_ASSEMBLY", children=children)
    return assembly


def main() -> int:
    PARTS.mkdir(parents=True, exist_ok=True)
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    parts = {
        "sts3250_dimension_controlled": controlled_sts3250(),
        "sts3250_first_article_gauge": first_article_gauge(),
        "left_ankle_roll_carrier": ankle_carrier("left"),
        "right_ankle_roll_carrier": ankle_carrier("right"),
        "left_ankle_roll_horn_adapter": horn_adapter("left"),
        "right_ankle_roll_horn_adapter": horn_adapter("right"),
        "left_sole_lightweighted": lightweight_sole("left"),
        "right_sole_lightweighted": lightweight_sole("right"),
        "m5stack_cores3_k128_purchased_envelope": cores3_purchased_envelope(),
        "m5stack_cores3_face_glass_reference": cores3_face_glass_reference(),
        "m5stack_cores3_camera_reference": cores3_camera_reference(),
        "m5stack_cores3_expression_reference": cores3_expression_reference(),
        "cores3_internal_torso_cradle_2mm_6061": cores3_internal_torso_cradle(),
        "body_skeleton_top_trimmed_45mm": trimmed_body_skeleton(),
    }
    rows = []
    for name, shape in parts.items():
        step = PARTS / f"{name}.step"
        stl = PARTS / f"{name}.stl"
        export_step(shape, step)
        export_stl(shape, stl, tolerance=0.03, angular_tolerance=0.12)
        bounds = shape.bounding_box().size
        rows.append(
            {
                "name": name,
                "step": step.relative_to(ROOT).as_posix(),
                "stl": stl.relative_to(ROOT).as_posix(),
                "valid_brep": bool(shape.is_valid),
                "solid_count": len(shape.solids()),
                "volume_mm3": float(shape.volume),
                "bbox_mm": [float(bounds.X), float(bounds.Y), float(bounds.Z)],
            }
        )
    assembly = gen_step()
    assembly_path = OUT / "ZEROTH01_V3_RL_FIXED_18DOF_DIAGNOSTIC_ASSEMBLY.step"
    export_step(assembly, assembly_path)
    payload = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.cad_build.v1",
        "cad_brief": "v2 load-bearing assembly with the obsolete upper head plate trimmed at Z=45 mm and controlled CoreS3/cradle/hip-servo installation pockets + 18 dimension-controlled blue actuator references + bilateral 30 mm non-interfering ankle-roll hardware + lightened thickened 7 mm soles + purchased M5Stack CoreS3 interaction head flush-sunk into the upper torso on a hidden 2mm aluminium cradle",
        "assembly": assembly_path.relative_to(ROOT).as_posix(),
        "parts": rows,
        "truth_boundary": "diagnostic assembly; CoreS3 is an official-size 54 x 54 x 15.5 mm purchased-module envelope backed by the official K128 product page/structure STL, not a printable substitute; STS3250 and torso-cradle first articles remain physical sign-off items",
    }
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(assembly_path)
    print(REPORT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
