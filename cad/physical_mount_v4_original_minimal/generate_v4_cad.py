"""STEP-first Zeroth-01 v4 original-minimal manufacturing geometry.

The official v1 STL carriers remain the geometric baseline.  Only the head
cover is replaced, at the measured source seam, while service mounts and the
v3 ankle-roll correction are additive/reversible parts.
"""

from __future__ import annotations

import json
import math
import shutil
import struct
from collections import defaultdict
from pathlib import Path

import numpy as np
from build123d import (
    Align,
    Box,
    Color,
    Compound,
    Cylinder,
    Location,
    Shape,
    Solid,
    export_step,
    export_stl,
    import_step,
    import_stl,
)
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
OUT = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal"
PARTS = OUT / "parts"
REPORT = ROOT / "reports" / "v4_original_minimal" / "cad_build.json"
V1_STL = ROOT / "generated" / "cad" / "physical_mount_v1" / "skeleton"
STS3250_STEP_PARTS = ROOT / "source_assets" / "step_parts" / "feetech_sts3250.step"
V1_STEP = (
    ROOT.parents[1]
    / "reference"
    / "zeroth01"
    / "generated"
    / "cad"
    / "physical_mount_v1"
    / "step"
    / "skeleton"
)

WHITE = Color("#F7F8FA")
BLACK = Color("#101820")
CYAN = Color("#00B8D9")
ORANGE = Color("#FF9100")
MAGENTA = Color("#D500F9")
GREEN = Color("#64DD17")
GREY = Color("#BFC7D1")
BLUE = Color("#1677FF")

# Measured directly from connected component 11 of the released v1 torso STL.
SOURCE_HEAD_MIN_MM = (-40.3749756515, -6.3102724962, 7.4818544090)
SOURCE_HEAD_MAX_MM = (40.3750278056, 24.6897283942, 69.0412372351)
SOURCE_HEAD_CENTER_MM = tuple(
    (SOURCE_HEAD_MIN_MM[index] + SOURCE_HEAD_MAX_MM[index]) / 2.0
    for index in range(3)
)
SOURCE_HEAD_SIZE_MM = tuple(
    SOURCE_HEAD_MAX_MM[index] - SOURCE_HEAD_MIN_MM[index]
    for index in range(3)
)
HEAD_SEAM_Z_MM = SOURCE_HEAD_MIN_MM[2]
HEAD_OUTER_SIZE_MM = (
    SOURCE_HEAD_SIZE_MM[0] + 10.0,
    SOURCE_HEAD_SIZE_MM[1],
    SOURCE_HEAD_SIZE_MM[2] + 10.0,
)
HEAD_WALL_MM = 2.2
HEAD_RADIUS_MM = 5.0
HEAD_SPLIT_Y_MM = SOURCE_HEAD_CENTER_MM[1]

# Purchased M5Stack UnitV2 official envelope and mass.  The released product
# includes a GC2145 1080p camera and one microphone.
UNITV2_SIZE_MM = (48.0, 18.5, 24.0)
UNITV2_CENTER_MM = (0.0, 6.0, 45.0)
UNITV2_MASS_KG = 0.018
CAMERA_CENTER_MM = (-14.0, SOURCE_HEAD_MIN_MM[1] - 1.2, 45.0)
MICROPHONE_CENTER_MM = (14.0, SOURCE_HEAD_MIN_MM[1] - 1.2, 45.0)

# Compact remote-drive ankle selected by exact B-Rep screening.  Coordinates
# are authored in the ankle-roll frame: +X outboard, +Y up, +Z along the roll
# axis.  The physical STS3250 stays clear of its neighbouring pitch servo;
# equal 16T pulleys return the foot pivot to the original Zeroth-01 height.
REMOTE_SERVO_CENTER_MM = (15.0, -25.0)
REMOTE_BELT_CENTER_DISTANCE_MM = math.hypot(*REMOTE_SERVO_CENTER_MM)
GT2_PITCH_MM = 2.0
GT2_TOOTH_COUNT = 16
GT2_PITCH_RADIUS_MM = GT2_TOOTH_COUNT * GT2_PITCH_MM / (2.0 * math.pi)
GT2_BELT_LENGTH_MM = 90.0
STS_CASE_CENTER_X_MM = (-32.72 + 12.50) / 2.0
STS_M2_X_MM = (-28.50, 8.30)
STS_M2_Y_MM = (-10.25, 10.25)
# Parts in this file are authored in the released BODY mesh frame.  That frame
# rotates about +89.95 deg into the RL world: local +X is world left and local
# +Y is world rear.  Boards span local X and the thin service-pod depth spans Y.
SERVICE_POD_CENTER_Y_MM = 84.0
COMPUTE_SIZE_MM = (70.0, 12.0, 32.0)
COMPUTE_CENTER_MM = (0.0, SERVICE_POD_CENTER_Y_MM, -18.0)
BATTERY_SIZE_MM = (75.0, 22.0, 34.0)
BATTERY_CENTER_MM = (0.0, SERVICE_POD_CENTER_Y_MM, -63.0)
IMU_CENTER_MM = (0.0, 94.0, -18.0)
SHIN_SHORTEN_MM = 18.0
ANKLE_ROLL_DIRECT_OFFSET_MM = 26.5
BODY_HEAD_INTERFACE_TRIM_MM = 2.5
SHOULDER_SERVO_HEAD_CLEARANCE_MM = 0.8
OBSOLETE_GENERATED_PART_STEMS = {
    "body_original_head_interface_trimmed_2mm",
    "body_original_minus_head",
    "left_original_hand",
    "right_original_hand",
    "left_small_light_palm",
    "right_small_light_palm",
    "rear_service_pod",
    "hip_yaw_horn_spacer_2mm",
    "remote_ankle_2x688_bearings",
    "remote_ankle_equal_16t_pulleys",
    "remote_ankle_foot_horn_adapter",
    "remote_ankle_front_plate",
    "remote_ankle_gt2_90mm_belt_reference",
    "remote_ankle_output_shaft",
    "remote_ankle_rear_plate",
    "remote_ankle_spacers",
    "sts3250_pcd14_child_standoff_8p9mm",
    "sts3250_pcd14_child_standoff_3mm",
    "sts3250_pcd14_child_standoff_13mm",
    "sts3250_pcd14_child_standoff_1p95mm",
}
# The step.parts model uses its vendor frame: output axis +Y, shaft line at
# X=-2.89/Z=0.  The released Zeroth installation frame uses output axis +Z
# and places the child-joint datum 2.05 mm beyond the supplied output face.
# This proper rotation/translation preserves the exact 45.22 x 24.72 x 37.40
# purchased geometry while keeping every released joint axis unchanged.
STS3250_NATIVE_TO_SHAFT_FRAME = (
    (1.0, 0.0, 0.0, 2.89),
    (0.0, 0.0, -1.0, 0.0),
    (0.0, 1.0, 0.0, -11.25),
)
STS3250_OUTPUT_FACE_GAP_MM = 2.05
HIP_SERVO_MATRICES = (
    (
        (9.551763069156241e-17, -0.99999999999994, -3.4641009687928327e-07, -7.700908393644671),
        (1.0, 0.0, 2.757357004084333e-10, 42.8141460140535),
        (-2.7573570040841675e-10, -3.4641009687928327e-07, 0.99999999999994, -83.418),
    ),
    (
        (-9.551763069156241e-17, 0.99999999999994, -3.4641009687928327e-07, -7.769134480913454),
        (-1.0, 0.0, 2.757357004084333e-10, -42.861826820814365),
        (2.7573570040841675e-10, 3.4641009687928327e-07, 0.99999999999994, -83.418),
    ),
)


def rounded_box(size, center=(0.0, 0.0, 0.0), radius=2.0) -> Shape:
    base = Box(*size, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    solid = Solid(base.wrapped)
    maximum = max(0.25, min(size) / 2.0 - 0.2)
    try:
        result = solid.fillet(min(radius, maximum), solid.edges())
    except Exception:
        result = solid
    return result.moved(Location(center))


def common(shape: Shape, tool: Shape) -> Shape:
    operation = BRepAlgoAPI_Common(shape.wrapped, tool.wrapped)
    operation.Build()
    if not operation.IsDone():
        raise RuntimeError("OCCT common operation failed")
    return Compound(operation.Shape())


def cut(shape: Shape, *tools: Shape) -> Shape:
    current = shape.wrapped
    for tool in tools:
        operation = BRepAlgoAPI_Cut(current, tool.wrapped)
        operation.Build()
        if not operation.IsDone():
            raise RuntimeError("OCCT cut operation failed")
        current = operation.Shape()
    return Compound(current)


def moved_by_matrix(shape: Shape, matrix) -> Shape:
    transform = gp_Trsf()
    transform.SetValues(
        *[float(matrix[row][column]) for row in range(3) for column in range(4)]
    )
    return shape.moved(Location(gp_trsf=transform))


def scaled_source_stl(name: str) -> Shape:
    source = import_stl(V1_STL / f"{name}.stl")
    # Released STL vertices are in metres. Tuple scaling forces a geometric
    # transform of the triangulated face instead of only changing location.
    result = source.scale((1000.0, 1000.0, 1000.0))
    result.label = f"ZEROTH01_V1_SOURCE_{name}"
    return result


def exact_sts3250_shaft_frame() -> Shape:
    """Purchased STS3250 STEP normalized to the released shaft datum."""

    if not STS3250_STEP_PARTS.is_file():
        raise FileNotFoundError(STS3250_STEP_PARTS)
    moved = moved_by_matrix(import_step(STS3250_STEP_PARTS), STS3250_NATIVE_TO_SHAFT_FRAME)
    # Flatten the vendor product tree to its 13 manufacturing solids.  OCCT's
    # STEP writer cannot re-serialize the imported nested assembly wrapper,
    # while a flat compound preserves every face and thread exactly.
    result = Compound(children=list(moved.solids()))
    result.label = "FEETECH_STS3250_STEP_PARTS_EXACT_SHAFT_FRAME"
    result.color = BLUE
    return result


def source_step(name: str) -> Shape:
    path = V1_STEP / f"{name}.step"
    if not path.is_file():
        raise FileNotFoundError(
            f"validated official-source STEP is missing: {path}; regenerate the "
            "physical_mount_v1 source conversion before v4"
        )
    result = import_step(path)
    result.label = f"ZEROTH01_V1_SOURCE_{name}"
    return result


def body_without_old_head() -> Shape:
    """Remove the complete old head at its measured source seam.

    One planar common operation is used. Additional hole-by-hole booleans are
    deliberately avoided on the legacy 6368-facet STEP; the released M3 drill
    jig and nut plate own that first-article operation instead.
    """
    cached = PARTS / "body_original_minus_head_base.step"
    if cached.is_file() and cached.stat().st_size > 1_000_000:
        print("body: reuse cached true-seam faceted STEP", flush=True)
        body = import_step(cached)
        body.label = "ZEROTH01_V1_BODY_OLD_HEAD_REMOVED_AT_TRUE_Z7P482_SEAM"
        body.color = WHITE
        base_body = body
    else:
        print("body: trim official faceted STEP once at true Z=7.482 mm seam", flush=True)
        source = source_step("Z_BOT2_MASTER_BODY_SKELETON")
        keep = Box(
            400.0,
            400.0,
            600.0,
            align=(Align.CENTER, Align.CENTER, Align.MAX),
        ).moved(Location((0.0, 0.0, HEAD_SEAM_Z_MM + 0.002)))
        base_body = common(source, keep)
        export_step(base_body, cached)

    # Keep the released faceted load-bearing body unchanged below this planar
    # head-interface trim.  Its unsewn source cannot truthfully accept detailed
    # B-Rep pocket booleans; the hip-yaw exact-servo clearance is instead owned
    # by the explicit axial shim/output stack in the assembly manifest.
    keep = Box(
        400.0,
        400.0,
        600.0,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).moved(Location((0.0, 0.0, HEAD_SEAM_Z_MM - BODY_HEAD_INTERFACE_TRIM_MM + 0.002)))
    body = common(base_body, keep)
    body.label = "ZEROTH01_V4_ORIGINAL_BODY_HEAD_INTERFACE_TRIMMED_2P5MM"
    body.color = WHITE
    return body


def head_shell() -> Shape:
    outer = rounded_box(
        HEAD_OUTER_SIZE_MM,
        SOURCE_HEAD_CENTER_MM,
        HEAD_RADIUS_MM,
    )
    inner_size = tuple(value - 2.0 * HEAD_WALL_MM for value in HEAD_OUTER_SIZE_MM)
    inner = rounded_box(inner_size, SOURCE_HEAD_CENTER_MM, HEAD_RADIUS_MM - 1.6)
    shell = outer.cut(inner)

    # Hollow clearance around the original torso-top footprint allows the
    # 5 mm lower visual expansion to overlap as a shroud without solid-body
    # collision.  The structural mount remains on the internal nut plate.
    torso_top_clearance = rounded_box(
        (82.0, 34.0, 10.0),
        (SOURCE_HEAD_CENTER_MM[0], SOURCE_HEAD_CENTER_MM[1], 6.2),
        2.0,
    )
    shell = shell.cut(torso_top_clearance)

    # Expanding the released head by 5 mm downward makes its hidden lower
    # corners overlap the two released shoulder-yaw STS3250 envelopes by only
    # 2.81 mm^3 per shell half.  Keep the requested outer silhouette and cut
    # exact B-Rep underside pockets instead of raising the head or moving the
    # proven shoulder axes.  These matrices are the two installed STS3250
    # transforms expressed in the released BODY-local frame.  A second copy
    # shifted 0.8 mm upward owns the FDM assembly clearance without turning
    # the pocket into a large cosmetic notch.
    shoulder_servo_source = import_step(
        ROOT
        / "generated"
        / "cad"
        / "physical_mount_v3_rl_fixed"
        / "parts"
        / "sts3250_dimension_controlled.step"
    )
    shoulder_servo_body_matrices = (
        (
            (1.0, 0.0, -3.46410207e-7, 55.995),
            (-3.46410207e-7, -3.26794897e-7, -1.0, -10.610),
            (0.0, 1.0, -3.26794897e-7, -9.528),
        ),
        (
            (-1.0, 0.0, 0.0, -55.995),
            (0.0, 3.26794897e-7, -1.0, -10.610),
            (0.0, -1.0, -3.26794897e-7, -9.528),
        ),
    )
    for matrix in shoulder_servo_body_matrices:
        installed_servo = moved_by_matrix(shoulder_servo_source, matrix)
        shell = shell.cut(
            installed_servo,
            installed_servo.moved(Location((0.0, 0.0, SHOULDER_SERVO_HEAD_CLEARANCE_MM))),
        )

    # Camera and acoustic passages are both normal to the source front face.
    camera_passage = Cylinder(
        7.0,
        10.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location(CAMERA_CENTER_MM, (90.0, 0.0, 0.0)))
    microphone_passage = Cylinder(
        1.4,
        10.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location(MICROPHONE_CENTER_MM, (90.0, 0.0, 0.0)))
    shell = shell.cut(camera_passage, microphone_passage)

    # Four M2.5 split-line bosses give front and rear halves a real fastener
    # stack instead of relying on a cosmetic seam.
    for x_pos in (-35.0, 35.0):
        for z_pos in (15.0, 61.0):
            boss = Cylinder(
                4.0,
                8.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Location((x_pos, HEAD_SPLIT_Y_MM, z_pos), (90.0, 0.0, 0.0)))
            hole = Cylinder(
                1.35,
                12.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Location((x_pos, HEAD_SPLIT_Y_MM, z_pos), (90.0, 0.0, 0.0)))
            shell = shell.fuse(boss).cut(hole)
    return shell


def split_head(front: bool) -> Shape:
    shell = head_shell()
    if front:
        clip = Box(
            200.0,
            100.0,
            200.0,
            align=(Align.CENTER, Align.MAX, Align.CENTER),
        ).moved(Location((0.0, HEAD_SPLIT_Y_MM, SOURCE_HEAD_CENTER_MM[2])))
        result = common(shell, clip)
        result.label = "ZEROTH01_V4_HEAD_FRONT_5MM_EACH_SIDE_SMALL_RADIUS"
    else:
        clip = Box(
            200.0,
            100.0,
            200.0,
            align=(Align.CENTER, Align.MIN, Align.CENTER),
        ).moved(Location((0.0, HEAD_SPLIT_Y_MM, SOURCE_HEAD_CENTER_MM[2])))
        result = common(shell, clip)
        # USB-C cable exit to the torso; the exit is at the rear/bottom and
        # includes 1 mm radial service clearance around an 8 x 4 mm plug.
        cable_exit = rounded_box(
            (12.0, 10.0, 7.0),
            (25.0, SOURCE_HEAD_MAX_MM[1], 14.0),
            2.0,
        )
        result = cut(result, cable_exit)
        result.label = "ZEROTH01_V4_HEAD_REAR_5MM_EACH_SIDE_SMALL_RADIUS"
    result.color = WHITE
    return result


def head_visor() -> Shape:
    visor = rounded_box(
        (76.0, 2.0, 51.0),
        (0.0, SOURCE_HEAD_MIN_MM[1] - 1.0, 45.0),
        5.0,
    )
    camera = Cylinder(7.2, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location(CAMERA_CENTER_MM, (90.0, 0.0, 0.0))
    )
    microphone = Cylinder(
        1.6,
        5.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location(MICROPHONE_CENTER_MM, (90.0, 0.0, 0.0)))
    visor = visor.cut(camera, microphone)
    visor.label = "ZEROTH01_V4_SIMPLE_BLACK_CAMERA_MIC_VISOR"
    visor.color = BLACK
    return visor


def unitv2_envelope() -> Shape:
    module = rounded_box(UNITV2_SIZE_MM, UNITV2_CENTER_MM, 2.0)
    lens = Cylinder(6.0, 3.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location(CAMERA_CENTER_MM, (90.0, 0.0, 0.0))
    )
    module = module.fuse(lens)
    module.label = "PURCHASED_M5STACK_UNITV2_48X18P5X24_CAMERA_MIC_ENVELOPE"
    module.color = CYAN
    return module


def unitv2_bracket() -> Shape:
    center_x, _, center_z = UNITV2_CENTER_MM
    rear_y = UNITV2_CENTER_MM[1] + UNITV2_SIZE_MM[1] / 2.0 + 1.0
    rear = rounded_box((52.0, 2.0, 28.0), (center_x, rear_y, center_z), 1.5)
    sides = [
        rounded_box((3.0, 19.0, 28.0), (center_x + sign * 25.5, UNITV2_CENTER_MM[1], center_z), 1.0)
        for sign in (-1.0, 1.0)
    ]
    base = rounded_box((52.0, 19.0, 3.0), (center_x, UNITV2_CENTER_MM[1], center_z - 13.5), 1.0)
    bracket = rear.fuse(*sides, base)
    for x_pos in (-20.0, 20.0):
        bracket = bracket.cut(
            Cylinder(1.35, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((x_pos, rear_y, center_z), (90.0, 0.0, 0.0))
            )
        )
    bracket.label = "ZEROTH01_V4_UNITV2_REMOVABLE_M2P5_CRADLE"
    bracket.color = GREY
    return bracket


def head_torso_nut_plate() -> Shape:
    plate = rounded_box((60.0, 22.0, 2.0), (0.0, 9.0, HEAD_SEAM_Z_MM + 1.1), 2.0)
    for x_pos in (-25.0, 25.0):
        for y_pos in (3.0, 15.0):
            plate = plate.cut(
                Cylinder(1.7, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                    Location((x_pos, y_pos, HEAD_SEAM_Z_MM + 1.1))
                )
            )
    plate.label = "ZEROTH01_V4_DIRECT_HEAD_TO_TORSO_M3_NUT_PLATE_NO_NECK"
    plate.color = GREY
    return plate


def head_mount_drill_jig() -> Shape:
    jig = rounded_box((66.0, 28.0, 3.0), (0.0, 9.0, HEAD_SEAM_Z_MM + 3.0), 2.0)
    for x_pos in (-25.0, 25.0):
        for y_pos in (3.0, 15.0):
            jig = jig.cut(
                Cylinder(1.7, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                    Location((x_pos, y_pos, HEAD_SEAM_Z_MM + 3.0))
                )
            )
    jig.label = "ZEROTH01_V4_4XM3_TORSO_TOP_DRILL_JIG"
    jig.color = GREY
    return jig


def compute_envelope() -> Shape:
    result = rounded_box(COMPUTE_SIZE_MM, COMPUTE_CENTER_MM, 2.0)
    result.label = "RASPBERRY_PI_ZERO_2W_CLASS_COMPUTE_ENVELOPE_70X12X32"
    result.color = ORANGE
    return result


def compute_tray() -> Shape:
    rear = rounded_box((76.0, 2.0, 36.0), (0.0, 92.0, -18.0), 2.0)
    side_left = rounded_box((2.0, 14.0, 36.0), (-37.0, 85.0, -18.0), 1.0)
    side_right = rounded_box((2.0, 14.0, 36.0), (37.0, 85.0, -18.0), 1.0)
    bottom = rounded_box((76.0, 14.0, 2.0), (0.0, 85.0, -35.0), 1.0)
    tray = rear.fuse(side_left, side_right, bottom)
    tray = tray.cut(rounded_box((36.0, 4.0, 29.0), (0.0, 92.0, -18.0), 1.0))
    for x_pos in (-30.0, 30.0):
        for z_pos in (-30.0, -6.0):
            tray = tray.cut(
                Cylinder(1.7, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                    Location((x_pos, 92.0, z_pos), (90.0, 0.0, 0.0))
                )
            )
    tray.label = "ZEROTH01_V4_REMOVABLE_COMPUTE_TRAY_M3"
    tray.color = GREY
    return tray


def battery_envelope() -> Shape:
    result = rounded_box(BATTERY_SIZE_MM, BATTERY_CENTER_MM, 2.0)
    result.label = "BATTERY_2S_THIN_CONTROLLED_ENVELOPE_75X22X34"
    result.color = MAGENTA
    return result


def battery_cage() -> Shape:
    outer = rounded_box((79.0, 26.0, 38.0), BATTERY_CENTER_MM, 3.0)
    inner = rounded_box((75.8, 22.8, 34.8), BATTERY_CENTER_MM, 2.0)
    cage = outer.cut(inner)
    # Open the front service face but retain two 8 mm strap ribs.
    opening = rounded_box((65.0, 8.0, 26.0), (0.0, 71.0, -63.0), 2.0)
    cage = cage.cut(opening)
    for x_pos in (-26.0, 26.0):
        cage = cage.cut(
            rounded_box((8.0, 32.0, 3.0), (x_pos, 84.0, -43.0), 1.0)
        )
    cage.label = "ZEROTH01_V4_BATTERY_CAGE_WITH_STRAP_AND_SERVICE_FACE"
    cage.color = GREY
    return cage


def imu_envelope() -> Shape:
    result = rounded_box((32.0, 8.0, 25.0), IMU_CENTER_MM, 1.0)
    result.label = "TORSO_IMU_CONTROLLED_ENVELOPE_32X25X8"
    result.color = GREEN
    return result


def imu_shelf() -> Shape:
    shelf = rounded_box((38.0, 2.0, 31.0), (0.0, 99.0, -18.0), 1.5)
    shelf.label = "ZEROTH01_V4_TORSO_IMU_RIGID_M2_SHELF"
    shelf.color = GREY
    return shelf


def harness_guides() -> Shape:
    guides = []
    for y_pos in (-45.0, 45.0):
        for z_pos in (-30.0, 0.0):
            guide = rounded_box((6.0, 10.0, 8.0), (70.0, y_pos, z_pos), 2.0)
            guide = guide.cut(
                rounded_box((8.0, 5.0, 4.0), (70.0, y_pos, z_pos), 1.2)
            )
            guides.append(guide)
    result = Compound(children=guides)
    result.label = "ZEROTH01_V4_SHOULDER_HIP_HARNESS_STRAIN_RELIEF_GUIDES"
    result.color = GREY
    return result


def rear_service_pod() -> Shape:
    """Open-front white shell that preserves the released solid torso.

    The shell is authored in the BODY-local rear direction and inherits the
    measured BODY installation transform in the assembly manifest. Four bored
    standoffs provide a reversible M3 interface; no electronics envelope is
    exposed in the normal assembly.
    """

    outer = rounded_box((92.0, 32.0, 100.0), (0.0, 84.0, -42.0), 5.0)
    inner = rounded_box((88.0, 28.0, 96.0), (0.0, 84.0, -42.0), 3.5)
    shell = outer.cut(inner)
    shell = shell.cut(rounded_box((84.0, 8.0, 92.0), (0.0, 68.0, -42.0), 2.0))
    standoffs = []
    for x_pos in (-34.0, 34.0):
        for z_pos in (-16.0, -70.0):
            boss = Cylinder(3.2, 2.8, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((x_pos, 68.6, z_pos), (90.0, 0.0, 0.0))
            )
            bore = Cylinder(1.7, 4.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((x_pos, 68.6, z_pos), (90.0, 0.0, 0.0))
            )
            standoffs.append(boss.cut(bore))
    # Cable guides are integral so they cannot become loose external parts.
    guides = []
    for x_pos in (-45.0, 45.0):
        for z_pos in (-25.0, -55.0):
            guide = rounded_box((8.0, 6.0, 8.0), (x_pos, 84.0, z_pos), 2.0)
            guide = guide.cut(rounded_box((4.0, 8.0, 4.0), (x_pos, 84.0, z_pos), 1.0))
            guides.append(guide)
    result = shell.fuse(*standoffs, *guides, imu_shelf())
    result.label = "ZEROTH01_V4_REVERSIBLE_WHITE_REAR_SERVICE_POD_4XM3"
    result.color = WHITE
    return result


def small_palm(side: str) -> Shape:
    """Compact fixed palm retaining the v2 twin-ear/M3 wrist datum."""

    if side not in {"left", "right"}:
        raise ValueError(side)
    direction = -1.0 if side == "left" else 1.0
    center_x = -3.4808
    center_y = direction * 24.0
    center_z = 18.7993
    palm = rounded_box((30.0, 36.0, 24.0), (center_x, center_y, center_z), 8.0)
    bottom_ear = rounded_box((26.8, 18.0, 4.5), (center_x, direction * 2.0, 3.0), 2.0)
    top_ear = rounded_box((26.8, 18.0, 4.5), (center_x, direction * 2.0, 34.6), 2.0)
    result = palm.fuse(bottom_ear, top_ear)
    wrist_gap = rounded_box((21.8, 22.0, 25.0), (center_x, 0.0, center_z), 4.0)
    result = result.cut(wrist_gap)
    crossbolt = Cylinder(
        1.7,
        42.0,
        align=(Align.CENTER, Align.CENTER, Align.CENTER),
    ).moved(Location((center_x, 0.0, center_z), (90.0, 0.0, 0.0)))
    result = result.cut(crossbolt)
    result.label = f"ZEROTH01_V4_{side.upper()}_SMALL_LIGHT_PALM_TWIN_EAR_M3"
    result.color = WHITE
    return result


def shortened_lower_leg(side: str) -> Shape:
    """Remove 18 mm only from the straight mid-span of the source shin.

    Both terminal servo interfaces are retained verbatim.  The distal source
    half is translated toward the knee and two 3 mm outer splice webs recover
    print continuity; no hole pattern or terminal envelope is rescaled.
    """

    if side not in {"left", "right"}:
        raise ValueError(side)
    source_name = "3215_BothFlange_13" if side == "left" else "3215_BothFlange_14"
    source = source_step(source_name)
    split_x = 50.0
    proximal_clip = Box(
        500.0, 500.0, 500.0, align=(Align.MAX, Align.CENTER, Align.CENTER)
    ).moved(Location((split_x, 0.0, -18.0)))
    distal_clip = Box(
        500.0, 500.0, 500.0, align=(Align.MIN, Align.CENTER, Align.CENTER)
    ).moved(Location((split_x + SHIN_SHORTEN_MM, 0.0, -18.0)))
    proximal = common(source, proximal_clip)
    distal = common(source, distal_clip).moved(Location((-SHIN_SHORTEN_MM, 0.0, 0.0)))
    webs = [
        rounded_box((24.0, 3.0, 30.0), (split_x, y_pos, -18.0), 1.2)
        for y_pos in (-15.0, 15.0)
    ]
    result = Compound(children=[proximal, distal, *webs])
    result.label = f"ZEROTH01_V4_{side.upper()}_SOURCE_SHIN_SHORTENED_18MM_TERMINALS_RETAINED"
    result.color = WHITE
    return result


def direct_ankle_carrier(side: str) -> Shape:
    """Mirrored double-face ankle cage for the exact purchased STS3250.

    The output and rear plates both use the drawing-derived four-M2 pattern.
    This removes the old friction-fit shell assumption and gives the case a
    closed parent-side load path before the PCD14 output bridge drives FOOT.
    """

    cage_z = -20.75
    outer = rounded_box((52.0, 28.0, 43.0), (STS_CASE_CENTER_X_MM, 0.0, cage_z), 3.0)
    inner = rounded_box((46.42, 25.92, 42.0), (STS_CASE_CENTER_X_MM, 0.0, cage_z), 2.0)
    cage = outer.cut(inner)
    cage = cage.cut(
        Box(50.0, 40.0, 18.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((STS_CASE_CENTER_X_MM, 0.0, -4.5))
        )
    )
    anchor = rounded_box(
        (3.0, ANKLE_ROLL_DIRECT_OFFSET_MM, 20.0),
        (14.5, ANKLE_ROLL_DIRECT_OFFSET_MM / 2.0, 0.0),
        1.2,
    )
    cage = cage.fuse(anchor).cut(
        Cylinder(14.2, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER))
    )
    # Four M2 case screws on both axial faces.  The holes are coaxial with the
    # purchased servo's tapped pattern in the normalized shaft frame.
    for x_pos in STS_M2_X_MM:
        for y_pos in STS_M2_Y_MM:
            cage = cage.cut(
                Cylinder(
                    1.15,
                    48.0,
                    align=(Align.CENTER, Align.CENTER, Align.CENTER),
                ).moved(Location((x_pos, y_pos, cage_z)))
            )
    for z_pos in (-6.0, 6.0):
        cage = cage.cut(
            Cylinder(1.7, 6.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((14.5, ANKLE_ROLL_DIRECT_OFFSET_MM - 3.0, z_pos), (0.0, 90.0, 0.0))
            )
        )
    cage.label = f"ZEROTH01_V4_{side.upper()}_STS3250_DIRECT_ANKLE_CARRIER_26P5MM"
    cage.color = WHITE
    return cage


def sts3250_output_bridge_2p05mm() -> Shape:
    """PCD14 bridge between the purchased output face and child datum.

    The step.parts actuator stops 2.05 mm behind the released Zeroth joint
    plane.  This keyed four-M3 plate fills only that measured axial gap; it is
    not a cosmetic disc and is assigned to the opposite/rotating joint side.
    """

    bridge = Cylinder(
        9.975,
        STS3250_OUTPUT_FACE_GAP_MM,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    )
    bridge = bridge.cut(
        Cylinder(
            3.1,
            STS3250_OUTPUT_FACE_GAP_MM + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MAX),
        )
    )
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle_rad = math.radians(angle_deg)
        bridge = bridge.cut(
            Cylinder(
                1.6,
                STS3250_OUTPUT_FACE_GAP_MM + 1.0,
                align=(Align.CENTER, Align.CENTER, Align.MAX),
            ).moved(
                Location(
                    (
                        7.0 * math.cos(angle_rad),
                        7.0 * math.sin(angle_rad),
                        0.0,
                    )
                )
            )
        )
    bridge.label = "ZEROTH01_V4_STS3250_PCD14_OUTPUT_BRIDGE_2P05MM"
    bridge.color = BLUE
    return bridge


def sts3250_child_standoff(height_mm: float, outer_radius_mm: float = 9.975) -> Shape:
    """Four-hole PCD14 child-side standoff from joint datum to carrier.

    The original Zeroth carriers use three nominal output offsets: 1 mm for
    compact flanges, 3 mm at the elbows, and 8.9 mm at the ankle-pitch cage.
    Keeping this as a separate replaceable spacer preserves the released
    shaft axis while making the torque path explicit and printable.
    """

    standoff = Cylinder(
        outer_radius_mm,
        height_mm,
        align=(Align.CENTER, Align.CENTER, Align.MIN),
    )
    standoff = standoff.cut(
        Cylinder(
            3.1,
            height_mm + 1.0,
            align=(Align.CENTER, Align.CENTER, Align.MIN),
        )
    )
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle_rad = math.radians(angle_deg)
        standoff = standoff.cut(
            Cylinder(
                1.6,
                height_mm + 1.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(
                Location(
                    (
                        7.0 * math.cos(angle_rad),
                        7.0 * math.sin(angle_rad),
                        0.0,
                    )
                )
            )
        )
    standoff.label = (
        f"ZEROTH01_V4_STS3250_PCD14_CHILD_STANDOFF_{height_mm:g}MM_"
        f"R{outer_radius_mm:g}"
    )
    standoff.color = BLUE
    return standoff


def sts3250_case_4xm2_standoff_4mm() -> Shape:
    """Four M2 screw shanks spanning the hip-yaw case-side 4 mm offset."""

    screws = []
    for x_pos in STS_M2_X_MM:
        for y_pos in STS_M2_Y_MM:
            screw = Cylinder(
                0.9,
                4.0,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(Location((x_pos, y_pos, -39.45)))
            screws.append(screw)
    result = Compound(children=screws)
    result.label = "ZEROTH01_V4_STS3250_CASE_4XM2_TIE_RODS_4MM"
    result.color = BLUE
    return result


def sts3250_pcd14_4xm3_tie_rods_1p95mm() -> Shape:
    """Four M3 shanks joining the fixed joint plane to the shifted horn."""

    rods = []
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle_rad = math.radians(angle_deg)
        rods.append(
            Cylinder(
                1.25,
                1.95,
                align=(Align.CENTER, Align.CENTER, Align.MIN),
            ).moved(
                Location(
                    (
                        7.0 * math.cos(angle_rad),
                        7.0 * math.sin(angle_rad),
                        0.0,
                    )
                )
            )
        )
    result = Compound(children=rods)
    result.label = "ZEROTH01_V4_STS3250_PCD14_4XM3_TIE_RODS_1P95MM"
    result.color = BLUE
    return result


def remote_ankle_plate(front: bool) -> Shape:
    """Two-sided KHR-style support plate for the belt-driven roll shaft."""

    servo_x, servo_y = REMOTE_SERVO_CENTER_MM
    z_pos = 2.0 if front else -43.0
    pivot_pad = Cylinder(
        18.0, 3.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)
    ).moved(Location((0.0, 0.0, z_pos)))
    servo_pad = rounded_box(
        (52.0, 30.0, 3.0),
        (servo_x + STS_CASE_CENTER_X_MM, servo_y, z_pos),
        4.0,
    )
    midpoint = (servo_x / 2.0, servo_y / 2.0, z_pos)
    bridge_angle = math.degrees(math.atan2(servo_y, servo_x))
    bridge = rounded_box(
        (REMOTE_BELT_CENTER_DISTANCE_MM + 22.0, 18.0, 3.0),
        radius=5.0,
    ).moved(Location(midpoint, (0.0, 0.0, bridge_angle)))
    plate = pivot_pad.fuse(servo_pad, bridge)

    # 688-2RS driven-shaft bearing seat and the STS output/rear-boss seat.
    plate = plate.cut(
        Cylinder(8.05, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((0.0, 0.0, z_pos))
        )
    )
    plate = plate.cut(
        Cylinder(10.3, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((servo_x, servo_y, z_pos))
        )
    )
    for x_pos in STS_M2_X_MM:
        for y_pos in STS_M2_Y_MM:
            plate = plate.cut(
                Cylinder(1.1, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                    Location((servo_x + x_pos, servo_y + y_pos, z_pos))
                )
            )
    # Four M3 through-bolts clamp the two plates with replaceable spacers.
    for x_pos, y_pos in ((-13.0, 10.0), (13.0, 10.0), (-4.0, -32.0), (24.0, -32.0)):
        plate = plate.cut(
            Cylinder(1.65, 8.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((x_pos, y_pos, z_pos))
            )
        )
    plate.label = (
        "ZEROTH01_V4_REMOTE_ANKLE_FRONT_PLATE_688_STS3250"
        if front else "ZEROTH01_V4_REMOTE_ANKLE_REAR_PLATE_688_STS3250"
    )
    plate.color = WHITE
    return plate


def remote_ankle_spacers() -> Shape:
    spacers = []
    for x_pos, y_pos in ((-13.0, 10.0), (13.0, 10.0), (-4.0, -32.0), (24.0, -32.0)):
        spacer = Cylinder(
            3.0, 42.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        ).moved(Location((x_pos, y_pos, -20.5)))
        spacer = spacer.cut(
            Cylinder(1.65, 46.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((x_pos, y_pos, -20.5))
            )
        )
        spacers.append(spacer)
    result = Compound(children=spacers)
    result.label = "ZEROTH01_V4_REMOTE_ANKLE_4XM3_SPACERS"
    result.color = GREY
    return result


def hip_yaw_horn_spacer_2mm() -> Shape:
    """Two-millimetre PCD14 output spacer for the re-clocked hip-yaw pair."""

    spacer = Cylinder(9.975, 2.0, align=(Align.CENTER, Align.CENTER, Align.MIN))
    spacer = spacer.cut(Cylinder(3.1, 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle_rad = math.radians(angle_deg)
        spacer = spacer.cut(
            Cylinder(1.6, 3.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
                Location((7.0 * math.cos(angle_rad), 7.0 * math.sin(angle_rad), 0.0))
            )
        )
    spacer.label = "ZEROTH01_V4_HIP_YAW_PCD14_2MM_OUTPUT_SPACER"
    spacer.color = BLUE
    return spacer


def remote_ankle_bearings() -> Shape:
    bearings = []
    for z_pos in (2.0, -43.0):
        outer = Cylinder(8.0, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((0.0, 0.0, z_pos))
        )
        inner = Cylinder(4.0, 7.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((0.0, 0.0, z_pos))
        )
        bearings.append(outer.cut(inner))
    result = Compound(children=bearings)
    result.label = "PURCHASED_2X_688_2RS_BEARING_ENVELOPES"
    result.color = GREY
    return result


def remote_ankle_output_shaft() -> Shape:
    shaft = Cylinder(4.0, 57.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location((0.0, 0.0, -18.5))
    )
    shaft.label = "ZEROTH01_V4_8MM_ROLL_OUTPUT_SHAFT"
    shaft.color = GREY
    return shaft


def remote_ankle_pulleys() -> Shape:
    servo_x, servo_y = REMOTE_SERVO_CENTER_MM
    pulleys = []
    for x_pos, y_pos in ((0.0, 0.0), (servo_x, servo_y)):
        pulley = Cylinder(
            GT2_PITCH_RADIUS_MM + 1.2,
            6.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Location((x_pos, y_pos, 7.0)))
        bore_radius = 4.05 if x_pos == 0.0 else 3.0
        pulley = pulley.cut(
            Cylinder(
                bore_radius,
                8.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Location((x_pos, y_pos, 7.0)))
        )
        pulleys.append(pulley)
    result = Compound(children=pulleys)
    result.label = "ZEROTH01_V4_EQUAL_16T_GT2_PULLEYS_1TO1"
    result.color = BLUE
    return result


def remote_ankle_belt_reference() -> Shape:
    """Conservative 90 mm GT2 belt path envelope for assembly inspection."""

    servo_x, servo_y = REMOTE_SERVO_CENTER_MM
    angle = math.degrees(math.atan2(servo_y, servo_x))
    midpoint = (servo_x / 2.0, servo_y / 2.0, 7.0)
    # A narrow rounded path envelope is intentionally a reference, not a
    # collision solid. The real flexible belt wraps the two equal pulleys.
    path = rounded_box(
        (REMOTE_BELT_CENTER_DISTANCE_MM + 15.0, 3.0, 5.0), radius=1.3
    ).moved(Location(midpoint, (0.0, 0.0, angle)))
    path.label = "PURCHASED_GT2_90MM_X_5MM_CLOSED_BELT_PATH_REFERENCE"
    path.color = BLACK
    return path


def remote_ankle_horn_adapter() -> Shape:
    horn = Cylinder(13.5, 3.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
        Location((0.0, 0.0, 12.0))
    )
    horn = horn.cut(
        Cylinder(4.05, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
            Location((0.0, 0.0, 12.0))
        )
    )
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle_rad = math.radians(angle_deg)
        horn = horn.cut(
            Cylinder(1.6, 5.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)).moved(
                Location((7.0 * math.cos(angle_rad), 7.0 * math.sin(angle_rad), 12.0))
            )
        )
    horn.label = "ZEROTH01_V4_8MM_SHAFT_TO_FOOT_PCD14_HORN"
    horn.color = BLUE
    return horn


def parts() -> dict[str, Shape]:
    body = body_without_old_head()
    return {
        "body_original_head_interface_trimmed_2p5mm": body,
        "sts3250_step_parts_exact_shaft_frame": exact_sts3250_shaft_frame(),
        "sts3250_pcd14_output_bridge_2p05mm": sts3250_output_bridge_2p05mm(),
        "sts3250_pcd14_child_standoff_1mm": sts3250_child_standoff(1.0),
        "sts3250_pcd14_4xm3_tie_rods_1p95mm": sts3250_pcd14_4xm3_tie_rods_1p95mm(),
        "sts3250_pcd14_child_standoff_12p95mm": sts3250_child_standoff(12.95),
        "sts3250_case_4xm2_standoff_4mm": sts3250_case_4xm2_standoff_4mm(),
        "left_source_shin_shortened_18mm": shortened_lower_leg("left"),
        "right_source_shin_shortened_18mm": shortened_lower_leg("right"),
        "left_direct_ankle_carrier_26p5mm": direct_ankle_carrier("left"),
        "right_direct_ankle_carrier_26p5mm": direct_ankle_carrier("right"),
        "head_front_5mm_each_side": split_head(True),
        "head_rear_5mm_each_side": split_head(False),
        "head_simple_visor": head_visor(),
        "m5stack_unitv2_purchased_envelope": unitv2_envelope(),
        "unitv2_removable_cradle": unitv2_bracket(),
        "direct_head_torso_nut_plate": head_torso_nut_plate(),
        "head_mount_4xm3_drill_jig": head_mount_drill_jig(),
        "compute_envelope": compute_envelope(),
        "compute_removable_tray": compute_tray(),
        "battery_envelope": battery_envelope(),
        "battery_service_cage": battery_cage(),
        "torso_imu_envelope": imu_envelope(),
        "torso_imu_shelf": imu_shelf(),
        "harness_strain_relief_guides": harness_guides(),
    }


def gen_step() -> Compound:
    selected = parts()
    assembly = Compound(children=list(selected.values()))
    assembly.label = "ZEROTH01_V4_ORIGINAL_MINIMAL_HEAD_AND_SERVICE_INSTALLATION"
    return assembly


def main() -> int:
    PARTS.mkdir(parents=True, exist_ok=True)
    for stem in sorted(OBSOLETE_GENERATED_PART_STEMS):
        for suffix in (".step", ".stl"):
            (PARTS / f"{stem}{suffix}").unlink(missing_ok=True)
    rows = []
    selected = parts()
    for name, shape in selected.items():
        step_path = PARTS / f"{name}.step"
        stl_path = PARTS / f"{name}.stl"
        if name == "body_original_head_interface_trimmed_2p5mm" and step_path.is_file() and step_path.stat().st_size > 1_000_000:
            export_stl(shape, stl_path, tolerance=0.08, angular_tolerance=0.15)
        else:
            export_step(shape, step_path)
            export_stl(shape, stl_path, tolerance=0.08, angular_tolerance=0.15)
        box = shape.bounding_box()
        rows.append(
            {
                "name": name,
                "step": step_path.relative_to(ROOT).as_posix(),
                "stl": stl_path.relative_to(ROOT).as_posix(),
                "bbox_min_mm": list(box.min),
                "bbox_max_mm": list(box.max),
                "bbox_size_mm": list(box.size),
                "volume_mm3": float(shape.volume),
                "valid_brep": bool(shape.is_valid),
            }
        )
        print(name, rows[-1]["bbox_size_mm"], rows[-1]["valid_brep"], flush=True)

    diagnostic = Compound(children=list(selected.values()))
    diagnostic.label = "ZEROTH01_V4_ORIGINAL_MINIMAL_HEAD_AND_SERVICE_INSTALLATION"
    diagnostic_path = OUT / "ZEROTH01_V4_ORIGINAL_MINIMAL_INSTALLATION_DIAGNOSTIC.step"
    export_step(diagnostic, diagnostic_path)
    report = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.cad_build.v1",
        "source_head_bbox_mm": {
            "minimum": SOURCE_HEAD_MIN_MM,
            "maximum": SOURCE_HEAD_MAX_MM,
            "size": SOURCE_HEAD_SIZE_MM,
        },
        "v4_head_outer_size_mm": HEAD_OUTER_SIZE_MM,
        "head_expansion_mm": {"left": 5.0, "right": 5.0, "top": 5.0, "bottom_nominal_outer_envelope": 5.0},
        "head_local_underside_exception": "Shoulder-servo B-Rep pockets remove the hidden lower corners with 0.8 mm clearance; local solid expansion there is about 3.9 mm while the nominal outer envelope remains +5 mm.",
        "old_head_removal_seam_z_mm": HEAD_SEAM_Z_MM,
        "purchased_head_payload": {
            "model": "M5Stack UnitV2",
            "size_mm": UNITV2_SIZE_MM,
            "mass_kg": UNITV2_MASS_KG,
            "camera": "GC2145 1080p",
            "microphone_count": 1,
        },
        "diagnostic": diagnostic_path.relative_to(ROOT).as_posix(),
        "parts": rows,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(REPORT, flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
