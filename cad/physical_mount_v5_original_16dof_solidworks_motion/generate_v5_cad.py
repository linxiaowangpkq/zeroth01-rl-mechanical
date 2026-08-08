"""Generate the minimal new CAD used by Zeroth-01 v5.

v5 deliberately reuses the released Zeroth-01 load-bearing links and the
validated v4 head/payload geometry.  It adds only the purchased-exact
STS3250 interface stack and a compact fixed wrist support.  There are no
ankle-roll carriers, shortened shins, black soles, claws or large palms.
"""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

from build123d import Align, Axis, Compound, Cylinder, Location, Shape, Solid, Wire, export_step, export_stl


ROOT = Path(__file__).resolve().parents[2]
V4_SCRIPT = ROOT / "cad" / "physical_mount_v4_original_minimal" / "generate_v4_cad.py"
OUT = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion"
PARTS = OUT / "parts"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "cad_build.json"

WHITE = "#F7F8FA"
BLUE = "#1677FF"
GREY = "#BFC7D1"
STEEL = "#586069"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v4 = load(V4_SCRIPT, "zeroth01_v4_cad_reused_by_v5")


def servo_parent_thrust_ring() -> Shape:
    """Parent-side annulus used by the real SolidWorks revolute mate.

    The purchased STS3250 ends at local Z=-2.05 mm.  This ring begins at
    -2.00 mm, surrounds (without intersecting) the 19.95 mm output bridge and
    presents a cylindrical axis plus a Z=0 thrust face.  It is locked to the
    parent carrier; the output bridge is locked to the child carrier.
    """

    ring = Cylinder(
        12.5,
        2.0,
        align=(Align.CENTER, Align.CENTER, Align.MAX),
    ).cut(
        Cylinder(
            10.2,
            2.2,
            align=(Align.CENTER, Align.CENTER, Align.MAX),
        )
    )
    for angle_deg in (45.0, 135.0, 225.0, 315.0):
        a = math.radians(angle_deg)
        ring = ring.cut(
            Cylinder(
                1.15,
                3.0,
                align=(Align.CENTER, Align.CENTER, Align.MAX),
            ).moved(Location((11.25 * math.cos(a), 11.25 * math.sin(a), 0.0)))
        )
    ring.label = "ZEROTH01_V5_STS3250_PARENT_THRUST_RING_MOTION_DATUM"
    ring.color = GREY
    return ring


def wrist_support(side: str) -> Shape:
    """Small two-bolt support bumper on the released forearm end face.

    This is deliberately neither a gripper nor a cosmetic palm.  The rounded
    block sits outside the released forearm instead of being buried inside it;
    two M3 through-bolts provide positive retention and anti-rotation.
    """

    if side not in {"left", "right"}:
        raise ValueError(side)
    direction = -1.0 if side == "left" else 1.0
    cx = -3.4808
    cz = 18.7993
    # The released forearm occupies local |Y| <= about 10.11 mm at this end.
    # A 7 mm pad centred at |Y|=14.0 leaves 0.39 mm nominal assembly clearance.
    support = v4.rounded_box((20.0, 7.0, 14.0), (cx, direction * 14.0, cz), 3.0)
    for x_pos in (cx - 5.0, cx + 5.0):
        through_bolt = Cylinder(
            1.7,
            30.0,
            align=(Align.CENTER, Align.CENTER, Align.CENTER),
        ).moved(Location((x_pos, 0.0, cz), (90.0, 0.0, 0.0)))
        support = support.cut(through_bolt)
    support.label = f"ZEROTH01_V5_{side.upper()}_TWO_BOLT_WRIST_SUPPORT_M3"
    support.color = WHITE
    return support


def pcd14_output_screws_m3x6() -> Shape:
    """Four real M3x6 shanks clamping carrier+bridge to the STS output."""

    screws = []
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle = math.radians(angle_deg)
        screws.append(
            Cylinder(1.25, 6.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
                Location((7.0 * math.cos(angle), 7.0 * math.sin(angle), -2.05))
            )
        )
    result = Compound(children=screws)
    result.label = "ZEROTH01_V5_STS3250_PCD14_4XM3X6_OUTPUT_CLAMP_SCREWS"
    result.color = BLUE
    return result


def hip_yaw_case_mount_ribs_2xm2() -> Shape:
    """Torso-integrated rear case mount in the neutral hip-yaw frame.

    Exact distance queries place the fixed torso side walls at local
    Y=-26.84/+26.88 mm.  Two narrow ribs bridge those walls to the official
    negative-X STS3250 M2 holes.  The mount stops at the exact case rear face;
    the surrounding 0.30 mm pocket remains a form-fit reaction surface.
    """

    pieces = []
    for y_mm in (-10.25, 10.25):
        sign = -1.0 if y_mm < 0.0 else 1.0
        outer_y = sign * 28.0
        bore = Cylinder(
            1.1, 6.0, align=(Align.CENTER, Align.CENTER, Align.CENTER)
        ).moved(Location((-28.5, y_mm, -37.45)))
        rib = v4.rounded_box(
                (6.4, abs(outer_y - y_mm), 4.0),
                (-28.5, (y_mm + outer_y) / 2.0, -37.45),
                1.2,
            ).cut(bore)
        boss = Cylinder(
            3.2, 4.0, align=(Align.CENTER, Align.CENTER, Align.MIN)
        ).moved(Location((-28.5, y_mm, -39.45))).cut(bore)
        pieces.extend((rib, boss))
    result = Compound(children=pieces)
    result.label = "ZEROTH01_V5_TORSO_INTEGRATED_HIP_YAW_CASE_MOUNT_2XM2"
    result.color = WHITE
    return result


def hip_yaw_case_screws_2xm2x8() -> Shape:
    """Two M2x8 case screws plus low-profile heads, joint-local."""

    screws = []
    for y_mm in (-10.25, 10.25):
        shank = Cylinder(0.95, 8.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(
            Location((-28.5, y_mm, -39.45))
        )
        head = Cylinder(2.0, 1.5, align=(Align.CENTER, Align.CENTER, Align.MAX)).moved(
            Location((-28.5, y_mm, -39.45))
        )
        screws.extend((shank, head))
    result = Compound(children=screws)
    result.label = "ZEROTH01_V5_HIP_YAW_2XM2X8_CASE_SCREWS"
    result.color = STEEL
    return result


def tapered_sole_flare(side: str) -> Shape:
    """9 mm white sole: top follows the released foot, bottom is wider.

    The 9 mm thickness preserves the released Zeroth contact plane at local
    |Y|=44.6 mm; the earlier 6.5 mm draft left the RL contact sites floating
    2.47 mm below the physical ground face.
    """

    if side not in {"left", "right"}:
        raise ValueError(side)
    # Original foot local bounds are X=-61..28.95, Z=-1.52..38.92.  The top
    # footprint stays inside that outline; the ground face gains 4 mm per side.
    center_x = -16.025
    center_z = 18.70
    top_y = 35.63 if side == "left" else -35.63
    direction = 1.0 if side == "left" else -1.0
    top = Wire.make_rect(90.0, 40.0)
    bottom = Wire.make_rect(98.0, 48.0).moved(Location((0.0, 0.0, -9.0)))
    flare = Solid.make_loft((top, bottom), ruled=False)
    try:
        flare = flare.fillet(1.8, flare.edges())
    except Exception:
        pass
    flare = flare.rotate(Axis.X, 90.0 if side == "left" else -90.0)
    flare = flare.moved(Location((center_x, top_y, center_z)))
    # Four M3 clearance holes make the sole replaceable rather than cosmetic.
    for x_pos in (center_x - 30.0, center_x + 30.0):
        for z_pos in (center_z - 12.0, center_z + 12.0):
            hole = Cylinder(
                1.7,
                12.0,
                align=(Align.CENTER, Align.CENTER, Align.CENTER),
            ).moved(Location((x_pos, top_y + direction * 4.5, z_pos), (90.0, 0.0, 0.0)))
            flare = flare.cut(hole)
    flare.label = f"ZEROTH01_V5_{side.upper()}_TAPERED_LOWER_WIDER_SOLE_FLARE"
    flare.color = WHITE
    return flare


def four_post_child_standoff(height_mm: float) -> Shape:
    """Single-body PCD14 spacer with a clear STS3250 center hub.

    The earlier full annulus intersected the purchased output boss at the two
    4 mm hip-yaw stacks.  A 9.6 mm center clearance removes that clash while
    retaining one connected printable body; four PCD14 M3 holes carry preload
    into the output bridge and child carrier.
    """

    result = Cylinder(9.975, height_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))
    result = result.cut(Cylinder(4.8, height_mm + 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        a = math.radians(angle_deg)
        center = (7.0 * math.cos(a), 7.0 * math.sin(a), 0.0)
        bore = Cylinder(1.6, height_mm + 1.0, align=(Align.CENTER, Align.CENTER, Align.MIN)).moved(Location(center))
        result = result.cut(bore)
    result.label = f"ZEROTH01_V5_STS3250_PCD14_HUB_CLEARANCE_CHILD_STANDOFF_{height_mm:g}MM"
    result.color = BLUE
    return result


def four_sleeve_child_spacer(height_mm: float, start_z_mm: float = 0.0) -> Shape:
    """Four independent off-the-shelf OD4 M3 sleeves; no full disc.

    OD4 is the largest standard metal spacer envelope that clears the exact
    purchased STS3250 face at the two 4 mm hip-yaw offsets.  These are BOM
    hardware, not weak printed 0.4 mm-wall parts.
    """

    sleeves = []
    for angle_deg in (0.0, 90.0, 180.0, 270.0):
        angle = math.radians(angle_deg)
        center = (7.0 * math.cos(angle), 7.0 * math.sin(angle), 0.0)
        sleeve = Cylinder(2.0, height_mm, align=(Align.CENTER, Align.CENTER, Align.MIN))
        sleeve = sleeve.cut(Cylinder(1.6, height_mm + 0.5, align=(Align.CENTER, Align.CENTER, Align.MIN)))
        sleeves.append(sleeve.moved(Location((center[0], center[1], start_z_mm))))
    result = Compound(children=sleeves)
    result.label = f"ZEROTH01_V5_STS3250_PCD14_FOUR_M3_SPACER_SLEEVES_{height_mm:g}MM_Z{start_z_mm:g}"
    result.color = BLUE
    return result


def parts() -> dict[str, Shape]:
    # Reuse only v4 items that were already accepted by the user: original
    # torso with head seam trim, rounded +5 mm head and internal electronics.
    return {
        "body_original_head_interface_trimmed_2p5mm": v4.body_without_old_head(),
        "sts3250_step_parts_exact_shaft_frame": v4.exact_sts3250_shaft_frame(),
        "sts3250_pcd14_output_bridge_2p05mm": v4.sts3250_output_bridge_2p05mm(),
        "sts3250_parent_thrust_ring_2mm": servo_parent_thrust_ring(),
        "sts3250_pcd14_child_standoff_1mm": four_post_child_standoff(1.0),
        "sts3250_pcd14_child_standoff_1p95mm": four_post_child_standoff(1.95),
        # Exact-BRep fit: the purchased STS face occupies local Z=0..0.25 at
        # the PCD14 pads.  OD4 metal sleeves therefore seat at Z=0.30 and end
        # at Z=1.95 against the rotating bridge, with 0.05 mm face clearance.
        "sts3250_pcd14_four_sleeve_spacer_1p65mm_z0p30": four_sleeve_child_spacer(1.65, 0.30),
        "sts3250_pcd14_4xm3_tie_rods_1p95mm": v4.sts3250_pcd14_4xm3_tie_rods_1p95mm(),
        "sts3250_pcd14_child_standoff_3mm": four_post_child_standoff(3.0),
        "sts3250_pcd14_child_standoff_12p95mm": four_post_child_standoff(12.95),
        "sts3250_case_4xm2_standoff_4mm": v4.sts3250_case_4xm2_standoff_4mm(),
        "left_fixed_wrist_support": wrist_support("left"),
        "right_fixed_wrist_support": wrist_support("right"),
        "sts3250_pcd14_4xm3_output_screws_m3x6": pcd14_output_screws_m3x6(),
        "sts3250_hip_yaw_case_mount_ribs_2xm2": hip_yaw_case_mount_ribs_2xm2(),
        "sts3250_hip_yaw_2xm2x8_case_screws": hip_yaw_case_screws_2xm2x8(),
        "left_tapered_sole_flare": tapered_sole_flare("left"),
        "right_tapered_sole_flare": tapered_sole_flare("right"),
        "head_front_5mm_each_side": v4.split_head(True),
        "head_rear_5mm_each_side": v4.split_head(False),
        "head_simple_visor": v4.head_visor(),
        "m5stack_unitv2_purchased_envelope": v4.unitv2_envelope(),
        "unitv2_removable_cradle": v4.unitv2_bracket(),
        "direct_head_torso_nut_plate": v4.head_torso_nut_plate(),
        "compute_envelope": v4.compute_envelope(),
        "compute_removable_tray": v4.compute_tray(),
        "battery_envelope": v4.battery_envelope(),
        "battery_service_cage": v4.battery_cage(),
        "torso_imu_envelope": v4.imu_envelope(),
        "harness_strain_relief_guides": v4.harness_guides(),
    }


def main() -> int:
    PARTS.mkdir(parents=True, exist_ok=True)
    rows = []
    generated = parts()
    for name, shape in generated.items():
        step_path = PARTS / f"{name}.step"
        stl_path = PARTS / f"{name}.stl"
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
    diagnostic = OUT / "ZEROTH01_V5_NEW_PARTS_DIAGNOSTIC.step"
    export_step(Compound(children=list(generated.values())), diagnostic)
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.cad_build.v1",
        "design_policy": "original Zeroth-01 16-actuator geometry; no ankle roll, no shortened shin, no black sole, no claw, no large palm",
        "diagnostic": diagnostic.relative_to(ROOT).as_posix(),
        "parts": rows,
        "overall": "PASS" if all(row["valid_brep"] for row in rows) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"part_count": len(rows), "overall": payload["overall"], "report": str(REPORT)}, indent=2))
    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
