"""Generate the Zeroth-01 v4 original-minimal 18DoF RL URDF.

The released v2 tree remains the kinematic provenance.  V4 changes only the
two ankle frames required by the verified direct 26.5 mm roll stage and the
18 mm straight-shin compaction.  All visible manufacturing meshes come from
the v4 SolidWorks-gated CAD manifest; collisions stay simple and convex for
MJX training.  Generated XML is an artifact and must not be hand edited.
"""

from __future__ import annotations

import copy
import importlib.util
import json
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
V3_SOURCE = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "build_v3_urdf.py"
V4_CAD = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal"
V4_PARTS = V4_CAD / "parts"
V3_PARTS = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed" / "parts"
V4_MANIFEST = V4_CAD / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
OUT_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v4_original_minimal"
OUT_URDF = OUT_ROOT / "zeroth01_physical_mount_v4_original_minimal_18dof.urdf"
REPORT = ROOT / "reports" / "v4_original_minimal" / "urdf_mass_inertia_gate.json"

TARGET_TOTAL_MASS_KG = 2.850
SHIN_SHORTEN_M = 0.018
ANKLE_ROLL_OFFSET_M = 0.0265
SOLE_WORLD_CENTER_Z_M = -0.03260004
SOLE_CONTACT_WORLD_Z_M = -0.03560004

# Nominal, pre-first-article engineering estimates.  STS3250 mass is already
# owned exactly once by the aggregate articulated links inherited from v3.
V4_FIXED_MASSES = {
    "v4_head_shell": 0.055,
    "m5stack_unitv2": 0.018,
}


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v3 = load_module(V3_SOURCE, "zeroth01_v3_urdf_base_for_v4")


def neutral_transforms_v4(old_tf):
    """Return CAD-matching neutral frames without changing released axes."""

    result = dict(old_tf)
    for carrier, foot in (
        (v3.LEFT_ANKLE_CARRIER, "FOOT"),
        (v3.RIGHT_ANKLE_CARRIER, "FOOT_2"),
    ):
        rotation, released_position = old_tf[foot]
        pitch_position = v3.add(released_position, (0.0, 0.0, SHIN_SHORTEN_M))
        result[carrier] = (rotation, pitch_position)
        result[foot] = (
            rotation,
            v3.add(pitch_position, (0.0, 0.0, -ANKLE_ROLL_OFFSET_M)),
        )
    return result


_v3_joint_kinematics = v3.joint_kinematics


def joint_kinematics_v4(old_robot: ET.Element, neutral_tf):
    rows = _v3_joint_kinematics(old_robot, neutral_tf)
    result = []
    for name, parent, child, origin, axis, limits in rows:
        if name in {"left_ankle_pitch", "right_ankle_pitch"}:
            origin = v3.relative_transform(neutral_tf[parent], neutral_tf[child])
        result.append((name, parent, child, origin, axis, limits))
    return result


def link_by_name(robot: ET.Element, name: str) -> ET.Element:
    link = next((item for item in robot.findall("link") if item.get("name") == name), None)
    if link is None:
        raise KeyError(name)
    return link


def remove_links(robot: ET.Element, names: set[str]) -> None:
    for joint in list(robot.findall("joint")):
        parent = str(joint.find("parent").get("link"))
        child = str(joint.find("child").get("link"))
        if parent in names or child in names:
            robot.remove(joint)
    for link in list(robot.findall("link")):
        if str(link.get("name")) in names:
            robot.remove(link)


def add_mesh_visual(
    link: ET.Element,
    name: str,
    filename: str,
    rgba: str,
    *,
    xyz=(0.0, 0.0, 0.0),
    rpy=(0.0, 0.0, 0.0),
) -> None:
    visual = ET.SubElement(link, "visual", name=name)
    ET.SubElement(visual, "origin", xyz=v3.fmt(xyz), rpy=v3.fmt(rpy))
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", filename=filename, scale="0.001 0.001 0.001")
    v3.add_material(visual, f"{name}_material", rgba)


def add_box_collision(link: ET.Element, name: str, size, center) -> None:
    collision = ET.SubElement(link, "collision", name=name)
    ET.SubElement(collision, "origin", xyz=v3.fmt(center), rpy="0 0 0")
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "box", size=v3.fmt(size))


def add_fixed_body_link(
    robot: ET.Element,
    name: str,
    mass: float,
    inertia_size,
    inertia_center,
    *,
    parent: str | None = None,
) -> ET.Element:
    link = ET.SubElement(robot, "link", name=name)
    v3.add_inertial(link, mass, inertia_size, inertia_center)
    v3.add_joint(
        robot,
        f"{name}_fixed_joint",
        parent or v3.BODY,
        name,
        (0.0, 0.0, 0.0),
        kind="fixed",
    )
    return link


def replace_manufacturing_visuals(robot: ET.Element) -> None:
    replacements = {
        "meshes/v3/body_skeleton_top_trimmed_45mm.stl": "meshes/v4/body_original_head_interface_trimmed_2p5mm.stl",
        "meshes/skeleton/3215_BothFlange_13.stl": "meshes/v4/left_source_shin_shortened_18mm.stl",
        "meshes/skeleton/3215_BothFlange_14.stl": "meshes/v4/right_source_shin_shortened_18mm.stl",
        "meshes/v3/left_ankle_roll_carrier.stl": "meshes/v4/left_direct_ankle_carrier_26p5mm.stl",
        "meshes/v3/right_ankle_roll_carrier.stl": "meshes/v4/right_direct_ankle_carrier_26p5mm.stl",
        "meshes/v3/sts3250_dimension_controlled.stl": "meshes/v4/sts3250_step_parts_exact_shaft_frame.stl",
    }
    for mesh in robot.findall(".//mesh"):
        filename = str(mesh.get("filename"))
        if filename in replacements:
            mesh.set("filename", replacements[filename])
            mesh.set("scale", "0.001 0.001 0.001")

    for link_name, side in (("FINGER_1", "left"), ("FINGER_1_2", "right")):
        link = link_by_name(robot, link_name)
        for visual in list(link.findall("visual")):
            link.remove(visual)
        for collision in list(link.findall("collision")):
            link.remove(collision)

    # Remove every later-added external sole.  The original Zeroth FOOT/FOOT_2
    # geometry remains and defines the corrected contact plane below.
    for foot_name in ("FOOT", "FOOT_2"):
        foot = link_by_name(robot, foot_name)
        for visual in list(foot.findall("visual")):
            if "sole" in str(visual.get("name", "")).lower():
                foot.remove(visual)
        for collision in list(foot.findall("collision")):
            if "sole" in str(collision.get("name", "")).lower():
                foot.remove(collision)


def add_output_bridge_visuals(robot: ET.Element) -> None:
    """Attach each PCD14 bridge/standoff stack to the rotating side."""

    manifest = json.loads(V4_MANIFEST.read_text(encoding="utf-8"))
    old_robot = ET.parse(v3.V2_URDF).getroot()
    neutral_tf = neutral_transforms_v4(v3.old_fk(old_robot))
    rows = [
        row
        for row in manifest["components"]
        if row.get("role") in {
            "sts3250_pcd14_output_bridge_to_child",
            "sts3250_pcd14_child_standoff_to_carrier",
            "sts3250_pcd14_4xm3_tie_rods_to_carrier",
        }
    ]
    case_rows = [
        row
        for row in manifest["components"]
        if row.get("role") == "sts3250_case_4xm2_tie_rods_to_parent"
    ]
    bridge_count = sum(row.get("role") == "sts3250_pcd14_output_bridge_to_child" for row in rows)
    standoff_count = sum(
        row.get("role") in {
            "sts3250_pcd14_child_standoff_to_carrier",
            "sts3250_pcd14_4xm3_tie_rods_to_carrier",
        }
        for row in rows
    )
    expected_standoff_count = len(manifest.get("child_output_standoffs_mm", {}))
    if (bridge_count, standoff_count) != (18, expected_standoff_count):
        raise RuntimeError(
            f"expected 18 PCD14 bridges and {expected_standoff_count} child standoffs, got {bridge_count} and {standoff_count}"
        )
    if len(case_rows) != len(manifest.get("servo_axial_shims_mm", {})):
        raise RuntimeError(f"unexpected case-side standoff count: {len(case_rows)}")
    for row in rows + case_rows:
        owner = str(row["owner_link"])
        local_tf = v3.relative_transform(
            neutral_tf[owner],
            v3.component_world_transform(row),
        )
        add_mesh_visual(
            link_by_name(robot, owner),
            f"{row['component_id']}_visual",
            f"meshes/v4/{Path(str(row['source'])).with_suffix('.stl').name}",
            "0.086 0.467 1 1",
            xyz=local_tf[1],
            rpy=v3.matrix_rpy(local_tf[0]),
        )


def add_v4_body_payloads(robot: ET.Element) -> None:
    head = add_fixed_body_link(
        robot,
        "v4_head_shell",
        V4_FIXED_MASSES["v4_head_shell"],
        (0.090750004, 0.031000001, 0.070409235),
        (0.0, 0.009189728, 0.03883662),
    )
    for filename, name, rgba in (
        ("head_front_5mm_each_side.stl", "v4_head_front_visual", "0.969 0.973 0.980 1"),
        ("head_rear_5mm_each_side.stl", "v4_head_rear_visual", "0.969 0.973 0.980 1"),
        ("head_simple_visor.stl", "v4_simple_visor_visual", "0.02 0.03 0.04 1"),
        ("unitv2_removable_cradle.stl", "v4_unitv2_cradle_visual", "0.75 0.78 0.82 1"),
        ("direct_head_torso_nut_plate.stl", "v4_head_nut_plate_visual", "0.75 0.78 0.82 1"),
    ):
        add_mesh_visual(head, name, f"meshes/v4/{filename}", rgba)
    add_box_collision(
        head,
        "v4_head_convex_training_collision",
        (0.090750004, 0.031000001, 0.070409235),
        (0.0, 0.009189728, 0.03883662),
    )

    unitv2 = add_fixed_body_link(
        robot,
        "m5stack_unitv2",
        V4_FIXED_MASSES["m5stack_unitv2"],
        (0.048, 0.024260273, 0.024),
        (0.0, 0.006, 0.045),
        parent="v4_head_shell",
    )
    add_mesh_visual(
        unitv2,
        "m5stack_unitv2_visual",
        "meshes/v4/m5stack_unitv2_purchased_envelope.stl",
        "0 0.72 0.85 1",
    )

    # UnitV2 optical +Z points through the front face, i.e. BODY-local -Y.
    for name, parent, xyz, rpy in (
        ("camera_optical_frame", "m5stack_unitv2", (-0.014, -0.0075102725, 0.045), (1.57079632679, 0.0, 0.0)),
        ("microphone_frame", "m5stack_unitv2", (0.014, -0.0075102725, 0.045), (0.0, 0.0, 0.0)),
    ):
        ET.SubElement(robot, "link", name=name)
        v3.add_joint(robot, f"{name}_joint", parent, name, xyz, kind="fixed", rpy=rpy)


def rebalance_body_mass(robot: ET.Element) -> dict[str, float]:
    masses = {
        str(link.get("name")): float(link.find("./inertial/mass").get("value"))
        for link in robot.findall("link")
        if link.find("./inertial/mass") is not None
    }
    current = sum(masses.values())
    body = link_by_name(robot, v3.BODY)
    inertial = body.find("inertial")
    old_mass = float(inertial.find("mass").get("value"))
    new_mass = old_mass + TARGET_TOTAL_MASS_KG - current
    if new_mass <= 0.40:
        raise RuntimeError(f"implausible body aggregate mass after rebalance: {new_mass}")
    scale = new_mass / old_mass
    inertial.find("mass").set("value", f"{new_mass:.12g}")
    inertia = inertial.find("inertia")
    for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
        inertia.set(key, f"{float(inertia.get(key)) * scale:.12g}")
    masses[v3.BODY] = new_mass
    return masses


def update_sole_contact_frames(robot: ET.Element) -> None:
    old_robot = ET.parse(v3.V2_URDF).getroot()
    neutral_tf = neutral_transforms_v4(v3.old_fk(old_robot))
    joints = {str(joint.get("name")): joint for joint in robot.findall("joint")}
    for side, foot in (("left", "FOOT"), ("right", "FOOT_2")):
        rotation_t = v3.mat_t(neutral_tf[foot][0])
        for corner, world_offset in (
            ("front_medial", (0.045, -0.030, SOLE_CONTACT_WORLD_Z_M)),
            ("front_lateral", (0.045, 0.030, SOLE_CONTACT_WORLD_Z_M)),
            ("rear_medial", (-0.055, -0.030, SOLE_CONTACT_WORLD_Z_M)),
            ("rear_lateral", (-0.055, 0.030, SOLE_CONTACT_WORLD_Z_M)),
        ):
            frame = f"{side}_sole_{corner}_contact"
            joints[f"{frame}_joint"].find("origin").set(
                "xyz", v3.fmt(v3.mat_vec(rotation_t, world_offset))
            )


def copy_meshes(robot: ET.Element) -> None:
    mesh_root = OUT_ROOT / "meshes"
    if mesh_root.is_dir():
        shutil.rmtree(mesh_root)
    for filename in sorted({str(mesh.get("filename")) for mesh in robot.findall(".//mesh")}):
        relative = Path(filename)
        if not relative.parts or relative.parts[0] != "meshes":
            raise RuntimeError(f"non-portable mesh reference: {filename}")
        if len(relative.parts) >= 2 and relative.parts[1] == "v4":
            source = V4_PARTS / relative.name
        elif len(relative.parts) >= 2 and relative.parts[1] == "v3":
            source = V3_PARTS / relative.name
        else:
            source = v3.V2_ROOT / relative
        if not source.is_file():
            raise FileNotFoundError(source)
        target = OUT_ROOT / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def gen_urdf() -> ET.Element:
    v3.ASSEMBLY_MANIFEST = V4_MANIFEST
    v3.neutral_transforms = neutral_transforms_v4
    v3.joint_kinematics = joint_kinematics_v4
    v3.TARGET_TOTAL_MASS_KG = TARGET_TOTAL_MASS_KG
    v3.ANKLE_ROLL_OFFSET_M = ANKLE_ROLL_OFFSET_M
    v3.FIXED_MASSES["FINGER_1"] = 0.005
    v3.FIXED_MASSES["FINGER_1_2"] = 0.005
    v3.FIXED_MASSES["3215_BothFlange_13"] = 0.205
    v3.FIXED_MASSES["3215_BothFlange_14"] = 0.205
    v3.FIXED_MASSES[v3.LEFT_ANKLE_CARRIER] = 0.095
    v3.FIXED_MASSES[v3.RIGHT_ANKLE_CARRIER] = 0.095
    v3.COLLISION["FOOT"] = (
        (0.090, 0.041, 0.006),
        (-0.010, 0.0, SOLE_WORLD_CENTER_Z_M),
    )
    v3.COLLISION["FOOT_2"] = v3.COLLISION["FOOT"]

    robot = v3.gen_urdf()
    robot.set("name", "zeroth01_physical_mount_v4_original_minimal_18dof")
    remove_links(
        robot,
        {
            "cores3_internal_torso_cradle",
            "m5stack_cores3_head_module",
            "IMU_2",
            "torso_imu_module",
            "compute_module",
            "battery_pack",
            "camera_optical_frame",
            "left_microphone_frame",
            "right_microphone_frame",
            "head_speaker_frame",
            "head_imu_frame",
        },
    )
    replace_manufacturing_visuals(robot)
    add_output_bridge_visuals(robot)
    add_v4_body_payloads(robot)
    update_sole_contact_frames(robot)
    masses = rebalance_body_mass(robot)
    ET.indent(robot, space="  ")

    total = sum(masses.values())
    movable = [joint for joint in robot.findall("joint") if joint.get("type") == "revolute"]
    if len(movable) != 18:
        raise RuntimeError(f"expected 18 movable joints, got {len(movable)}")
    if abs(total - TARGET_TOTAL_MASS_KG) > 1.0e-9:
        raise RuntimeError((total, TARGET_TOTAL_MASS_KG))
    return robot


def main() -> int:
    robot = gen_urdf()
    OUT_ROOT.mkdir(parents=True, exist_ok=True)
    copy_meshes(robot)
    ET.ElementTree(robot).write(OUT_URDF, encoding="utf-8", xml_declaration=True)
    masses = {
        str(link.get("name")): float(link.find("./inertial/mass").get("value"))
        for link in robot.findall("link")
        if link.find("./inertial/mass") is not None
    }
    payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.mass_inertia_gate.v1",
        "urdf": OUT_URDF.relative_to(ROOT).as_posix(),
        "movable_joint_count": len([joint for joint in robot.findall("joint") if joint.get("type") == "revolute"]),
        "nominal_total_mass_kg": sum(masses.values()),
        "hard_mass_limit_kg": 3.0,
        "margin_to_limit_kg": 3.0 - sum(masses.values()),
        "mass_gate": "PASS" if sum(masses.values()) <= 3.0 else "FAIL",
        "inertial_gate": "PASS" if all(value > 0.0 for value in masses.values()) else "FAIL",
        "link_masses_kg": masses,
        "confidence": "engineering_estimate_pending_first_article_scale_and_swing_identification",
        "truth_boundary": "Nominal RL mass/inertia is complete and internally consistent, but physical release remains HOLD until the printed first article, purchased payload and fasteners are weighed and identified.",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(OUT_URDF)
    print(f"movable_joints={payload['movable_joint_count']} total_mass_kg={payload['nominal_total_mass_kg']:.9f}")
    print(REPORT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
