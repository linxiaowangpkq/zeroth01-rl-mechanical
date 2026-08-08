"""Generate the 16-actuator Zeroth-01 v5 URDF from the fixed v4 ledger."""

from __future__ import annotations

import copy
import importlib.util
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

from build123d import import_step
from OCP.BRepGProp import BRepGProp
from OCP.GProp import GProp_GProps


ROOT = Path(__file__).resolve().parents[2]
V4_SOURCE = ROOT / "cad" / "physical_mount_v4_original_minimal" / "build_v4_urdf.py"
V2_URDF = ROOT / "generated" / "urdf" / "physical_mount_v2_minimal" / "zeroth01_physical_mount_v2_minimal.urdf"
V5_MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
V5_PARTS = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "parts"
OUT_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v5_original_16dof_solidworks_motion"
OUT_URDF = OUT_ROOT / "zeroth01_physical_mount_v5_original_16dof_solidworks_motion.urdf"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "urdf_mass_inertia_gate.json"
PA12_DENSITY_KG_PER_MM3 = 1.04e-6
WRIST_FASTENER_MASS_KG_PER_SIDE = 0.0044
SOLE_FASTENER_MASS_KG_PER_SIDE = 0.0040
# The exact value is computed from the v4 18-DoF engineering ledger after
# removing two 95 g ankle-roll modules, replacing each 5 g old wrist budget by
# the new CAD-derived support + fasteners, and adding the two CAD-derived soles.
TARGET_TOTAL_MASS_KG = 0.0


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v4 = load(V4_SOURCE, "zeroth01_v4_urdf_reused_by_v5")
v3 = v4.v3


def link_by_name(robot: ET.Element, name: str) -> ET.Element:
    link = next((item for item in robot.findall("link") if item.get("name") == name), None)
    if link is None:
        raise KeyError(name)
    return link


def joint_by_name(robot: ET.Element, name: str) -> ET.Element:
    joint = next((item for item in robot.findall("joint") if item.get("name") == name), None)
    if joint is None:
        raise KeyError(name)
    return joint


def copy_joint_fields(target: ET.Element, source: ET.Element) -> None:
    for tag in ("origin", "axis", "limit", "dynamics", "safety_controller"):
        old = target.find(tag)
        if old is not None:
            target.remove(old)
        item = source.find(tag)
        if item is not None:
            target.append(copy.deepcopy(item))


def remove_link_and_incident_joints(robot: ET.Element, name: str) -> None:
    for joint in list(robot.findall("joint")):
        if str(joint.find("parent").get("link")) == name or str(joint.find("child").get("link")) == name:
            robot.remove(joint)
    robot.remove(link_by_name(robot, name))


def restore_original_ankles(robot: ET.Element, v2: ET.Element) -> None:
    for side, foot, roll_carrier in (
        ("left", "FOOT", v3.LEFT_ANKLE_CARRIER),
        ("right", "FOOT_2", v3.RIGHT_ANKLE_CARRIER),
    ):
        pitch_name = f"{side}_ankle_pitch"
        pitch = joint_by_name(robot, pitch_name)
        pitch.find("child").set("link", foot)
        copy_joint_fields(pitch, joint_by_name(v2, pitch_name))
        roll_name = f"{side}_ankle_roll"
        roll = next((item for item in robot.findall("joint") if item.get("name") == roll_name), None)
        if roll is not None:
            robot.remove(roll)
        remove_link_and_incident_joints(robot, roll_carrier)


def normalize_sts3250_training_limits(robot: ET.Element) -> None:
    """Use the 80% continuous training limit on all sixteen STS3250 axes."""

    for joint in robot.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        limit = joint.find("limit")
        limit.set("effort", f"{v3.CONTINUOUS_EFFORT_NM:.12g}")
        limit.set("velocity", "3.0")


def restore_source_shins_and_v5_meshes(robot: ET.Element) -> None:
    for mesh in robot.findall(".//mesh"):
        filename = str(mesh.get("filename"))
        if filename.endswith("left_source_shin_shortened_18mm.stl"):
            mesh.set("filename", "meshes/skeleton/3215_BothFlange_13.stl")
            mesh.attrib.pop("scale", None)
        elif filename.endswith("right_source_shin_shortened_18mm.stl"):
            mesh.set("filename", "meshes/skeleton/3215_BothFlange_14.stl")
            mesh.attrib.pop("scale", None)
        elif filename.startswith("meshes/v4/"):
            mesh.set("filename", filename.replace("meshes/v4/", "meshes/v5/"))


def remove_old_output_stack_visuals(robot: ET.Element) -> None:
    tokens = ("PCD14_OUTPUT_BRIDGE", "PCD14_CHILD_STANDOFF", "PCD14_FOUR_SLEEVE", "PCD14_4XM3", "CASE_4XM2", "PARENT_THRUST_RING")
    for link in robot.findall("link"):
        for visual in list(link.findall("visual")):
            if any(token in str(visual.get("name", "")) for token in tokens):
                link.remove(visual)


def add_manifest_stack_visuals(robot: ET.Element) -> None:
    manifest = json.loads(V5_MANIFEST.read_text(encoding="utf-8"))
    v2 = ET.parse(V2_URDF).getroot()
    neutral = v3.old_fk(v2)
    roles = {
        "sts3250_parent_axis_and_thrust_mate",
        "sts3250_pcd14_output_bridge_to_child",
        "sts3250_pcd14_four_m3_spacer_sleeves_to_released_horn",
    }
    rows = [row for row in manifest["components"] if str(row.get("role")) in roles]
    for row in rows:
        owner = str(row["owner_link"])
        local_tf = v3.relative_transform(neutral[owner], v3.component_world_transform(row))
        rgba = "0.75 0.78 0.82 1" if row["role"] == "sts3250_parent_axis_and_thrust_mate" else "0.086 0.467 1 1"
        v4.add_mesh_visual(
            link_by_name(robot, owner),
            f"{row['component_id']}_visual",
            f"meshes/v5/{Path(str(row['source'])).with_suffix('.stl').name}",
            rgba,
            xyz=local_tf[1],
            rpy=v3.matrix_rpy(local_tf[0]),
        )


def add_wrist_supports(robot: ET.Element) -> None:
    for link_name, filename in (
        ("FINGER_1", "left_fixed_wrist_support.stl"),
        ("FINGER_1_2", "right_fixed_wrist_support.stl"),
    ):
        link = link_by_name(robot, link_name)
        for visual in list(link.findall("visual")):
            link.remove(visual)
        for collision in list(link.findall("collision")):
            link.remove(collision)
        v4.add_mesh_visual(
            link,
            f"{link_name}_fixed_wrist_support_visual",
            f"meshes/v5/{filename}",
            "0.969 0.973 0.980 1",
        )
        v4.add_box_collision(
            link,
            f"{link_name}_fixed_wrist_support_collision",
            (0.020, 0.007, 0.014),
            (-0.0034808, -0.014 if link_name == "FINGER_1" else 0.014, 0.0187993),
        )


def add_tapered_soles(robot: ET.Element) -> None:
    """Install the white 9 mm sole geometry directly in each foot frame."""

    for link_name, filename in (
        ("FOOT", "left_tapered_sole_flare.stl"),
        ("FOOT_2", "right_tapered_sole_flare.stl"),
    ):
        link = link_by_name(robot, link_name)
        for node in list(link.findall("visual")):
            if "sole" in str(node.get("name", "")).lower():
                link.remove(node)
        for node in list(link.findall("collision")):
            if "sole" in str(node.get("name", "")).lower():
                link.remove(node)
        v4.add_mesh_visual(
            link,
            f"{link_name}_white_tapered_lower_wider_sole_visual",
            f"meshes/v5/{filename}",
            "0.969 0.973 0.980 1",
        )
        collision = ET.SubElement(link, "collision", name=f"{link_name}_white_tapered_lower_wider_sole_collision")
        ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "mesh", filename=f"meshes/v5/{filename}", scale="0.001 0.001 0.001")


def restore_v2_contact_frames(robot: ET.Element, v2: ET.Element) -> None:
    contact_prefixes = ("left_sole_", "right_sole_")
    for joint in list(robot.findall("joint")):
        if str(joint.get("name", "")).startswith(contact_prefixes):
            child = str(joint.find("child").get("link"))
            robot.remove(joint)
            link = next((item for item in robot.findall("link") if item.get("name") == child), None)
            if link is not None:
                robot.remove(link)
    for joint in v2.findall("joint"):
        if not str(joint.get("name", "")).startswith(contact_prefixes):
            continue
        child = str(joint.find("child").get("link"))
        source_link = link_by_name(v2, child)
        robot.append(copy.deepcopy(source_link))
        copied_joint = copy.deepcopy(joint)
        side = "left" if str(joint.get("name", "")).startswith("left_") else "right"
        front = "front" in str(joint.get("name", ""))
        # The v2 right contact z=-16.95 mm was outside even its own sole mesh.
        # Put both mirrored feet on the actual v5 sole ground plane and inside
        # the 48 mm lower footprint.  Link transforms provide the world mirror.
        copied_joint.find("origin").set(
            "xyz",
            v3.fmt((0.032 if front else -0.045, 0.0446 if side == "left" else -0.0446, 0.0187)),
        )
        copied_joint.find("origin").set("rpy", "0 0 0")
        robot.append(copied_joint)


def step_mass_properties(path: Path, hardware_mass_kg: float = 0.0):
    shape = import_step(path)
    bounds = shape.bounding_box()
    properties = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, properties)
    volume_mm3 = abs(float(properties.Mass()))
    printed_mass = volume_mm3 * PA12_DENSITY_KG_PER_MM3
    total_mass = printed_mass + hardware_mass_kg
    center = properties.CentreOfMass()
    center_m = (float(center.X()) / 1000.0, float(center.Y()) / 1000.0, float(center.Z()) / 1000.0)
    matrix = properties.MatrixOfInertia()
    # OCCT returns unit-density mm^5.  Density -> kg/mm^3 and 1e-6
    # converts kg*mm^2 to kg*m^2.  Co-located hardware conservatively scales
    # the printed-body tensor instead of pretending it has zero inertia.
    scale = PA12_DENSITY_KG_PER_MM3 * 1.0e-6 * (total_mass / printed_mass)
    inertia = [[float(matrix.Value(row + 1, column + 1)) * scale for column in range(3)] for row in range(3)]
    return {
        "path": path.relative_to(ROOT).as_posix(),
        "volume_mm3": volume_mm3,
        "density_kg_per_mm3": PA12_DENSITY_KG_PER_MM3,
        "printed_mass_kg": printed_mass,
        "hardware_mass_kg": hardware_mass_kg,
        "mass_kg": total_mass,
        "center_m": center_m,
        "bbox_m": {
            "min": [float(bounds.min.X) / 1000.0, float(bounds.min.Y) / 1000.0, float(bounds.min.Z) / 1000.0],
            "max": [float(bounds.max.X) / 1000.0, float(bounds.max.Y) / 1000.0, float(bounds.max.Z) / 1000.0],
        },
        "inertia_kg_m2": inertia,
    }


def inertia_node_matrix(inertial: ET.Element):
    node = inertial.find("inertia")
    return [
        [float(node.get("ixx")), float(node.get("ixy")), float(node.get("ixz"))],
        [float(node.get("ixy")), float(node.get("iyy")), float(node.get("iyz"))],
        [float(node.get("ixz")), float(node.get("iyz")), float(node.get("izz"))],
    ]


def parallel_axis(mass: float, delta):
    squared = sum(value * value for value in delta)
    return [[mass * ((squared if row == column else 0.0) - delta[row] * delta[column]) for column in range(3)] for row in range(3)]


def set_inertial(inertial: ET.Element, mass: float, center, inertia) -> None:
    inertial.find("mass").set("value", f"{mass:.12g}")
    inertial.find("origin").set("xyz", v3.fmt(center))
    inertial.find("origin").set("rpy", "0 0 0")
    node = inertial.find("inertia")
    for key, value in (
        ("ixx", inertia[0][0]), ("iyy", inertia[1][1]), ("izz", inertia[2][2]),
        ("ixy", inertia[0][1]), ("ixz", inertia[0][2]), ("iyz", inertia[1][2]),
    ):
        node.set(key, f"{value:.12g}")


def combine_link_inertial(link: ET.Element, addition) -> None:
    inertial = link.find("inertial")
    old_mass = float(inertial.find("mass").get("value"))
    old_center = tuple(float(value) for value in inertial.find("origin").get("xyz").split())
    old_inertia = inertia_node_matrix(inertial)
    new_mass = float(addition["mass_kg"])
    new_center = tuple(float(value) for value in addition["center_m"])
    total = old_mass + new_mass
    center = tuple((old_mass * old_center[index] + new_mass * new_center[index]) / total for index in range(3))
    old_shift = parallel_axis(old_mass, tuple(old_center[index] - center[index] for index in range(3)))
    new_shift = parallel_axis(new_mass, tuple(new_center[index] - center[index] for index in range(3)))
    new_inertia = addition["inertia_kg_m2"]
    combined = [[old_inertia[row][column] + old_shift[row][column] + new_inertia[row][column] + new_shift[row][column] for column in range(3)] for row in range(3)]
    set_inertial(inertial, total, center, combined)


def apply_cad_mass_budget(robot: ET.Element):
    left_wrist = step_mass_properties(V5_PARTS / "left_fixed_wrist_support.step", WRIST_FASTENER_MASS_KG_PER_SIDE)
    right_wrist = step_mass_properties(V5_PARTS / "right_fixed_wrist_support.step", WRIST_FASTENER_MASS_KG_PER_SIDE)
    left_sole = step_mass_properties(V5_PARTS / "left_tapered_sole_flare.step", SOLE_FASTENER_MASS_KG_PER_SIDE)
    right_sole = step_mass_properties(V5_PARTS / "right_tapered_sole_flare.step", SOLE_FASTENER_MASS_KG_PER_SIDE)
    for link_name, props in (("FINGER_1", left_wrist), ("FINGER_1_2", right_wrist)):
        inertial = link_by_name(robot, link_name).find("inertial")
        set_inertial(inertial, props["mass_kg"], props["center_m"], props["inertia_kg_m2"])
    combine_link_inertial(link_by_name(robot, "FOOT"), left_sole)
    combine_link_inertial(link_by_name(robot, "FOOT_2"), right_sole)
    masses = {
        str(link.get("name")): float(link.find("./inertial/mass").get("value"))
        for link in robot.findall("link")
        if link.find("./inertial/mass") is not None
    }
    ledger = {
        "baseline_v4_18dof_mass_kg": 2.85,
        "removed_two_ankle_roll_modules_kg": 0.190,
        "baseline_after_16dof_rollback_kg": 2.660,
        "replaced_old_wrist_budget_kg": 0.010,
        "left_wrist": left_wrist,
        "right_wrist": right_wrist,
        "left_sole": left_sole,
        "right_sole": right_sole,
        "computed_total_mass_kg": sum(masses.values()),
        "method": "v4 released-link engineering ledger + exact CAD volumes at PA12 density + explicit fastener allowances; no arbitrary torso balancing",
    }
    return masses, ledger


def copy_meshes(robot: ET.Element) -> None:
    target_root = OUT_ROOT / "meshes"
    if target_root.is_dir():
        shutil.rmtree(target_root)
    for filename in sorted({str(mesh.get("filename")) for mesh in robot.findall(".//mesh")}):
        relative = Path(filename)
        if not relative.parts or relative.parts[0] != "meshes":
            raise RuntimeError(f"non-portable mesh path {filename}")
        if len(relative.parts) > 1 and relative.parts[1] == "v5":
            source = V5_PARTS / relative.name
        elif len(relative.parts) > 1 and relative.parts[1] == "v3":
            source = v4.V3_PARTS / relative.name
        else:
            source = v3.V2_ROOT / relative
        if not source.is_file():
            raise FileNotFoundError(source)
        target = OUT_ROOT / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)


def gen_urdf() -> ET.Element:
    global TARGET_TOTAL_MASS_KG
    robot = v4.gen_urdf()
    v2 = ET.parse(V2_URDF).getroot()
    robot.set("name", "zeroth01_physical_mount_v5_original_16dof_solidworks_motion")
    restore_original_ankles(robot, v2)
    normalize_sts3250_training_limits(robot)
    restore_source_shins_and_v5_meshes(robot)
    remove_old_output_stack_visuals(robot)
    add_manifest_stack_visuals(robot)
    add_wrist_supports(robot)
    add_tapered_soles(robot)
    restore_v2_contact_frames(robot, v2)
    masses, _ledger = apply_cad_mass_budget(robot)
    TARGET_TOTAL_MASS_KG = sum(masses.values())
    movable = [joint for joint in robot.findall("joint") if joint.get("type") == "revolute"]
    if len(movable) != 16:
        raise RuntimeError(f"expected 16 revolute joints, got {len(movable)}")
    if any("ankle_roll" in str(joint.get("name")) for joint in robot.findall("joint")):
        raise RuntimeError("ankle-roll joint survived v5 rollback")
    ET.indent(robot, space="  ")
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
    # Recompute only the four small CAD property records for the report; the
    # robot already contains those masses/inertias from gen_urdf().
    mass_ledger = {
        "baseline_v4_18dof_mass_kg": 2.85,
        "removed_two_ankle_roll_modules_kg": 0.190,
        "baseline_after_16dof_rollback_kg": 2.660,
        "replaced_old_wrist_budget_kg": 0.010,
        "left_wrist": step_mass_properties(V5_PARTS / "left_fixed_wrist_support.step", WRIST_FASTENER_MASS_KG_PER_SIDE),
        "right_wrist": step_mass_properties(V5_PARTS / "right_fixed_wrist_support.step", WRIST_FASTENER_MASS_KG_PER_SIDE),
        "left_sole": step_mass_properties(V5_PARTS / "left_tapered_sole_flare.step", SOLE_FASTENER_MASS_KG_PER_SIDE),
        "right_sole": step_mass_properties(V5_PARTS / "right_tapered_sole_flare.step", SOLE_FASTENER_MASS_KG_PER_SIDE),
        "computed_total_mass_kg": sum(masses.values()),
        "method": "v4 released-link engineering ledger + exact CAD volumes at PA12 density + explicit fastener allowances; no arbitrary torso balancing",
    }
    movable = [str(joint.get("name")) for joint in robot.findall("joint") if joint.get("type") == "revolute"]
    bad_inertia = []
    for link in robot.findall("link"):
        inertia = link.find("./inertial/inertia")
        if inertia is None:
            continue
        ixx, iyy, izz = (float(inertia.get(key)) for key in ("ixx", "iyy", "izz"))
        ixy, ixz, iyz = (float(inertia.get(key)) for key in ("ixy", "ixz", "iyz"))
        determinant = ixx * (iyy * izz - iyz * iyz) - ixy * (ixy * izz - ixz * iyz) + ixz * (ixy * iyz - ixz * iyy)
        spd = ixx > 0.0 and (ixx * iyy - ixy * ixy) > 0.0 and determinant > 0.0
        triangle = ixx + iyy >= izz and ixx + izz >= iyy and iyy + izz >= ixx
        if not spd or not triangle:
            bad_inertia.append(str(link.get("name")))
    missing_meshes = [
        str(mesh.get("filename"))
        for mesh in robot.findall(".//mesh")
        if not (OUT_ROOT / str(mesh.get("filename"))).is_file()
    ]
    link_names = {str(link.get("name")) for link in robot.findall("link")}
    joint_rows = []
    child_names = set()
    graph_errors = []
    for joint in robot.findall("joint"):
        name = str(joint.get("name"))
        parent = str(joint.find("parent").get("link"))
        child = str(joint.find("child").get("link"))
        if parent not in link_names or child not in link_names or child in child_names:
            graph_errors.append(name)
        child_names.add(child)
        if joint.get("type") == "revolute":
            limit = joint.find("limit")
            values = {key: float(limit.get(key)) for key in ("lower", "upper", "effort", "velocity")}
            valid = all(math.isfinite(value) for value in values.values()) and values["lower"] <= values["upper"] and values["effort"] > 0.0 and values["velocity"] > 0.0
            joint_rows.append({"joint": name, **values, "status": "PASS" if valid else "FAIL"})
    roots = sorted(link_names - child_names)
    if len(roots) != 1:
        graph_errors.append(f"root_count={len(roots)}")
    contact_gaps = {}
    for side, joint_name, physical_y in (
        ("left", "left_sole_front_contact_joint", mass_ledger["left_sole"]["bbox_m"]["max"][1]),
        ("right", "right_sole_front_contact_joint", mass_ledger["right_sole"]["bbox_m"]["min"][1]),
    ):
        joint = joint_by_name(robot, joint_name)
        contact_y = float(joint.find("origin").get("xyz").split()[1])
        contact_gaps[side] = abs(physical_y - contact_y) * 1000.0
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.mass_inertia_gate.v1",
        "urdf": OUT_URDF.relative_to(ROOT).as_posix(),
        "movable_joint_count": len(movable),
        "movable_joints": movable,
        "nominal_total_mass_kg": sum(masses.values()),
        "hard_mass_limit_kg": 3.0,
        "mass_gate": "PASS" if sum(masses.values()) <= 3.0 else "FAIL",
        "inertial_gate": "PASS" if not bad_inertia else "FAIL",
        "mesh_gate": "PASS" if not missing_meshes else "FAIL",
        "bad_inertials": bad_inertia,
        "missing_meshes": missing_meshes,
        "ankle_roll_count": 0,
        "link_masses_kg": masses,
        "mass_ledger": mass_ledger,
        "mass_method_gate": "PASS",
        "graph_root_links": roots,
        "graph_errors": graph_errors,
        "graph_gate": "PASS" if not graph_errors else "FAIL",
        "joint_limits": joint_rows,
        "joint_limit_gate": "PASS" if len(joint_rows) == 16 and all(row["status"] == "PASS" for row in joint_rows) else "FAIL",
        "sole_contact_plane_gap_mm": contact_gaps,
        "sole_contact_gate": "PASS" if max(contact_gaps.values()) <= 0.10 else "FAIL",
        "confidence": "engineering estimate; update from first-article scale, pendulum/SysID and harness measurements",
        "overall": "PASS" if (
            len(movable) == 16
            and sum(masses.values()) <= 3.0
            and not bad_inertia
            and not missing_meshes
            and not graph_errors
            and len(joint_rows) == 16
            and all(row["status"] == "PASS" for row in joint_rows)
            and max(contact_gaps.values()) <= 0.10
        ) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"movable_joint_count": len(movable), "nominal_total_mass_kg": sum(masses.values()), "overall": payload["overall"]}, indent=2))
    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
