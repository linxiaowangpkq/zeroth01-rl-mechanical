"""Build the RL-ready v2-minimal URDF from the released v1 mechanism."""

from __future__ import annotations

import copy
import json
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np

from minimal_v2_common import HEAD_Z_SHIFT_MM


ROOT = Path(__file__).resolve().parents[2]
BASE_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v1"
BASE_URDF = BASE_ROOT / "zeroth01_physical_mount_v1.urdf"
OUTPUT_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v2_minimal"
OUTPUT_URDF = OUTPUT_ROOT / "zeroth01_physical_mount_v2_minimal.urdf"
PART_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
REPLACEMENT_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements"
MANIFEST = ROOT / "reports" / "physical_mount_v2_minimal" / "component_manifest.json"
MASS_PROPERTIES = ROOT / "generated" / "config" / "physical_mount_v2_minimal_mass_properties.json"
ELECTRONICS = ROOT / "config" / "round_v1_electronics_layout_source.json"
SERVO_MANIFEST = ROOT / "reports" / "physical_mount_v1" / "servo_component_manifest.json"
BODY_LINK = "Z_BOT2_MASTER_BODY_SKELETON"
TARGET_SERVO_MASS_KG = 0.0745

MODULE_CENTERS_M = {
    "eye_display_module": (0.0, -0.043, (125.0 + HEAD_Z_SHIFT_MM) / 1000.0),
    "camera_module": (0.0, -0.033, (160.0 + HEAD_Z_SHIFT_MM) / 1000.0),
    "tof_module": (0.029, -0.038, (163.0 + HEAD_Z_SHIFT_MM) / 1000.0),
    "compute_module": (0.0, 0.042, -0.002),
    "battery_pack": (0.0, 0.028, -0.052),
    "imu_module": (0.0, 0.012, 0.018),
}

MODULE_COLORS = {
    "eye_display_module": "0.0 0.72 0.85 1",
    "camera_module": "0.18 0.49 0.20 1",
    "tof_module": "0.67 0.0 1.0 1",
    "compute_module": "1.0 0.57 0.0 1",
    "battery_pack": "0.84 0.0 0.98 1",
    "imu_module": "0.39 0.87 0.09 1",
}


def _vector(text: str | None) -> list[float]:
    values = [float(value) for value in (text or "0 0 0").split()]
    if len(values) != 3:
        raise ValueError(text)
    return values


def _rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    ry = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    rz = np.array(((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0)))
    return rz @ ry @ rx


def _inertia_matrix(element: ET.Element) -> np.ndarray:
    return np.array(
        (
            (float(element.get("ixx", "0")), float(element.get("ixy", "0")), float(element.get("ixz", "0"))),
            (float(element.get("ixy", "0")), float(element.get("iyy", "0")), float(element.get("iyz", "0"))),
            (float(element.get("ixz", "0")), float(element.get("iyz", "0")), float(element.get("izz", "0"))),
        )
    )


def _dict_inertia(values) -> np.ndarray:
    return np.array(
        (
            (float(values["ixx"]), float(values["ixy"]), float(values["ixz"])),
            (float(values["ixy"]), float(values["iyy"]), float(values["iyz"])),
            (float(values["ixz"]), float(values["iyz"]), float(values["izz"])),
        )
    )


def _parallel_axis(mass: float, offset) -> np.ndarray:
    vector = np.array(offset, dtype=float)
    return mass * (float(vector @ vector) * np.identity(3) - np.outer(vector, vector))


def _set_inertial(link: ET.Element, mass: float, com, inertia: np.ndarray) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        inertial = ET.SubElement(link, "inertial")
    for child in list(inertial):
        inertial.remove(child)
    ET.SubElement(
        inertial,
        "origin",
        xyz=" ".join(f"{float(value):.12g}" for value in com),
        rpy="0 0 0",
    )
    ET.SubElement(inertial, "mass", value=f"{mass:.12g}")
    ET.SubElement(
        inertial,
        "inertia",
        ixx=f"{inertia[0, 0]:.12g}",
        iyy=f"{inertia[1, 1]:.12g}",
        izz=f"{inertia[2, 2]:.12g}",
        ixy=f"{inertia[0, 1]:.12g}",
        ixz=f"{inertia[0, 2]:.12g}",
        iyz=f"{inertia[1, 2]:.12g}",
    )


def _baseline_body(link: ET.Element) -> dict[str, object]:
    inertial = link.find("inertial")
    if inertial is None:
        raise ValueError(f"missing inertial: {link.get('name')}")
    mass_element = inertial.find("mass")
    inertia_element = inertial.find("inertia")
    origin = inertial.find("origin")
    if mass_element is None or inertia_element is None:
        raise ValueError(f"incomplete inertial: {link.get('name')}")
    rotation = _rpy_matrix(_vector(origin.get("rpy") if origin is not None else None))
    return {
        "mass": float(mass_element.get("value", "0")),
        "com": np.array(_vector(origin.get("xyz") if origin is not None else None)),
        "inertia": rotation @ _inertia_matrix(inertia_element) @ rotation.T,
    }


def _combine_link_inertia(link: ET.Element, additions: list[dict[str, object]]) -> None:
    bodies = [_baseline_body(link)]
    for addition in additions:
        bodies.append(
            {
                "mass": float(addition["nominal_mass_kg"]),
                "com": np.array(addition["com_m"], dtype=float),
                "inertia": _dict_inertia(addition["inertia_kg_m2_at_com"]),
            }
        )
    total_mass = sum(float(body["mass"]) for body in bodies)
    combined_com = sum(float(body["mass"]) * body["com"] for body in bodies) / total_mass
    combined = np.zeros((3, 3))
    for body in bodies:
        combined += body["inertia"] + _parallel_axis(
            float(body["mass"]), body["com"] - combined_com
        )
    _set_inertial(link, total_mass, combined_com, combined)


def _box_inertia(mass: float, size) -> np.ndarray:
    x, y, z = [float(value) for value in size]
    return np.diag(
        (
            mass * (y * y + z * z) / 12.0,
            mass * (x * x + z * z) / 12.0,
            mass * (x * x + y * y) / 12.0,
        )
    )


def _replace_mesh(link: ET.Element, old_token: str, new_filename: str, scale: str | None = None) -> None:
    changed = 0
    for mesh in link.findall("./visual/geometry/mesh") + link.findall("./collision/geometry/mesh"):
        if old_token not in str(mesh.get("filename", "")):
            continue
        mesh.set("filename", new_filename)
        if scale is None:
            mesh.attrib.pop("scale", None)
        else:
            mesh.set("scale", scale)
        changed += 1
    if changed < 2:
        raise RuntimeError(f"expected visual+collision replacement for {link.get('name')}, got {changed}")


def _replace_hand(link: ET.Element, key: str, part_mass: dict[str, object]) -> None:
    for element in list(link.findall("visual")) + list(link.findall("collision")):
        link.remove(element)
    visual = ET.SubElement(link, "visual", name=f"{key}_replacement_visual")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", filename=f"meshes/minimal/{key}.stl", scale="0.001 0.001 0.001")
    material = ET.SubElement(visual, "material", name=f"{key}_white")
    ET.SubElement(material, "color", rgba="0.969 0.973 0.980 1")
    collision = ET.SubElement(link, "collision", name=f"{key}_replacement_collision")
    ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(geometry, "mesh", filename=f"meshes/minimal/{key}.stl", scale="0.001 0.001 0.001")
    _set_inertial(
        link,
        float(part_mass["nominal_mass_kg"]),
        part_mass["com_m"],
        _dict_inertia(part_mass["inertia_kg_m2_at_com"]),
    )


def _add_part_visual(link: ET.Element, row: dict[str, object]) -> None:
    key = str(row["key"])
    visual = ET.SubElement(link, "visual", name=f"minimal_{key}_visual")
    ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "mesh", filename=f"meshes/minimal/{key}.stl", scale="0.001 0.001 0.001")
    material = ET.SubElement(visual, "material", name=f"minimal_{key}_material")
    value = str(row["color_hex"]).lstrip("#")
    channels = [int(value[index : index + 2], 16) / 255.0 for index in (0, 2, 4)]
    ET.SubElement(material, "color", rgba=" ".join(f"{value:.6g}" for value in channels) + " 1")
    if bool(row["urdf_collision"]):
        collision = ET.SubElement(link, "collision", name=f"minimal_{key}_collision")
        ET.SubElement(collision, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "mesh", filename=f"meshes/minimal/{key}.stl", scale="0.001 0.001 0.001")


def _add_electronics(robot: ET.Element, source: dict[str, object]) -> None:
    for source_name, module in source["modules"].items():
        center = MODULE_CENTERS_M[source_name]
        size = [float(value) for value in module["size_xyz_m"]]
        mass = float(module["nominal_mass_kg"])
        link_name = {
            "eye_display_module": "display_module",
            "imu_module": "torso_imu_module",
        }.get(source_name, source_name)
        link = ET.SubElement(robot, "link", name=link_name)
        _set_inertial(link, mass, (0.0, 0.0, 0.0), _box_inertia(mass, size))
        visual = ET.SubElement(link, "visual", name=f"{link_name}_controlled_envelope")
        ET.SubElement(visual, "origin", xyz="0 0 0", rpy="0 0 0")
        geometry = ET.SubElement(visual, "geometry")
        ET.SubElement(geometry, "box", size=" ".join(f"{value:.12g}" for value in size))
        material = ET.SubElement(visual, "material", name=f"{link_name}_material")
        ET.SubElement(material, "color", rgba=MODULE_COLORS[source_name])
        joint = ET.SubElement(robot, "joint", name=f"{link_name}_fixed_joint", type="fixed")
        ET.SubElement(joint, "parent", link=BODY_LINK)
        ET.SubElement(joint, "child", link=link_name)
        ET.SubElement(joint, "origin", xyz=" ".join(f"{value:.12g}" for value in center), rpy="0 0 0")

    optical_frames = {
        "camera_optical_frame": ("camera_module", (0.0, -0.0295, 0.0060)),
        "tof_optical_frame": ("tof_module", (0.0, -0.0245, 0.0030)),
    }
    for name, (parent, origin) in optical_frames.items():
        ET.SubElement(robot, "link", name=name)
        joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(
            joint,
            "origin",
            xyz=" ".join(f"{value:.12g}" for value in origin),
            rpy=f"{-math.pi / 2.0:.12g} 0 {math.pi:.12g}",
        )


def _add_contact_frames(robot: ET.Element) -> None:
    sites = {
        "left_sole_front_contact": ("FOOT", (0.032, 0.0446, 0.01695)),
        "left_sole_rear_contact": ("FOOT", (-0.045, 0.0446, 0.01695)),
        "right_sole_front_contact": ("FOOT_2", (0.032, -0.0446, -0.01695)),
        "right_sole_rear_contact": ("FOOT_2", (-0.045, -0.0446, -0.01695)),
    }
    for name, (parent, origin) in sites.items():
        ET.SubElement(robot, "link", name=name)
        joint = ET.SubElement(robot, "joint", name=f"{name}_joint", type="fixed")
        ET.SubElement(joint, "parent", link=parent)
        ET.SubElement(joint, "child", link=name)
        ET.SubElement(joint, "origin", xyz=" ".join(f"{value:.12g}" for value in origin), rpy="0 0 0")


def _adjust_trimmed_forearm(link: ET.Element, ratio: float) -> None:
    baseline = _baseline_body(link)
    carrier_mass = float(baseline["mass"]) - TARGET_SERVO_MASS_KG
    if carrier_mass <= 0.0:
        raise ValueError(f"invalid forearm mass split: {link.get('name')}")
    new_mass = TARGET_SERVO_MASS_KG + carrier_mass * ratio
    # Keep the measured v1 COM/inertia orientation and scale total inertia by
    # the mass ratio.  Exact trimmed-mesh inertias are overridden after the
    # first printed carrier is weighed; the conservative baseline tensor is
    # preferable to inventing an unmeasured polymer density for the source.
    new_inertia = baseline["inertia"] * (new_mass / float(baseline["mass"]))
    _set_inertial(link, new_mass, baseline["com"], new_inertia)


def _copy_meshes() -> None:
    if OUTPUT_ROOT.exists():
        # Do not remove the root: preserve unrelated user-created notes.  Only
        # refresh generated mesh subtrees file-by-file.
        pass
    for subdir in ("skeleton", "servos"):
        source = BASE_ROOT / "meshes" / subdir
        target = OUTPUT_ROOT / "meshes" / subdir
        target.mkdir(parents=True, exist_ok=True)
        for path in source.glob("*.stl"):
            shutil.copy2(path, target / path.name)
    minimal = OUTPUT_ROOT / "meshes" / "minimal"
    minimal.mkdir(parents=True, exist_ok=True)
    for path in PART_ROOT.glob("*.stl"):
        shutil.copy2(path, minimal / path.name)
    replacements = OUTPUT_ROOT / "meshes" / "replacements"
    replacements.mkdir(parents=True, exist_ok=True)
    for path in REPLACEMENT_ROOT.glob("*.stl"):
        shutil.copy2(path, replacements / path.name)


def gen_urdf() -> ET.Element:
    robot = copy.deepcopy(ET.parse(BASE_URDF).getroot())
    robot.set("name", "zeroth01_physical_mount_v2_minimal_rl")
    links = {str(link.get("name")): link for link in robot.findall("link")}
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    mass = json.loads(MASS_PROPERTIES.read_text(encoding="utf-8"))
    electronics = json.loads(ELECTRONICS.read_text(encoding="utf-8"))

    _replace_mesh(
        links["R_ARM_MIRROR_1"],
        "R_ARM_MIRROR_1.stl",
        "meshes/replacements/R_ARM_MIRROR_1_WRIST_TRIMMED.stl",
    )
    _replace_mesh(
        links["L_ARM_MIRROR_1"],
        "L_ARM_MIRROR_1.stl",
        "meshes/replacements/L_ARM_MIRROR_1_WRIST_TRIMMED.stl",
    )
    ratios = mass["forearm_retained_volume_ratio"]
    _adjust_trimmed_forearm(
        links["R_ARM_MIRROR_1"],
        float(ratios["R_ARM_MIRROR_1_WRIST_TRIMMED"]),
    )
    _adjust_trimmed_forearm(
        links["L_ARM_MIRROR_1"],
        float(ratios["L_ARM_MIRROR_1_WRIST_TRIMMED"]),
    )
    _replace_hand(links["FINGER_1"], "left_q_hand", mass["parts"]["left_q_hand"])
    _replace_hand(links["FINGER_1_2"], "right_q_hand", mass["parts"]["right_q_hand"])

    additions: dict[str, list[dict[str, object]]] = {}
    replacement_keys = {"left_q_hand", "right_q_hand"}
    for key, part in mass["parts"].items():
        if key in replacement_keys:
            continue
        additions.setdefault(str(part["installed_link"]), []).append(part)
    for link_name, items in additions.items():
        _combine_link_inertia(links[link_name], items)

    for row in manifest["parts"]:
        key = str(row["key"])
        if key in replacement_keys or str(row["classification"]) == "internal_payload_controlled_envelope":
            continue
        _add_part_visual(links[str(row["installed_link"])], row)

    _add_electronics(robot, electronics)
    _add_contact_frames(robot)
    ET.indent(robot, space="  ")
    return robot


def main() -> int:
    _copy_meshes()
    robot = gen_urdf()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(robot).write(OUTPUT_URDF, encoding="utf-8", xml_declaration=True)
    total_mass = sum(float(link.find("./inertial/mass").get("value")) for link in robot.findall("link") if link.find("./inertial/mass") is not None)
    summary = {
        "schema": "zeroth01.physical_mount_v2_minimal.urdf_build.v1",
        "urdf": OUTPUT_URDF.relative_to(ROOT).as_posix(),
        "revolute_joint_count": sum(joint.get("type") == "revolute" for joint in robot.findall("joint")),
        "total_mass_kg": total_mass,
        "head_z_shift_mm": HEAD_Z_SHIFT_MM,
        "old_claw_visual_count": 0,
        "replacement_hand_count": 2,
    }
    report = ROOT / "reports" / "physical_mount_v2_minimal" / "urdf_build.json"
    report.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(OUTPUT_URDF)
    print(json.dumps(summary, indent=2))
    return 0 if summary["revolute_joint_count"] == 16 else 2


if __name__ == "__main__":
    raise SystemExit(main())
