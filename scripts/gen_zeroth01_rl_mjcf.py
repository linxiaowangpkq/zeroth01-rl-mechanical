from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_ready.urdf"
DEFAULT_OUTPUT = ROOT / "generated" / "mujoco" / "zeroth01_rl_ready.xml"
GLOBAL_BOX_REPORT = ROOT / "reports" / "global_collision_box_search.json"
ELECTRONICS_LAYOUT = (
    ROOT
    / "generated"
    / "config"
    / "round_v1_electronics_sensor_layout.json"
)
DEFAULT_MODEL_NAME = "zeroth01_rl_ready_16dof"

OFFICIAL_STANDING_POSE = {
    "left_hip_pitch": 0.23,
    "left_knee_pitch": -0.741,
    "left_hip_yaw": 0.0,
    "left_hip_roll": 0.0,
    "left_ankle_pitch": -0.5,
    "right_hip_pitch": -0.23,
    "right_knee_pitch": 0.741,
    "right_ankle_pitch": 0.5,
    "right_hip_yaw": 0.0,
    "right_hip_roll": 0.0,
}

def parse_vec(text: str | None, default=(0.0, 0.0, 0.0)) -> np.ndarray:
    return np.array(
        [float(value) for value in text.split()] if text else default,
        dtype=float,
    )


def fmt_vec(values) -> str:
    return " ".join(f"{float(value):.12g}" for value in values)


def rpy_to_quaternion(rpy: np.ndarray) -> np.ndarray:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ]
    )


def inertia_data(link: ET.Element) -> tuple[float, np.ndarray, np.ndarray]:
    inertial = link.find("inertial")
    if inertial is None:
        raise ValueError(f"link lacks inertial: {link.get('name')}")
    mass = float(inertial.find("mass").get("value"))
    origin = inertial.find("origin")
    com = parse_vec(origin.get("xyz") if origin is not None else None)
    inertia = inertial.find("inertia")
    matrix = np.array(
        [
            [
                float(inertia.get("ixx")),
                float(inertia.get("ixy")),
                float(inertia.get("ixz")),
            ],
            [
                float(inertia.get("ixy")),
                float(inertia.get("iyy")),
                float(inertia.get("iyz")),
            ],
            [
                float(inertia.get("ixz")),
                float(inertia.get("iyz")),
                float(inertia.get("izz")),
            ],
        ]
    )
    return mass, com, matrix


def parallel_axis(mass: float, displacement: np.ndarray) -> np.ndarray:
    return mass * (
        float(displacement @ displacement) * np.eye(3)
        - np.outer(displacement, displacement)
    )


def aggregate_base_and_torso(
    base_link: ET.Element,
    torso_link: ET.Element,
) -> tuple[float, np.ndarray, np.ndarray]:
    base_mass, base_com, base_inertia = inertia_data(base_link)
    torso_mass, torso_com, torso_inertia = inertia_data(torso_link)
    total_mass = base_mass + torso_mass
    total_com = (
        base_mass * base_com + torso_mass * torso_com
    ) / total_mass
    total_inertia = (
        base_inertia
        + parallel_axis(base_mass, base_com - total_com)
        + torso_inertia
        + parallel_axis(torso_mass, torso_com - total_com)
    )
    return total_mass, total_com, total_inertia


def add_inertial(
    body: ET.Element,
    mass: float,
    com: np.ndarray,
    inertia: np.ndarray,
) -> None:
    ET.SubElement(
        body,
        "inertial",
        {
            "pos": fmt_vec(com),
            "mass": f"{mass:.12g}",
            "fullinertia": fmt_vec(
                [
                    inertia[0, 0],
                    inertia[1, 1],
                    inertia[2, 2],
                    inertia[0, 1],
                    inertia[0, 2],
                    inertia[1, 2],
                ]
            ),
        },
    )


def geometry_attributes(
    geometry: ET.Element,
    mesh_names: dict[int, str],
) -> dict[str, str]:
    mesh = geometry.find("mesh")
    if mesh is not None:
        return {"type": "mesh", "mesh": mesh_names[id(mesh)]}
    box = geometry.find("box")
    if box is not None:
        full_size = parse_vec(box.get("size"))
        return {"type": "box", "size": fmt_vec(full_size * 0.5)}
    sphere = geometry.find("sphere")
    if sphere is not None:
        return {"type": "sphere", "size": sphere.get("radius", "0")}
    cylinder = geometry.find("cylinder")
    if cylinder is not None:
        return {
            "type": "cylinder",
            "size": (
                f"{cylinder.get('radius', '0')} "
                f"{float(cylinder.get('length', '0')) * 0.5:.12g}"
            ),
        }
    raise ValueError("unsupported URDF geometry")


def add_link_geometries(
    body: ET.Element,
    link: ET.Element,
    mesh_names: dict[int, str],
) -> None:
    name = link.get("name", "")
    for index, visual in enumerate(link.findall("visual")):
        geometry = visual.find("geometry")
        if geometry is None:
            continue
        color = visual.find("./material/color")
        attributes = {
            "name": f"{name}_visual_{index}",
            "class": "visual",
        }
        attributes.update(geometry_attributes(geometry, mesh_names))
        origin = visual.find("origin")
        if origin is not None:
            attributes["pos"] = fmt_vec(parse_vec(origin.get("xyz")))
            attributes["quat"] = fmt_vec(
                rpy_to_quaternion(parse_vec(origin.get("rpy")))
            )
        if color is not None and color.get("rgba"):
            attributes["rgba"] = color.get("rgba")
        ET.SubElement(body, "geom", attributes)
    for index, collision in enumerate(link.findall("collision")):
        geometry = collision.find("geometry")
        if geometry is None:
            continue
        attributes = {
            "name": f"{name}_collision_{index}",
            "class": "collision",
        }
        attributes.update(geometry_attributes(geometry, mesh_names))
        origin = collision.find("origin")
        if origin is not None:
            attributes["pos"] = fmt_vec(parse_vec(origin.get("xyz")))
            attributes["quat"] = fmt_vec(
                rpy_to_quaternion(parse_vec(origin.get("rpy")))
            )
        ET.SubElement(
            body,
            "geom",
            attributes,
        )


def gen_mjcf(
    urdf_path: Path = URDF,
    model_name: str = DEFAULT_MODEL_NAME,
    base_height_m: float = 0.32,
) -> ET.Element:
    urdf_root = ET.parse(urdf_path).getroot()
    electronics_layout = (
        json.loads(ELECTRONICS_LAYOUT.read_text(encoding="utf-8"))
        if ELECTRONICS_LAYOUT.is_file()
        else {}
    )
    links = {
        link.get("name", ""): link for link in urdf_root.findall("link")
    }
    joints = {
        joint.get("name", ""): joint for joint in urdf_root.findall("joint")
    }
    moving = {
        name: joint
        for name, joint in joints.items()
        if joint.get("type") in {"revolute", "continuous"}
    }
    if len(moving) != 16:
        raise ValueError(f"expected 16 moving joints, got {len(moving)}")

    parent_joint_by_child = {
        joint.find("child").get("link", ""): joint
        for joint in urdf_root.findall("joint")
    }
    children_by_parent: dict[str, list[ET.Element]] = {}
    for joint in urdf_root.findall("joint"):
        parent = joint.find("parent").get("link", "")
        children_by_parent.setdefault(parent, []).append(joint)

    model = ET.Element("mujoco", {"model": model_name})
    ET.SubElement(
        model,
        "compiler",
        {
            "angle": "radian",
            "meshdir": "../urdf/meshes",
            "autolimits": "true",
            "inertiafromgeom": "false",
            "balanceinertia": "false",
        },
    )
    ET.SubElement(
        model,
        "option",
        {
            "timestep": "0.002",
            "integrator": "implicitfast",
            "solver": "Newton",
            "iterations": "50",
            "gravity": "0 0 -9.81",
        },
    )
    default = ET.SubElement(model, "default")
    ET.SubElement(
        default,
        "joint",
        {
            "limited": "true",
            "damping": "0.53",
            "armature": "0.008793405204572328",
            "frictionloss": "0.001",
        },
    )
    ET.SubElement(
        default,
        "motor",
        {
            "ctrllimited": "true",
            "ctrlrange": "-2 2",
            "forcelimited": "true",
            "forcerange": "-2 2",
        },
    )
    visual_default = ET.SubElement(default, "default", {"class": "visual"})
    ET.SubElement(
        visual_default,
        "geom",
        {"group": "2", "contype": "0", "conaffinity": "0"},
    )
    collision_default = ET.SubElement(
        default, "default", {"class": "collision"}
    )
    ET.SubElement(
        collision_default,
        "geom",
        {
            "group": "3",
            "contype": "1",
            "conaffinity": "1",
            "condim": "4",
            "friction": "0.9 0.02 0.001",
            "margin": "0.001",
        },
    )

    asset = ET.SubElement(model, "asset")
    mesh_names: dict[int, str] = {}
    for link_name, link in links.items():
        mesh_index = 0
        for role in ("visual", "collision"):
            for element in link.findall(role):
                mesh = element.find("./geometry/mesh")
                if mesh is None or id(mesh) in mesh_names:
                    continue
                filename = mesh.get("filename", "").replace("\\", "/")
                if filename.startswith("meshes/"):
                    filename = filename[len("meshes/") :]
                mesh_name = f"{link_name}_{role}_{mesh_index}_mesh"
                mesh_index += 1
                attributes = {"name": mesh_name, "file": filename}
                if mesh.get("scale"):
                    attributes["scale"] = mesh.get("scale")
                ET.SubElement(asset, "mesh", attributes)
                mesh_names[id(mesh)] = mesh_name

    worldbody = ET.SubElement(model, "worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "ground",
            "type": "plane",
            "size": "0 0 0.05",
            "condim": "4",
            "friction": "0.9 0.02 0.001",
        },
    )
    ET.SubElement(
        worldbody,
        "light",
        {
            "name": "key_light",
            "directional": "true",
            "pos": "0 -1 2",
            "dir": "0 0 -1",
        },
    )
    torso_body = ET.SubElement(
        worldbody,
        "body",
        {"name": "Torso", "pos": f"0 0 {base_height_m:.12g}"},
    )
    ET.SubElement(torso_body, "freejoint", {"name": "floating_base"})
    mass, com, inertia = aggregate_base_and_torso(
        links["base"], links["Torso"]
    )
    add_inertial(torso_body, mass, com, inertia)
    add_link_geometries(torso_body, links["Torso"], mesh_names)
    qpos_order: list[str] = []

    def add_children(parent_link: str, parent_body: ET.Element) -> None:
        for joint in children_by_parent.get(parent_link, []):
            child_name = joint.find("child").get("link", "")
            if child_name == "Torso":
                continue
            origin = joint.find("origin")
            xyz = parse_vec(origin.get("xyz") if origin is not None else None)
            rpy = parse_vec(origin.get("rpy") if origin is not None else None)
            child_body = ET.SubElement(
                parent_body,
                "body",
                {
                    "name": child_name,
                    "pos": fmt_vec(xyz),
                    "quat": fmt_vec(rpy_to_quaternion(rpy)),
                },
            )
            child_link = links[child_name]
            if child_link.find("inertial") is not None:
                child_mass, child_com, child_inertia = inertia_data(
                    child_link
                )
                add_inertial(
                    child_body, child_mass, child_com, child_inertia
                )
            if joint.get("type") in {"revolute", "continuous"}:
                name = joint.get("name", "")
                axis = joint.find("axis")
                limit = joint.find("limit")
                dynamics = joint.find("dynamics")
                ET.SubElement(
                    child_body,
                    "joint",
                    {
                        "name": name,
                        "type": "hinge",
                        "axis": axis.get("xyz"),
                        "range": (
                            f"{limit.get('lower')} {limit.get('upper')}"
                        ),
                        "damping": dynamics.get("damping"),
                        "frictionloss": dynamics.get("friction"),
                        "armature": "0.008793405204572328",
                    },
                )
                qpos_order.append(name)
            add_link_geometries(child_body, child_link, mesh_names)
            if child_name == "imu_module":
                ET.SubElement(
                    child_body,
                    "site",
                    {
                        "name": "imu",
                        "pos": "0 0 0",
                        "size": "0.004",
                        "rgba": "0 1 0 1",
                    },
                )
            if child_name == "camera_module":
                camera_frame = electronics_layout.get("frames", {}).get(
                    "camera_optical_frame", {}
                )
                camera_pos = camera_frame.get(
                    "origin_xyz_m", [0.0, -0.011, 0.0]
                )
                ET.SubElement(
                    child_body,
                    "camera",
                    {
                        "name": "head_camera",
                        "pos": fmt_vec(camera_pos),
                        "xyaxes": "-1 0 0 0 0 1",
                        "fovy": "75",
                        "mode": "fixed",
                    },
                )
            contact_sites = electronics_layout.get(
                "foot_contact_sites", {}
            )
            for site_name, site in contact_sites.items():
                if site.get("link") != child_name:
                    continue
                ET.SubElement(
                    child_body,
                    "site",
                    {
                        "name": site_name,
                        "type": "box",
                        "pos": fmt_vec(site["center_xyz_m"]),
                        "size": fmt_vec(site["half_size_xyz_m"]),
                        "rgba": "0.1 0.8 0.8 0.35",
                    },
                )
            add_children(child_name, child_body)

    add_children("Torso", torso_body)

    actuators = ET.SubElement(model, "actuator")
    for name in qpos_order:
        ET.SubElement(
            actuators,
            "motor",
            {
                "name": f"{name}_motor",
                "joint": name,
                "gear": "1",
            },
        )

    sensors = ET.SubElement(model, "sensor")
    ET.SubElement(
        sensors,
        "framequat",
        {"name": "base_orientation", "objtype": "site", "objname": "imu"},
    )
    ET.SubElement(
        sensors,
        "gyro",
        {"name": "base_angular_velocity", "site": "imu", "noise": "0.005"},
    )
    ET.SubElement(
        sensors,
        "accelerometer",
        {"name": "base_linear_acceleration", "site": "imu", "noise": "0.01"},
    )
    for site_name in electronics_layout.get("foot_contact_sites", {}):
        ET.SubElement(
            sensors,
            "touch",
            {"name": f"{site_name}_touch", "site": site_name},
        )

    contact = ET.SubElement(model, "contact")
    collision_audit = json.loads(GLOBAL_BOX_REPORT.read_text(encoding="utf-8"))
    allowed_assembly_overlaps = []
    for value in collision_audit.get("allowed_neutral_pairs", []):
        names = [name.strip() for name in str(value).split("::")]
        if len(names) != 2 or not all(names):
            raise ValueError(f"invalid neutral overlap pair: {value!r}")
        allowed_assembly_overlaps.append((names[0], names[1]))
    for body1, body2 in allowed_assembly_overlaps:
        ET.SubElement(
            contact, "exclude", {"body1": body1, "body2": body2}
        )

    keyframe = ET.SubElement(model, "keyframe")
    standing = [
        0.0,
        0.0,
        base_height_m,
        1.0,
        0.0,
        0.0,
        0.0,
    ] + [float(OFFICIAL_STANDING_POSE.get(name, 0.0)) for name in qpos_order]
    ET.SubElement(
        keyframe,
        "key",
        {
            "name": "official_standing",
            "qpos": fmt_vec(standing),
        },
    )
    model.insert(
        0,
        ET.Comment(
            f"Generated from {urdf_path.name}. The floating base is "
            "attached to Torso; the 1 g URDF frame link is rigidly aggregated "
            "into the Torso inertia. Four assumed electronics modules retain "
            "their fixed-body masses, the camera optical frame is massless, "
            "and four sole pressure sites plus the torso IMU are exposed. "
            "Seven source-mesh assembly overlaps are explicit contact "
            "exclusions; all other self contacts remain enabled."
        ),
    )
    return model


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate the native floating-base Zeroth-01 MuJoCo model."
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--urdf", type=Path, default=URDF)
    parser.add_argument("--model-name", default=DEFAULT_MODEL_NAME)
    parser.add_argument("--base-height-m", type=float, default=0.32)
    args = parser.parse_args()
    output = args.output.resolve()
    model = gen_mjcf(
        urdf_path=args.urdf.resolve(),
        model_name=str(args.model_name),
        base_height_m=float(args.base_height_m),
    )
    ET.indent(model, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(model).write(
        output, encoding="utf-8", xml_declaration=True
    )
    print(f"MJCF={output}")


if __name__ == "__main__":
    main()
