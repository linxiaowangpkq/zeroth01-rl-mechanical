from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import xml.etree.ElementTree as ET


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
READY_GENERATOR = THIS_FILE.with_name("gen_zeroth01_rl_ready.py")
MASS_PROPERTIES = (
    ROOT / "generated" / "config" / "round_v1_mass_properties.json"
)
ELECTRONICS_LAYOUT = (
    ROOT
    / "generated"
    / "config"
    / "round_v1_electronics_sensor_layout.json"
)
ROBOT_NAME = "zeroth01_rl_round_v1_16dof"

CREAM = "0.909804 0.823529 0.701961 1"
TAN = "0.717647 0.529412 0.368627 1"
DARK = "0.164706 0.176471 0.196078 1"
TEAL = "0.333333 0.788235 0.776471 1"
MODULE_COLORS = {
    "camera_module": DARK,
    "imu_module": TEAL,
    "compute_module": TEAL,
    "battery_pack": TAN,
}

TORSO_VISUALS = [
    ("chest_front", "ZEROTH01_ROUND_V1_CHEST_FRONT.stl", CREAM),
    ("chest_back", "ZEROTH01_ROUND_V1_CHEST_BACK.stl", CREAM),
    ("head_front", "ZEROTH01_ROUND_V1_HEAD_FRONT.stl", CREAM),
    ("head_back", "ZEROTH01_ROUND_V1_HEAD_BACK.stl", CREAM),
    ("pelvis_front", "ZEROTH01_ROUND_V1_PELVIS_FRONT.stl", CREAM),
    ("pelvis_back", "ZEROTH01_ROUND_V1_PELVIS_BACK.stl", CREAM),
    ("muzzle_badge", "ZEROTH01_ROUND_V1_MUZZLE_BADGE.stl", TAN),
    ("visor_badge", "ZEROTH01_ROUND_V1_VISOR_BADGE.stl", DARK),
]

SOLE_VISUALS = {
    "foot_left": (
        "left_sole",
        "ZEROTH01_ROUND_V1_LEFT_SOLE.stl",
    ),
    "foot_right": (
        "right_sole",
        "ZEROTH01_ROUND_V1_RIGHT_SOLE.stl",
    ),
}


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import generator: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def find_link(root: ET.Element, name: str) -> ET.Element:
    for link in root.findall("link"):
        if link.get("name") == name:
            return link
    raise KeyError(name)


def set_link_inertial(
    link: ET.Element,
    payload: dict[str, object],
) -> None:
    inertial = link.find("inertial")
    if inertial is None:
        inertial = ET.SubElement(link, "inertial")
    origin = inertial.find("origin")
    if origin is None:
        origin = ET.SubElement(inertial, "origin")
    mass = inertial.find("mass")
    if mass is None:
        mass = ET.SubElement(inertial, "mass")
    inertia = inertial.find("inertia")
    if inertia is None:
        inertia = ET.SubElement(inertial, "inertia")

    com = payload["com_m"]
    matrix = payload["inertia_kg_m2_at_com"]
    origin.set("xyz", " ".join(f"{float(value):.12g}" for value in com))
    origin.set("rpy", "0 0 0")
    mass.set("value", f"{float(payload['mass_kg']):.12g}")
    for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz"):
        inertia.set(key, f"{float(matrix[key]):.12g}")


def add_mesh_visual(
    link: ET.Element,
    name: str,
    filename: str,
    rgba: str,
) -> None:
    visual = ET.SubElement(link, "visual", {"name": f"round_v1_{name}"})
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "mesh",
        {
            "filename": f"meshes/round_v1/{filename}",
            "scale": "0.001 0.001 0.001",
        },
    )
    material = ET.SubElement(
        visual, "material", {"name": f"round_v1_{name}_material"}
    )
    ET.SubElement(material, "color", {"rgba": rgba})


def add_box_collision(
    link: ET.Element,
    name: str,
    xyz: tuple[float, float, float],
    size: tuple[float, float, float],
) -> None:
    collision = ET.SubElement(
        link, "collision", {"name": f"round_v1_{name}_proxy"}
    )
    ET.SubElement(
        collision,
        "origin",
        {
            "xyz": " ".join(f"{value:.9g}" for value in xyz),
            "rpy": "0 0 0",
        },
    )
    geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(
        geometry,
        "box",
        {"size": " ".join(f"{value:.9g}" for value in size)},
    )


def add_box_visual(
    link: ET.Element,
    name: str,
    size: list[float],
    rgba: str,
) -> None:
    visual = ET.SubElement(link, "visual", {"name": f"{name}_envelope"})
    ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(
        geometry,
        "box",
        {"size": " ".join(f"{float(value):.12g}" for value in size)},
    )
    material = ET.SubElement(visual, "material", {"name": f"{name}_material"})
    ET.SubElement(material, "color", {"rgba": rgba})


def add_electronics_layout(
    root: ET.Element,
    layout: dict[str, object],
) -> None:
    for name, module in layout["modules"].items():
        link = ET.SubElement(root, "link", {"name": name})
        add_box_visual(
            link,
            name,
            module["size_xyz_m"],
            MODULE_COLORS[name],
        )
        set_link_inertial(
            link,
            {
                "mass_kg": module["nominal_mass_kg"],
                "com_m": [0.0, 0.0, 0.0],
                "inertia_kg_m2_at_com": (
                    module["box_inertia_kg_m2_at_com"]
                ),
            },
        )
        joint = ET.SubElement(
            root,
            "joint",
            {"name": f"{name}_mount", "type": "fixed"},
        )
        ET.SubElement(
            joint,
            "origin",
            {
                "xyz": " ".join(
                    f"{float(value):.12g}"
                    for value in module["center_xyz_m"]
                ),
                "rpy": " ".join(
                    f"{float(value):.12g}"
                    for value in module["rpy_rad"]
                ),
            },
        )
        ET.SubElement(
            joint,
            "parent",
            {"link": str(module["parent_link"])},
        )
        ET.SubElement(joint, "child", {"link": name})

    for name, frame in layout["frames"].items():
        ET.SubElement(root, "link", {"name": name})
        joint = ET.SubElement(
            root,
            "joint",
            {"name": f"{name}_joint", "type": "fixed"},
        )
        ET.SubElement(
            joint,
            "origin",
            {
                "xyz": " ".join(
                    f"{float(value):.12g}"
                    for value in frame["origin_xyz_m"]
                ),
                "rpy": " ".join(
                    f"{float(value):.12g}"
                    for value in frame["origin_rpy_rad"]
                ),
            },
        )
        ET.SubElement(
            joint,
            "parent",
            {"link": str(frame["parent_link"])},
        )
        ET.SubElement(joint, "child", {"link": name})


def gen_urdf() -> ET.Element:
    if not MASS_PROPERTIES.is_file():
        raise FileNotFoundError(
            "run export_round_v1_mass_properties.py before generating "
            f"the round-v1 URDF: {MASS_PROPERTIES}"
        )
    if not ELECTRONICS_LAYOUT.is_file():
        raise FileNotFoundError(
            "run build_round_v1_electronics_layout.py before generating "
            f"the round-v1 URDF: {ELECTRONICS_LAYOUT}"
        )
    ready = load_module(READY_GENERATOR, "zeroth01_ready_generator")
    root = ready.gen_urdf()
    root.set("name", ROBOT_NAME)
    properties = json.loads(MASS_PROPERTIES.read_text(encoding="utf-8"))
    electronics_layout = json.loads(
        ELECTRONICS_LAYOUT.read_text(encoding="utf-8")
    )

    for link_name, overlay in properties["link_overlays"].items():
        set_link_inertial(find_link(root, link_name), overlay["combined"])

    torso = find_link(root, "Torso")
    for name, filename, rgba in TORSO_VISUALS:
        add_mesh_visual(torso, name, filename, rgba)

    # Conservative analytic proxies cover the central exterior while leaving
    # shoulder and hip keep-outs open. Exact B-Rep/servo checks are reported
    # separately; URDF boxes are for stable RL collision/contact.
    add_box_collision(torso, "chest_center", (0.0, 0.005, -0.006), (0.108, 0.076, 0.130))
    add_box_collision(torso, "head", (0.0, -0.003, 0.099), (0.132, 0.066, 0.078))
    # The exact shell has 21 mm spherical hip keep-outs centered at
    # x=+/-45.65 mm, leaving only 49.3 mm of uninterrupted center material.
    # Keep 1.65 mm extra margin per side in this conservative RL proxy.
    add_box_collision(torso, "pelvis_center", (0.0, 0.005, -0.091), (0.046, 0.068, 0.056))

    for link_name, (name, filename) in SOLE_VISUALS.items():
        link = find_link(root, link_name)
        add_mesh_visual(link, name, filename, DARK)
        z_center = 0.019025 if link_name == "foot_left" else -0.019025
        add_box_collision(
            link,
            name,
            (-0.010, -0.02211, z_center),
            (0.112, 0.016, 0.064),
        )

    add_electronics_layout(root, electronics_layout)

    root.insert(
        4,
        ET.Comment(
            "Round-v1 adds printable PETG cosmetic shells and 8 mm thicker "
            "sole prototypes. Nominal CAD-volume mass/inertia is included; "
            "hardware deployment remains blocked until every printed part and "
            "the final assembly are weighed and the inertials regenerated."
        ),
    )
    root.insert(
        5,
        ET.Comment(
            "The actual FEETECH STS3250 STEP is a SolidWorks/CAD placement "
            "reference. Servo mass is not added again because the baseline "
            "aggregate link inertials already represent the source assemblies."
        ),
    )
    root.insert(
        6,
        ET.Comment(
            "Camera, IMU, compute and 3S2P battery links use explicit "
            "ASSUMED_FOR_RL box envelopes and masses from "
            "round_v1_electronics_sensor_layout.json. They have no collision "
            "geometry because they are internal payloads. Replace their "
            "inertials and extrinsics after exact hardware selection and "
            "weighing. camera_optical_frame is intentionally massless."
        ),
    )
    ready.load_module(
        ready.AUDITED_GENERATOR, "zeroth01_round_v1_audited_validator"
    ).load_reference_module()._validate_tree(root)
    return root


if __name__ == "__main__":
    raise SystemExit(
        "Use the URDF skill launcher so generation-time validation is applied."
    )
