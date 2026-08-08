"""Generate the 16-DoF MuJoCo/MJX model from the v5 URDF ledger.

The v3 MJX generator remains the source for the floating-base convention,
primitive collision policy, foot sensors and actuator definition.  This
adapter restores the original Zeroth-01 ankle-pitch -> foot chain and removes
the two experimental ankle-roll bodies before the model reaches RL.
"""

from __future__ import annotations

import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path

from build123d import Location, import_step
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
V3_MJCF_SOURCE = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "build_v3_mjcf.py"
V5_URDF_SOURCE = Path(__file__).with_name("build_v5_urdf.py")
OUT = ROOT / "generated" / "mujoco" / "physical_mount_v5_original_16dof_solidworks_motion"
MJCF = OUT / "zeroth01_physical_mount_v5_original_16dof_solidworks_motion_mjx.xml"
REPORT = ROOT / "reports" / "v5_original_16dof_solidworks_motion" / "mjcf_compile_gate.json"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def vec(text: str | None):
    return tuple(float(value) for value in (text or "0 0 0").split())


def fmt(values):
    return " ".join(f"{float(value):.12g}" for value in values)


def inertia_box(mass, size):
    x, y, z = size
    return (
        mass * (y * y + z * z) / 12.0,
        mass * (x * x + z * z) / 12.0,
        mass * (x * x + y * y) / 12.0,
    )


def remove_named_body(parent: ET.Element, names: set[str]) -> None:
    for body in list(parent.findall("body")):
        if str(body.get("name")) in names:
            parent.remove(body)
        else:
            remove_named_body(body, names)


def joint_kinematics_original_16(v5, source_robot: ET.Element, neutral_tf):
    rows = []
    for joint in source_robot.findall("joint"):
        if joint.get("type") != "revolute":
            continue
        origin_node = joint.find("origin")
        axis_node = joint.find("axis")
        limit_node = joint.find("limit")
        origin = (
            v5.v3.rpy_matrix(vec(origin_node.get("rpy") if origin_node is not None else None)),
            vec(origin_node.get("xyz") if origin_node is not None else None),
        )
        rows.append(
            (
                str(joint.get("name")),
                str(joint.find("parent").get("link")),
                str(joint.find("child").get("link")),
                origin,
                vec(axis_node.get("xyz") if axis_node is not None else None),
                (float(limit_node.get("lower")), float(limit_node.get("upper"))),
            )
        )
    if len(rows) != 16:
        raise RuntimeError(f"expected 16 original joints, got {len(rows)}")
    return rows


def add_v5_head(torso: ET.Element) -> None:
    head = ET.SubElement(torso, "body", name="v4_head_shell")
    ET.SubElement(
        head,
        "inertial",
        pos="0 0.009189728 0.03883662",
        mass="0.055",
        diaginertia=fmt(inertia_box(0.055, (0.090750004, 0.031000001, 0.070409235))),
    )
    ET.SubElement(
        head,
        "geom",
        name="v4_head_shell_collision",
        type="box",
        size=fmt((0.045375002, 0.0155000005, 0.0352046175)),
        pos="0 0.009189728 0.03883662",
        rgba="0.97 0.98 0.99 1",
    )
    unitv2 = ET.SubElement(head, "body", name="m5stack_unitv2")
    ET.SubElement(
        unitv2,
        "inertial",
        pos="0 0.006 0.045",
        mass="0.018",
        diaginertia=fmt(inertia_box(0.018, (0.048, 0.024260273, 0.024))),
    )
    ET.SubElement(
        unitv2,
        "geom",
        name="m5stack_unitv2_visual",
        type="box",
        size="0.024 0.0121301365 0.012",
        pos="0 0.006 0.045",
        rgba="0 0.72 0.85 1",
        contype="0",
        conaffinity="0",
    )
    ET.SubElement(unitv2, "site", name="unitv2_camera_site", pos="-0.014 -0.0075102725 0.045", size="0.004", rgba="0 0.8 1 1")
    ET.SubElement(unitv2, "site", name="unitv2_microphone_site", pos="0.014 -0.0075102725 0.045", size="0.004", rgba="1 0.2 0.8 1")


def find_body(root: ET.Element, name: str) -> ET.Element:
    body = root.find(f".//body[@name='{name}']")
    if body is None:
        raise RuntimeError(f"missing MuJoCo body {name}")
    return body


def replace_collision_geometry(root: ET.Element) -> None:
    """Use v5 wrist/foot geometry instead of the old oversized proxies."""

    for name, center_y in (("FINGER_1", -0.014), ("FINGER_1_2", 0.014)):
        body = find_body(root, name)
        geom = body.find(f"./geom[@name='{name}_collision']")
        if geom is None:
            raise RuntimeError(f"missing wrist collision {name}")
        for key in ("fromto", "quat", "axisangle", "euler"):
            geom.attrib.pop(key, None)
        geom.set("type", "box")
        geom.set("size", "0.010 0.0035 0.007")
        geom.set("pos", fmt((-0.0034808, center_y, 0.0187993)))
        geom.set("rgba", "0.97 0.98 0.99 1")

    for name, carrier_y, sole_y, side in (
        ("FOOT", 0.012845, 0.04013, "left"),
        ("FOOT_2", -0.012845, -0.04013, "right"),
    ):
        body = find_body(root, name)
        carrier = body.find(f"./geom[@name='{name}_collision']")
        if carrier is None:
            raise RuntimeError(f"missing foot collision {name}")
        carrier.set("type", "box")
        carrier.set("size", "0.045 0.0228 0.0202")
        carrier.set("pos", fmt((-0.016025, carrier_y, 0.0187)))
        carrier.set("rgba", "0.92 0.93 0.95 1")
        carrier.set("friction", "1.0 0.02 0.001")
        sole = ET.SubElement(
            body,
            "geom",
            name=f"{name}_v5_white_tapered_sole_collision",
            type="box",
            size="0.049 0.0045 0.024",
            pos=fmt((-0.016025, sole_y, 0.0187)),
            rgba="0.97 0.98 0.99 1",
            friction="1.2 0.02 0.001",
        )
        _ = sole
        for site in list(body.findall("site")):
            if str(site.get("name", "")).startswith(f"{side}_sole_"):
                body.remove(site)
        for fore_aft, x_pos in (("front", 0.032), ("rear", -0.045)):
            for lateral, z_pos in (("medial", 0.0067), ("lateral", 0.0307)):
                ET.SubElement(
                    body,
                    "site",
                    name=f"{side}_sole_{fore_aft}_{lateral}",
                    pos=fmt((x_pos, 0.0446 if side == "left" else -0.0446, z_pos)),
                    size="0.004",
                    rgba="0 1 0.8 1",
                )


def apply_urdf_inertials(root: ET.Element, robot: ET.Element) -> None:
    """Copy every RL mass, COM and full inertia tensor from the v5 URDF."""

    for link in robot.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        name = str(link.get("name"))
        body = root.find(f".//body[@name='{name}']")
        if body is None:
            # Fixed massless contact-frame links become MuJoCo sites.
            continue
        target = body.find("inertial")
        if target is None:
            target = ET.SubElement(body, "inertial")
        origin = inertial.find("origin")
        tensor = inertial.find("inertia")
        target.set("pos", origin.get("xyz"))
        target.set("mass", inertial.find("mass").get("value"))
        target.set(
            "fullinertia",
            " ".join(tensor.get(key) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")),
        )
        for key in ("diaginertia", "quat"):
            target.attrib.pop(key, None)


def transformed_bbox_z(row, source: Path):
    shape = import_step(source)
    matrix = row["transform_local_mm_to_world_mm"]
    transform = gp_Trsf()
    transform.SetValues(*[float(matrix[r][c]) for r in range(3) for c in range(4)])
    world = shape.moved(Location(gp_trsf=transform))
    bounds = world.bounding_box()
    return float(bounds.min.Z) / 1000.0, float(bounds.max.Z) / 1000.0


def apply_actual_standing_height(root: ET.Element, u5, base, v2_robot) -> float:
    manifest = json.loads(u5.V5_MANIFEST.read_text(encoding="utf-8"))
    sole_rows = [row for row in manifest["components"] if row["role"] == "replaceable_white_tapered_lower_wider_sole"]
    bounds = [transformed_bbox_z(row, u5.ROOT / str(row["source"])) for row in sole_rows]
    neutral_min_z = min(item[0] for item in bounds)
    standing_height = -neutral_min_z
    neutral_body_z = base.old_fk(v2_robot)[base.BODY][1][2]
    for key in root.findall("./keyframe/key"):
        qpos = [float(value) for value in key.get("qpos").split()]
        qpos[2] = neutral_body_z + standing_height
        key.set("qpos", fmt(qpos))
    return standing_height


def main() -> int:
    u5 = load(V5_URDF_SOURCE, "zeroth01_v5_urdf_for_mjcf")
    robot = u5.gen_urdf()
    v2_robot = ET.parse(u5.V2_URDF).getroot()
    base = u5.v3

    masses = {
        str(link.get("name")): float(link.find("./inertial/mass").get("value"))
        for link in robot.findall("link")
        if link.find("./inertial/mass") is not None
    }
    body_mass = masses[base.BODY]
    for name, mass in masses.items():
        if name != base.BODY:
            base.FIXED_MASSES[name] = mass

    # Roll back the experimental 18-DoF topology at the shared primitive
    # collision layer as well as at URDF level.
    base.neutral_transforms = lambda old_tf: dict(old_tf)
    base.joint_kinematics = lambda old_robot, neutral_tf: joint_kinematics_original_16(u5, old_robot, neutral_tf)
    base.body_mass = lambda: body_mass
    base.SEGMENT_CHILD["3215_BothFlange_13"] = "FOOT"
    base.SEGMENT_CHILD["3215_BothFlange_14"] = "FOOT_2"
    base.SEGMENT_CHILD.pop(base.LEFT_ANKLE_CARRIER, None)
    base.SEGMENT_CHILD.pop(base.RIGHT_ANKLE_CARRIER, None)

    m3 = load(V3_MJCF_SOURCE, "zeroth01_v3_mjcf_builder_for_v5")
    m3.OUT = OUT
    m3.MJCF = MJCF
    m3.load_urdf_module = lambda: base
    m3.main()

    root = ET.parse(MJCF).getroot()
    root.set("model", "zeroth01_physical_mount_v5_original_16dof_solidworks_motion_mjx")
    torso = root.find(f"./worldbody/body[@name='{base.BODY}']")
    if torso is None:
        raise RuntimeError("missing torso body")
    remove_named_body(
        torso,
        {
            "IMU_2",
            "torso_imu_module",
            "compute_module",
            "battery_pack",
            base.CORES3_HEAD_ADAPTER,
            base.CORES3_HEAD_POD,
        },
    )
    # Those internal payloads are already in the v5 torso aggregate mass.
    # The two separately-accounted purchased head links are reinstated here.
    add_v5_head(torso)
    replace_collision_geometry(root)
    apply_urdf_inertials(root, robot)
    standing_height = apply_actual_standing_height(root, u5, base, v2_robot)

    actuators = root.findall("./actuator/motor")
    if len(actuators) != 16 or any("ankle_roll" in str(item.get("joint")) for item in actuators):
        raise RuntimeError("v5 actuator topology is not original 16-DoF")

    ET.indent(root, space="  ")
    OUT.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(MJCF, encoding="utf-8", xml_declaration=True)

    runtime = {"runtime_compile_gate": "PENDING_MUJOCO_RUNTIME"}
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MJCF))
        compiled_mass = float(model.body_mass.sum())
        runtime = {
            "runtime_compile_gate": "PASS",
            "mujoco_version": mujoco.__version__,
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "body_count": int(model.nbody),
            "geom_count": int(model.ngeom),
            "sensor_count": int(model.nsensor),
            "compiled_total_mass_kg": compiled_mass,
            "compiled_mass_delta_kg": compiled_mass - u5.TARGET_TOTAL_MASS_KG,
            "standing_height_m": standing_height,
        }
        if model.nu != 16 or abs(compiled_mass - u5.TARGET_TOTAL_MASS_KG) > 1.0e-8:
            runtime["runtime_compile_gate"] = "FAIL"
    except ImportError:
        pass

    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.mjcf_compile_gate.v1",
        "mjcf": MJCF.relative_to(ROOT).as_posix(),
        "expected_mass_kg": u5.TARGET_TOTAL_MASS_KG,
        "expected_joint_count": 16,
        "ankle_roll_actuator_count": 0,
        "source_gate": "PASS",
        "inertial_source": "full tensors copied from generated v5 URDF",
        "collision_source": "v5 compact wrist support + 9 mm lower-wider sole primitives",
        **runtime,
    }
    payload["overall"] = "PASS" if payload["runtime_compile_gate"] in {"PASS", "PENDING_MUJOCO_RUNTIME"} else "FAIL"
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0 if payload["overall"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
