"""Generate a primitive-only MuJoCo/MJX model from the v4 URDF ledger."""

from __future__ import annotations

import importlib.util
import json
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
V3_MJCF_SOURCE = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "build_v3_mjcf.py"
V4_URDF_SOURCE = Path(__file__).with_name("build_v4_urdf.py")
OUT = ROOT / "generated" / "mujoco" / "physical_mount_v4_original_minimal"
MJCF = OUT / "zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml"
REPORT = ROOT / "reports" / "v4_original_minimal" / "mjcf_compile_gate.json"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def add_inertial_body(parent, name, mass, size, center, *, rgba, collision=False):
    node = ET.SubElement(parent, "body", name=name)
    ET.SubElement(
        node,
        "inertial",
        pos=fmt(center),
        mass=f"{mass:.12g}",
        diaginertia=fmt(inertia_box(mass, size)),
    )
    kwargs = {
        "name": f"{name}_{'collision' if collision else 'visual'}",
        "type": "box",
        "size": fmt(tuple(value / 2.0 for value in size)),
        "pos": fmt(center),
        "rgba": rgba,
    }
    if not collision:
        kwargs.update({"contype": "0", "conaffinity": "0"})
    ET.SubElement(node, "geom", **kwargs)
    return node


def main() -> int:
    u4 = load(V4_URDF_SOURCE, "zeroth01_v4_urdf_for_mjcf")
    robot = u4.gen_urdf()
    body_mass = float(
        next(link for link in robot.findall("link") if link.get("name") == u4.v3.BODY)
        .find("./inertial/mass")
        .get("value")
    )

    # The proven v3 MJX builder owns actuator order, floating-base convention,
    # foot contacts and reset keyframes.  Feed it the v4 neutral transforms and
    # masses, then replace only its legacy payload block.
    base = u4.v3
    base.neutral_transforms = u4.neutral_transforms_v4
    base.joint_kinematics = u4.joint_kinematics_v4
    base.body_mass = lambda: body_mass
    m3 = load(V3_MJCF_SOURCE, "zeroth01_v3_mjcf_builder_for_v4")
    m3.OUT = OUT
    m3.MJCF = MJCF
    m3.load_urdf_module = lambda: base
    m3.main()

    root = ET.parse(MJCF).getroot()
    root.set("model", "zeroth01_physical_mount_v4_original_minimal_18dof_mjx")
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

    old_robot = ET.parse(base.V2_URDF).getroot()
    neutral_tf = u4.neutral_transforms_v4(base.old_fk(old_robot))
    for side, foot in (("left", "FOOT"), ("right", "FOOT_2")):
        foot_body = torso.find(f".//body[@name='{foot}']")
        if foot_body is None:
            raise RuntimeError(f"missing {foot}")
        rotation_t = base.mat_t(neutral_tf[foot][0])
        for corner, world_offset in (
            ("front_medial", (0.045, -0.030, u4.SOLE_CONTACT_WORLD_Z_M)),
            ("front_lateral", (0.045, 0.030, u4.SOLE_CONTACT_WORLD_Z_M)),
            ("rear_medial", (-0.055, -0.030, u4.SOLE_CONTACT_WORLD_Z_M)),
            ("rear_lateral", (-0.055, 0.030, u4.SOLE_CONTACT_WORLD_Z_M)),
        ):
            site = foot_body.find(f"site[@name='{side}_sole_{corner}']")
            if site is None:
                raise RuntimeError(f"missing {side}_{corner} contact site")
            site.set("pos", fmt(base.mat_vec(rotation_t, world_offset)))

    head = add_inertial_body(
        torso,
        "v4_head_shell",
        u4.V4_FIXED_MASSES["v4_head_shell"],
        (0.090750004, 0.031000001, 0.070409235),
        (0.0, 0.009189728, 0.03883662),
        rgba="0.97 0.98 0.99 1",
        collision=True,
    )
    unitv2 = add_inertial_body(
        head,
        "m5stack_unitv2",
        u4.V4_FIXED_MASSES["m5stack_unitv2"],
        (0.048, 0.024260273, 0.024),
        (0.0, 0.006, 0.045),
        rgba="0 0.72 0.85 1",
    )
    ET.SubElement(
        unitv2,
        "site",
        name="unitv2_camera_site",
        pos="-0.014 -0.0075102725 0.045",
        size="0.004",
        rgba="0 0.8 1 1",
    )

    pod = add_inertial_body(
        torso,
        "v4_rear_service_pod",
        u4.V4_FIXED_MASSES["v4_rear_service_pod"],
        (0.098, 0.0328, 0.100),
        (0.0, 0.084, -0.042),
        rgba="0.88 0.89 0.91 1",
        collision=True,
    )
    add_inertial_body(
        pod,
        "v4_compute_module",
        u4.V4_FIXED_MASSES["v4_compute_module"],
        (0.070, 0.012, 0.032),
        (0.0, 0.084, -0.018),
        rgba="1 0.57 0 1",
    )
    add_inertial_body(
        pod,
        "v4_battery_pack",
        u4.V4_FIXED_MASSES["v4_battery_pack"],
        (0.075, 0.022, 0.034),
        (0.0, 0.084, -0.063),
        rgba="0.84 0 0.98 1",
    )
    imu = add_inertial_body(
        pod,
        "v4_torso_imu",
        u4.V4_FIXED_MASSES["v4_torso_imu"],
        (0.032, 0.008, 0.025),
        (0.0, 0.094, -0.018),
        rgba="0.39 0.87 0.09 1",
    )
    imu_site = torso.find("site[@name='imu_site']")
    if imu_site is not None:
        torso.remove(imu_site)
    ET.SubElement(
        imu,
        "site",
        name="imu_site",
        pos="0 0.094 -0.018",
        size="0.006",
        rgba="0.2 1 0.2 1",
    )

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(MJCF, encoding="utf-8", xml_declaration=True)

    runtime = {
        "runtime_compile_gate": "PENDING_MUJOCO_RUNTIME",
    }
    try:
        import mujoco

        model = mujoco.MjModel.from_xml_path(str(MJCF))
        runtime = {
            "runtime_compile_gate": "PASS",
            "mujoco_version": mujoco.__version__,
            "nq": int(model.nq),
            "nv": int(model.nv),
            "nu": int(model.nu),
            "body_count": int(model.nbody),
            "geom_count": int(model.ngeom),
            "sensor_count": int(model.nsensor),
            "compiled_total_mass_kg": float(model.body_mass.sum()),
        }
    except ImportError:
        pass

    payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.mjcf_compile_gate.v1",
        "mjcf": MJCF.relative_to(ROOT).as_posix(),
        "expected_mass_kg": u4.TARGET_TOTAL_MASS_KG,
        "expected_joint_count": 18,
        "source_gate": "PASS",
        **runtime,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(MJCF)
    print(REPORT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
