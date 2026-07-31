from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import shutil
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

from PIL import Image
from vtkmodules.vtkCommonMath import vtkMatrix4x4
from vtkmodules.vtkFiltersCore import (
    vtkAppendPolyData,
    vtkCleanPolyData,
    vtkPolyDataConnectivityFilter,
)
from vtkmodules.vtkIOGeometry import vtkSTLReader, vtkSTLWriter
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkPolyDataMapper,
    vtkRenderWindow,
    vtkRenderer,
    vtkWindowToImageFilter,
)

# Register the OpenGL backend when VTK is installed as split vtkmodules.
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401


ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "config" / "physical_mount_v1_source_regions.json"
CAD_ROOT = ROOT / "generated" / "cad" / "physical_mount_v1"
SKELETON_DIR = CAD_ROOT / "skeleton"
SERVO_DIR = CAD_ROOT / "servos"
URDF_ROOT = ROOT / "generated" / "urdf" / "physical_mount_v1"
URDF_SKELETON_DIR = URDF_ROOT / "meshes" / "skeleton"
URDF_SERVO_DIR = URDF_ROOT / "meshes" / "servos"
URDF_PATH = URDF_ROOT / "zeroth01_physical_mount_v1.urdf"
ACTUATOR_PATH = (
    ROOT / "generated" / "config" / "physical_mount_v1_actuators.json"
)
REPORT_ROOT = ROOT / "reports" / "physical_mount_v1"
MANIFEST_PATH = REPORT_ROOT / "servo_component_manifest.json"
MANIFEST_CSV = REPORT_ROOT / "servo_component_manifest.csv"
GATE_PATH = REPORT_ROOT / "source_component_gate.json"
INERTIA_REPORT = REPORT_ROOT / "sts3250_inertia_delta.json"
SNAPSHOT_ROOT = ROOT / "snapshots" / "physical_mount_v1"
MOTION_ROOT = SNAPSHOT_ROOT / "motion_frames"
FRONT_SNAPSHOT = SNAPSHOT_ROOT / "physical_mount_v1_16_blue_servos_front.png"
XRAY_SNAPSHOT = SNAPSHOT_ROOT / "physical_mount_v1_16_blue_servos_xray.png"
MOTION_GIF = SNAPSHOT_ROOT / "physical_mount_v1_16dof_motion.gif"
GUARDED_LIMITS = ROOT / "reports" / "joint_servo_frames.csv"
PHYSICAL_GUARDED_LIMITS = (
    ROOT / "config" / "physical_mount_v1_guarded_limits.json"
)

STS3215_MASS_KG = 0.055
STS3250_MASS_KG = 0.0745
STS3250_MASS_TOLERANCE_KG = 0.001
SERVO_DELTA_MASS_KG = STS3250_MASS_KG - STS3215_MASS_KG
IDENTITY_R = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)


def _parse_vec(
    text: str | None,
    default: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[float, float, float]:
    if not text:
        return default
    values = tuple(float(value) for value in text.split())
    if len(values) != 3:
        raise ValueError(text)
    return values


def _mat_mul(
    first: tuple[tuple[float, ...], ...],
    second: tuple[tuple[float, ...], ...],
) -> tuple[tuple[float, ...], ...]:
    return tuple(
        tuple(
            sum(first[row][index] * second[index][column] for index in range(3))
            for column in range(3)
        )
        for row in range(3)
    )


def _mat_vec(
    matrix: tuple[tuple[float, ...], ...],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    )


def _vec_add(
    first: tuple[float, float, float],
    second: tuple[float, float, float],
) -> tuple[float, float, float]:
    return tuple(first[index] + second[index] for index in range(3))


def _tf_mul(
    first: tuple[tuple[tuple[float, ...], ...], tuple[float, ...]],
    second: tuple[tuple[tuple[float, ...], ...], tuple[float, ...]],
) -> tuple[tuple[tuple[float, ...], ...], tuple[float, ...]]:
    first_r, first_t = first
    second_r, second_t = second
    return _mat_mul(first_r, second_r), _vec_add(
        _mat_vec(first_r, second_t), first_t
    )


def _rpy_matrix(
    rpy: tuple[float, float, float],
) -> tuple[tuple[float, ...], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return _mat_mul(rz, _mat_mul(ry, rx))


def _axis_angle_matrix(
    axis: tuple[float, float, float],
    angle: float,
) -> tuple[tuple[float, ...], ...]:
    norm = math.sqrt(sum(value * value for value in axis))
    if norm <= 1e-12:
        return IDENTITY_R
    x_pos, y_pos, z_pos = (value / norm for value in axis)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    remainder = 1.0 - cosine
    return (
        (
            cosine + x_pos * x_pos * remainder,
            x_pos * y_pos * remainder - z_pos * sine,
            x_pos * z_pos * remainder + y_pos * sine,
        ),
        (
            y_pos * x_pos * remainder + z_pos * sine,
            cosine + y_pos * y_pos * remainder,
            y_pos * z_pos * remainder - x_pos * sine,
        ),
        (
            z_pos * x_pos * remainder - y_pos * sine,
            z_pos * y_pos * remainder + x_pos * sine,
            cosine + z_pos * z_pos * remainder,
        ),
    )


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _read_clean_mesh(path: Path) -> object:
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    clean = vtkCleanPolyData()
    clean.SetInputConnection(reader.GetOutputPort())
    clean.Update()
    return clean.GetOutput()


def _connectivity(polydata: object) -> object:
    connection = vtkPolyDataConnectivityFilter()
    connection.SetInputData(polydata)
    connection.SetExtractionModeToAllRegions()
    connection.ColorRegionsOn()
    connection.Update()
    return connection


def _extract_region(polydata: object, region_id: int) -> object:
    extract = vtkPolyDataConnectivityFilter()
    extract.SetInputData(polydata)
    extract.SetExtractionModeToSpecifiedRegions()
    extract.AddSpecifiedRegion(region_id)
    extract.Update()
    clean = vtkCleanPolyData()
    clean.SetInputConnection(extract.GetOutputPort())
    clean.Update()
    return clean.GetOutput()


def _append_regions(polydata: object, region_ids: list[int]) -> object:
    append = vtkAppendPolyData()
    for region_id in region_ids:
        append.AddInputData(_extract_region(polydata, region_id))
    append.Update()
    clean = vtkCleanPolyData()
    clean.SetInputConnection(append.GetOutputPort())
    clean.Update()
    return clean.GetOutput()


def _write_stl(polydata: object, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = vtkSTLWriter()
    writer.SetFileName(str(path))
    writer.SetFileTypeToBinary()
    writer.SetInputData(polydata)
    if writer.Write() != 1:
        raise RuntimeError(f"failed to write {path}")


def _used_bounds(polydata: object) -> tuple[float, ...]:
    point_ids: set[int] = set()
    for cell_id in range(polydata.GetNumberOfCells()):
        ids = polydata.GetCell(cell_id).GetPointIds()
        for index in range(ids.GetNumberOfIds()):
            point_ids.add(int(ids.GetId(index)))
    points = [polydata.GetPoint(point_id) for point_id in point_ids]
    if not points:
        raise ValueError("empty polydata region")
    return (
        min(point[0] for point in points),
        max(point[0] for point in points),
        min(point[1] for point in points),
        max(point[1] for point in points),
        min(point[2] for point in points),
        max(point[2] for point in points),
    )


def _bounds_size(bounds: tuple[float, ...]) -> tuple[float, float, float]:
    return (
        bounds[1] - bounds[0],
        bounds[3] - bounds[2],
        bounds[5] - bounds[4],
    )


def _bounds_center(bounds: tuple[float, ...]) -> tuple[float, float, float]:
    return (
        (bounds[0] + bounds[1]) / 2.0,
        (bounds[2] + bounds[3]) / 2.0,
        (bounds[4] + bounds[5]) / 2.0,
    )


def _load_guarded_limits() -> dict[str, tuple[float, float]]:
    limits: dict[str, tuple[float, float]] = {}
    with GUARDED_LIMITS.open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            limits[str(row["joint"])] = (
                float(row["guarded_lower_rad"]),
                float(row["guarded_upper_rad"]),
            )
    overrides = json.loads(
        PHYSICAL_GUARDED_LIMITS.read_text(encoding="utf-8")
    )["limits"]
    for joint, values in overrides.items():
        limits[str(joint)] = (float(values[0]), float(values[1]))
    return limits


def _source_link_meshes(
    root: ET.Element,
) -> dict[str, str]:
    result: dict[str, str] = {}
    for link in root.findall("link"):
        mesh = link.find("./visual/geometry/mesh")
        if mesh is None:
            continue
        filename = str(mesh.get("filename", ""))
        if not filename:
            continue
        result[str(link.get("name"))] = Path(filename).name
    return result


def split_source_geometry(
    config: dict[str, object],
    source_mesh_root: Path,
    link_meshes: dict[str, str],
) -> tuple[list[dict[str, object]], dict[str, str], dict[str, list[dict[str, object]]]]:
    servos = list(config["servos"])
    by_mesh: dict[str, list[dict[str, object]]] = defaultdict(list)
    for servo in servos:
        by_mesh[str(servo["source_mesh"])].append(servo)

    manifest: list[dict[str, object]] = []
    skeleton_meshes: dict[str, str] = {}
    servos_by_link: dict[str, list[dict[str, object]]] = defaultdict(list)
    processed_files: dict[str, Path] = {}

    for link_name, mesh_name in link_meshes.items():
        source = source_mesh_root / mesh_name
        if not source.is_file():
            raise FileNotFoundError(source)
        target = SKELETON_DIR / mesh_name
        mapped = by_mesh.get(mesh_name, [])
        if not mapped:
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            skeleton_meshes[link_name] = mesh_name
            processed_files[mesh_name] = target
            continue

        polydata = _read_clean_mesh(source)
        connection = _connectivity(polydata)
        region_count = int(connection.GetNumberOfExtractedRegions())
        mapped_ids = {int(item["region_id"]) for item in mapped}
        if not mapped_ids.issubset(set(range(region_count))):
            raise ValueError(
                f"{mesh_name} regions {sorted(mapped_ids)} outside 0..{region_count - 1}"
            )
        remaining_ids = [
            region_id
            for region_id in range(region_count)
            if region_id not in mapped_ids
        ]
        if not remaining_ids:
            raise ValueError(f"{mesh_name} has no carrier geometry after split")
        _write_stl(_append_regions(polydata, remaining_ids), target)
        skeleton_meshes[link_name] = mesh_name
        processed_files[mesh_name] = target

        for servo in mapped:
            region_id = int(servo["region_id"])
            servo_polydata = _extract_region(polydata, region_id)
            bounds = _used_bounds(servo_polydata)
            size_mm = tuple(value * 1000.0 for value in _bounds_size(bounds))
            center_m = _bounds_center(bounds)
            sorted_size = sorted(size_mm)
            family_fit = (
                abs(sorted_size[2] - 45.22) <= 1.0
                and abs(sorted_size[1] - 37.4) <= 1.0
                and 24.0 <= sorted_size[0] <= 28.5
            )
            filename = (
                f"{servo['id']}_{servo['joint']}_"
                "INSTALLED_STS3215_FAMILY.stl"
            )
            servo_path = SERVO_DIR / filename
            _write_stl(servo_polydata, servo_path)
            entry = {
                **servo,
                "output_mesh": str(servo_path.relative_to(ROOT)).replace("\\", "/"),
                "source_region_triangles": int(
                    servo_polydata.GetNumberOfCells()
                ),
                "source_region_points": int(
                    servo_polydata.GetNumberOfPoints()
                ),
                "bounds_m": [round(value, 9) for value in bounds],
                "center_m_in_owning_link": [
                    round(value, 9) for value in center_m
                ],
                "size_mm": [round(value, 6) for value in size_mm],
                "family_envelope_gate": "PASS" if family_fit else "FAIL",
                "source_sha256": _sha256(source),
                "output_sha256": _sha256(servo_path),
                "mounting_semantics": (
                    "EXTRACTED_IN_PLACE_FROM_ORIGINAL_ASSEMBLED_ZEROTH_LINK; "
                    "NO_TRANSLATION_OR_HOUSING_PHASE_ADDED"
                ),
            }
            manifest.append(entry)
            servos_by_link[str(servo["owning_link"])].append(entry)

    if len(manifest) != 16:
        raise ValueError(f"expected 16 servos, got {len(manifest)}")
    manifest.sort(key=lambda row: str(row["id"]))
    return manifest, skeleton_meshes, servos_by_link


def _write_manifest(manifest: list[dict[str, object]]) -> None:
    MANIFEST_PATH.parent.mkdir(parents=True, exist_ok=True)
    MANIFEST_PATH.write_text(
        json.dumps(
            {
                "schema": "zeroth01.physical_mount_v1.servo_manifest.v1",
                "servo_count": len(manifest),
                "servos": manifest,
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )
    fields: list[str] = []
    for row in manifest:
        for key in row:
            if key not in fields:
                fields.append(key)
    with MANIFEST_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=fields,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(manifest)


def _parallel_axis(
    mass: float,
    offset: tuple[float, float, float],
) -> list[list[float]]:
    x_pos, y_pos, z_pos = offset
    return [
        [mass * (y_pos * y_pos + z_pos * z_pos), -mass * x_pos * y_pos, -mass * x_pos * z_pos],
        [-mass * x_pos * y_pos, mass * (x_pos * x_pos + z_pos * z_pos), -mass * y_pos * z_pos],
        [-mass * x_pos * z_pos, -mass * y_pos * z_pos, mass * (x_pos * x_pos + y_pos * y_pos)],
    ]


def _add_matrix(first: list[list[float]], second: list[list[float]]) -> list[list[float]]:
    return [
        [first[row][column] + second[row][column] for column in range(3)]
        for row in range(3)
    ]


def update_inertias(
    root: ET.Element,
    servos_by_link: dict[str, list[dict[str, object]]],
) -> list[dict[str, object]]:
    report: list[dict[str, object]] = []
    for link in root.findall("link"):
        link_name = str(link.get("name"))
        additions = servos_by_link.get(link_name, [])
        if not additions:
            continue
        inertial = link.find("inertial")
        if inertial is None:
            raise ValueError(f"{link_name} has no inertial")
        mass_element = inertial.find("mass")
        inertia_element = inertial.find("inertia")
        origin_element = inertial.find("origin")
        if mass_element is None or inertia_element is None:
            raise ValueError(f"{link_name} inertial is incomplete")
        old_mass = float(mass_element.get("value", "0"))
        old_center = _parse_vec(
            origin_element.get("xyz") if origin_element is not None else None
        )
        old_inertia = [
            [
                float(inertia_element.get("ixx", "0")),
                float(inertia_element.get("ixy", "0")),
                float(inertia_element.get("ixz", "0")),
            ],
            [
                float(inertia_element.get("ixy", "0")),
                float(inertia_element.get("iyy", "0")),
                float(inertia_element.get("iyz", "0")),
            ],
            [
                float(inertia_element.get("ixz", "0")),
                float(inertia_element.get("iyz", "0")),
                float(inertia_element.get("izz", "0")),
            ],
        ]
        new_mass = old_mass + SERVO_DELTA_MASS_KG * len(additions)
        weighted = [
            old_mass * old_center[index]
            + SERVO_DELTA_MASS_KG
            * sum(
                float(item["center_m_in_owning_link"][index])
                for item in additions
            )
            for index in range(3)
        ]
        new_center = tuple(value / new_mass for value in weighted)
        old_offset = tuple(
            old_center[index] - new_center[index] for index in range(3)
        )
        new_inertia = _add_matrix(
            old_inertia,
            _parallel_axis(old_mass, old_offset),
        )
        for addition in additions:
            center = tuple(
                float(value) for value in addition["center_m_in_owning_link"]
            )
            offset = tuple(
                center[index] - new_center[index] for index in range(3)
            )
            new_inertia = _add_matrix(
                new_inertia,
                _parallel_axis(SERVO_DELTA_MASS_KG, offset),
            )

        mass_element.set("value", f"{new_mass:.9f}")
        if origin_element is None:
            origin_element = ET.SubElement(inertial, "origin")
        origin_element.set(
            "xyz", " ".join(f"{value:.9f}" for value in new_center)
        )
        origin_element.set("rpy", "0 0 0")
        inertia_element.attrib.update(
            {
                "ixx": f"{new_inertia[0][0]:.12g}",
                "iyy": f"{new_inertia[1][1]:.12g}",
                "izz": f"{new_inertia[2][2]:.12g}",
                "ixy": f"{new_inertia[0][1]:.12g}",
                "ixz": f"{new_inertia[0][2]:.12g}",
                "iyz": f"{new_inertia[1][2]:.12g}",
            }
        )
        report.append(
            {
                "link": link_name,
                "servo_ids": [item["id"] for item in additions],
                "old_mass_kg": old_mass,
                "new_mass_kg": new_mass,
                "old_com_m": list(old_center),
                "new_com_m": list(new_center),
                "method": (
                    "replace archived 55 g STS3215-family actuator mass with "
                    "74.5 g STS3250 by a 19.5 g point-mass delta at each "
                    "extracted actuator-region centre"
                ),
            }
        )
    INERTIA_REPORT.write_text(
        json.dumps(
            {
                "schema": "zeroth01.physical_mount_v1.inertia_delta.v1",
                "source_servo_mass_kg": STS3215_MASS_KG,
                "target_servo_mass_kg": STS3250_MASS_KG,
                "delta_per_servo_kg": SERVO_DELTA_MASS_KG,
                "links": report,
            },
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return report


def build_urdf(
    config: dict[str, object],
    source_urdf: Path,
    skeleton_meshes: dict[str, str],
    servos_by_link: dict[str, list[dict[str, object]]],
) -> ET.Element:
    tree = ET.parse(source_urdf)
    root = tree.getroot()
    root.set("name", "zeroth01_physical_mount_v1_sts3250_16dof")
    renames = {
        str(key): str(value)
        for key, value in dict(config["joint_renames"]).items()
    }
    fixed = {str(name) for name in config["fixed_joints"]}
    guarded = _load_guarded_limits()

    for joint in root.findall("joint"):
        source_name = str(joint.get("name"))
        final_name = renames.get(source_name, source_name)
        joint.set("name", final_name)
        if source_name in fixed:
            joint.set("type", "fixed")
            for child_tag in ("axis", "limit", "dynamics", "safety_controller"):
                child = joint.find(child_tag)
                if child is not None:
                    joint.remove(child)
            continue
        if joint.get("type") != "revolute":
            continue
        if final_name not in guarded:
            raise KeyError(f"no guarded limit for {final_name}")
        lower, upper = guarded[final_name]
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        limit.attrib.update(
            {
                "lower": f"{lower:.9f}",
                "upper": f"{upper:.9f}",
                "effort": "1.569",
                "velocity": "3.0",
            }
        )
        dynamics = joint.find("dynamics")
        if dynamics is None:
            dynamics = ET.SubElement(joint, "dynamics")
        dynamics.attrib.update({"damping": "0.05", "friction": "0.02"})

    for link in root.findall("link"):
        link_name = str(link.get("name"))
        mesh_name = skeleton_meshes.get(link_name)
        if not mesh_name:
            continue
        visual = link.find("visual")
        if visual is not None:
            mesh = visual.find("./geometry/mesh")
            if mesh is not None:
                mesh.set("filename", f"meshes/skeleton/{mesh_name}")
            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material")
            material.set("name", "physical_mount_v1_skeleton_white")
            color = material.find("color")
            if color is None:
                color = ET.SubElement(material, "color")
            color.set("rgba", "0.93 0.95 0.98 1")

        collision = link.find("collision")
        if collision is None:
            collision = ET.SubElement(
                link, "collision", {"name": f"{link_name}_skeleton_collision"}
            )
            ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            geometry = ET.SubElement(collision, "geometry")
            mesh = ET.SubElement(geometry, "mesh")
        else:
            mesh = collision.find("./geometry/mesh")
            if mesh is None:
                geometry = collision.find("geometry")
                if geometry is None:
                    geometry = ET.SubElement(collision, "geometry")
                for child in list(geometry):
                    geometry.remove(child)
                mesh = ET.SubElement(geometry, "mesh")
        mesh.set("filename", f"meshes/skeleton/{mesh_name}")

        for servo in servos_by_link.get(link_name, []):
            servo_file = Path(str(servo["output_mesh"])).name
            visual = ET.SubElement(
                link,
                "visual",
                {"name": f"{servo['id']}_{servo['joint']}_blue_servo_visual"},
            )
            ET.SubElement(visual, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            geometry = ET.SubElement(visual, "geometry")
            ET.SubElement(
                geometry,
                "mesh",
                {"filename": f"meshes/servos/{servo_file}"},
            )
            material = ET.SubElement(
                visual,
                "material",
                {"name": f"{servo['id']}_sts3250_blue"},
            )
            ET.SubElement(material, "color", {"rgba": "0.086 0.467 1.0 1"})

            collision = ET.SubElement(
                link,
                "collision",
                {"name": f"{servo['id']}_{servo['joint']}_servo_collision"},
            )
            ET.SubElement(collision, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            geometry = ET.SubElement(collision, "geometry")
            ET.SubElement(
                geometry,
                "mesh",
                {"filename": f"meshes/servos/{servo_file}"},
            )

    update_inertias(root, servos_by_link)
    ET.indent(tree, space="  ")
    URDF_ROOT.mkdir(parents=True, exist_ok=True)
    tree.write(URDF_PATH, encoding="utf-8", xml_declaration=True)
    return root


def copy_urdf_meshes(
    skeleton_meshes: dict[str, str],
    manifest: list[dict[str, object]],
) -> None:
    URDF_SKELETON_DIR.mkdir(parents=True, exist_ok=True)
    URDF_SERVO_DIR.mkdir(parents=True, exist_ok=True)
    for mesh_name in sorted(set(skeleton_meshes.values())):
        shutil.copy2(SKELETON_DIR / mesh_name, URDF_SKELETON_DIR / mesh_name)
    for servo in manifest:
        source = ROOT / str(servo["output_mesh"])
        shutil.copy2(source, URDF_SERVO_DIR / source.name)


def write_actuator_config(
    manifest: list[dict[str, object]],
) -> None:
    bus_ids = {
        "left_shoulder_yaw": 11,
        "left_shoulder_pitch": 12,
        "left_elbow_yaw": 13,
        "right_shoulder_yaw": 21,
        "right_shoulder_pitch": 22,
        "right_elbow_yaw": 23,
        "left_hip_yaw": 31,
        "left_hip_roll": 32,
        "left_hip_pitch": 33,
        "left_knee_pitch": 34,
        "left_ankle_pitch": 35,
        "right_hip_yaw": 41,
        "right_hip_roll": 42,
        "right_hip_pitch": 43,
        "right_knee_pitch": 44,
        "right_ankle_pitch": 45,
    }
    limits = _load_guarded_limits()
    items = []
    for servo in manifest:
        joint = str(servo["joint"])
        lower, upper = limits[joint]
        items.append(
            {
                "id": servo["id"],
                "joint": joint,
                "bus_id_candidate": bus_ids[joint],
                "bus_id_gate": "REQUIRES_PHYSICAL_BUS_SCAN",
                "model": "FEETECH STS3250-C001",
                "protocol": "TTL half-duplex serial",
                "supply_voltage_v": 12.0,
                "mass_kg": STS3250_MASS_KG,
                "mass_tolerance_kg": STS3250_MASS_TOLERANCE_KG,
                "rated_torque_nm": 1.569,
                "stall_torque_nm": 4.903,
                "stall_current_a": 4.2,
                "no_load_speed_rad_s": 7.853982,
                "encoder_counts_per_revolution": 4096,
                "neutral_count_candidate": 2048,
                "neutral_count_gate": "REQUIRES_JOG_CALIBRATION",
                "urdf_lower_rad": lower,
                "urdf_upper_rad": upper,
                "urdf_to_servo_direction_sign": "REQUIRES_JOG_CALIBRATION",
                "hardware_zero_offset_counts": "REQUIRES_PHYSICAL_CALIBRATION",
                "mounting_geometry": servo["output_mesh"],
                "mounting_geometry_semantics": (
                    "original installed STS3215-family geometry retained as "
                    "the STS3250 form-factor placement reference"
                ),
            }
        )
    ACTUATOR_PATH.parent.mkdir(parents=True, exist_ok=True)
    ACTUATOR_PATH.write_text(
        json.dumps(
            {
                "schema": "zeroth01.physical_mount_v1.actuators.v1",
                "count": 16,
                "target_model": "FEETECH STS3250-C001",
                "mechanical_interface": {
                    "case_mm": [45.22, 24.72, 35.0],
                    "shaft_offset_from_short_end_mm": 12.5,
                    "output": "dual 25T / OD 5.9 mm",
                    "output_retention": "M3x6",
                    "horn": "4xM3 on 14 mm PCD",
                    "case_mount": "4xM2 per STS3250 drawing",
                },
                "mass_accounting": (
                    "Each archived 55 g actuator already present in the source "
                    "link inertia is upgraded by a 19.5 g point-mass delta at "
                    "the extracted actuator-region centre."
                ),
                "servos": items,
            },
            indent=2,
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )


def _load_kinematic_model(
    root: ET.Element,
) -> tuple[str, list[dict[str, object]]]:
    child_links = {
        str(joint.find("child").get("link"))
        for joint in root.findall("joint")
        if joint.find("child") is not None
    }
    link_names = {str(link.get("name")) for link in root.findall("link")}
    base_links = sorted(link_names - child_links)
    if len(base_links) != 1:
        raise ValueError(f"expected one base link, got {base_links}")
    joints: list[dict[str, object]] = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        joints.append(
            {
                "name": str(joint.get("name")),
                "type": str(joint.get("type")),
                "parent": str(joint.find("parent").get("link")),
                "child": str(joint.find("child").get("link")),
                "origin": (
                    _rpy_matrix(
                        _parse_vec(origin.get("rpy") if origin is not None else None)
                    ),
                    _parse_vec(origin.get("xyz") if origin is not None else None),
                ),
                "axis": _parse_vec(axis.get("xyz") if axis is not None else None),
                "lower": float(limit.get("lower", "0")) if limit is not None else 0.0,
                "upper": float(limit.get("upper", "0")) if limit is not None else 0.0,
            }
        )
    return base_links[0], joints


def _forward_kinematics(
    base_link: str,
    joints: list[dict[str, object]],
    positions: dict[str, float],
) -> dict[str, tuple[tuple[tuple[float, ...], ...], tuple[float, ...]]]:
    transforms = {base_link: (IDENTITY_R, (0.0, 0.0, 0.0))}
    pending = joints[:]
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = str(joint["parent"])
            if parent not in transforms:
                continue
            motion = (
                _axis_angle_matrix(
                    joint["axis"],
                    float(positions.get(str(joint["name"]), 0.0)),
                ),
                (0.0, 0.0, 0.0),
            )
            transforms[str(joint["child"])] = _tf_mul(
                transforms[parent],
                _tf_mul(joint["origin"], motion),
            )
            pending.remove(joint)
            progressed = True
        if not progressed:
            raise ValueError("URDF tree could not be resolved")
    return transforms


def _vtk_matrix(transform: tuple[object, object]) -> vtkMatrix4x4:
    rotation, translation = transform
    matrix = vtkMatrix4x4()
    matrix.Identity()
    for row in range(3):
        for column in range(3):
            matrix.SetElement(row, column, float(rotation[row][column]))
        matrix.SetElement(row, 3, float(translation[row]))
    return matrix


def _actor_for_stl(path: Path, color: tuple[float, float, float], opacity: float) -> vtkActor:
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(reader.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    actor.GetProperty().SetColor(*color)
    actor.GetProperty().SetOpacity(opacity)
    actor.GetProperty().EdgeVisibilityOn()
    actor.GetProperty().SetEdgeColor(0.14, 0.17, 0.22)
    actor.GetProperty().SetLineWidth(0.25)
    return actor


def _capture(window: vtkRenderWindow, path: Path) -> None:
    capture = vtkWindowToImageFilter()
    capture.SetInput(window)
    capture.SetInputBufferTypeToRGB()
    capture.ReadFrontBufferOff()
    capture.Update()
    path.parent.mkdir(parents=True, exist_ok=True)
    writer = vtkPNGWriter()
    writer.SetFileName(str(path))
    writer.SetInputConnection(capture.GetOutputPort())
    writer.Write()


def render_review(
    root: ET.Element,
    skeleton_meshes: dict[str, str],
    servos_by_link: dict[str, list[dict[str, object]]],
    xray_opacity: float,
    frame_count: int,
) -> None:
    base_link, joints = _load_kinematic_model(root)
    renderer = vtkRenderer()
    renderer.SetBackground(0.965, 0.975, 0.99)
    window = vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(1000, 1400)
    window.SetMultiSamples(4)
    window.AddRenderer(renderer)

    actors_by_link: dict[str, list[vtkActor]] = defaultdict(list)
    skeleton_actors: list[vtkActor] = []
    for link, mesh_name in skeleton_meshes.items():
        actor = _actor_for_stl(
            SKELETON_DIR / mesh_name,
            (0.93, 0.95, 0.98),
            1.0,
        )
        renderer.AddActor(actor)
        actors_by_link[link].append(actor)
        skeleton_actors.append(actor)
    for link, servos in servos_by_link.items():
        for servo in servos:
            actor = _actor_for_stl(
                ROOT / str(servo["output_mesh"]),
                (0.086, 0.467, 1.0),
                1.0,
            )
            renderer.AddActor(actor)
            actors_by_link[link].append(actor)

    moving = [joint for joint in joints if joint["type"] == "revolute"]

    def apply_pose(positions: dict[str, float]) -> None:
        transforms = _forward_kinematics(base_link, joints, positions)
        for link, actors in actors_by_link.items():
            matrix = _vtk_matrix(transforms[link])
            for actor in actors:
                actor.SetUserMatrix(matrix)

    apply_pose({})
    window.Render()
    visible_bounds = renderer.ComputeVisiblePropBounds()
    visible_center = (
        (visible_bounds[0] + visible_bounds[1]) / 2.0,
        (visible_bounds[2] + visible_bounds[3]) / 2.0,
        (visible_bounds[4] + visible_bounds[5]) / 2.0,
    )
    visible_size = (
        visible_bounds[1] - visible_bounds[0],
        visible_bounds[3] - visible_bounds[2],
        visible_bounds[5] - visible_bounds[4],
    )
    visible_diagonal = math.sqrt(sum(value * value for value in visible_size))
    camera = renderer.GetActiveCamera()
    camera.ParallelProjectionOn()
    camera.SetFocalPoint(*visible_center)
    # The archived URDF inserts a +90 degree base yaw, so robot front/back is
    # global X and left/right is global Y.
    camera.SetPosition(
        visible_center[0] + visible_diagonal * 2.4,
        visible_center[1],
        visible_center[2],
    )
    camera.SetViewUp(0.0, 0.0, 1.0)
    camera.SetParallelScale(max(visible_size[2], visible_size[1] * 1.25) * 0.56)
    renderer.ResetCameraClippingRange()
    window.Render()
    _capture(window, FRONT_SNAPSHOT)

    for actor in skeleton_actors:
        actor.GetProperty().SetOpacity(xray_opacity)
    camera.SetPosition(
        visible_center[0] + visible_diagonal * 1.8,
        visible_center[1] - visible_diagonal * 1.8,
        visible_center[2] + visible_diagonal * 0.8,
    )
    camera.SetParallelScale(max(visible_size) * 0.72)
    renderer.ResetCameraClippingRange()
    window.Render()
    _capture(window, XRAY_SNAPSHOT)

    camera.SetPosition(
        visible_center[0] + visible_diagonal * 2.4,
        visible_center[1],
        visible_center[2],
    )
    camera.SetParallelScale(max(visible_size[2], visible_size[1] * 1.25) * 0.56)
    renderer.ResetCameraClippingRange()
    MOTION_ROOT.mkdir(parents=True, exist_ok=True)
    frame_paths: list[Path] = []
    for frame_index in range(frame_count):
        phase = 2.0 * math.pi * frame_index / max(1, frame_count - 1)
        positions: dict[str, float] = {}
        for joint_index, joint in enumerate(moving):
            lower = float(joint["lower"])
            upper = float(joint["upper"])
            amplitude = min(
                math.radians(8.0),
                max(0.0, -lower) * 0.6 if lower < 0.0 else math.radians(3.0),
                max(0.0, upper) * 0.6 if upper > 0.0 else math.radians(3.0),
            )
            positions[str(joint["name"])] = amplitude * math.sin(
                phase + (joint_index % 4) * math.pi / 2.0
            )
        apply_pose(positions)
        window.Render()
        frame_path = MOTION_ROOT / f"physical_mount_motion_{frame_index:03d}.png"
        _capture(window, frame_path)
        frame_paths.append(frame_path)

    images = [Image.open(path).convert("P", palette=Image.Palette.ADAPTIVE) for path in frame_paths]
    try:
        images[0].save(
            MOTION_GIF,
            save_all=True,
            append_images=images[1:],
            duration=90,
            loop=0,
            optimize=False,
        )
    finally:
        for image in images:
            image.close()
    window.Finalize()


def write_gate(
    manifest: list[dict[str, object]],
    skeleton_meshes: dict[str, str],
    root: ET.Element,
) -> None:
    moving_joints = [
        joint
        for joint in root.findall("joint")
        if joint.get("type") == "revolute"
    ]
    servo_ids = [str(item["id"]) for item in manifest]
    family_failures = [
        str(item["id"])
        for item in manifest
        if item["family_envelope_gate"] != "PASS"
    ]
    gate = {
        "schema": "zeroth01.physical_mount_v1.source_component_gate.v1",
        "physical_source": "archived assembled Zeroth/K-Scale Z-Bot STL geometry",
        "checks": {
            "extracted_servo_component_count": {
                "value": len(manifest),
                "expected": 16,
                "gate": "PASS" if len(manifest) == 16 else "FAIL",
            },
            "unique_servo_ids": {
                "value": len(set(servo_ids)),
                "expected": 16,
                "gate": "PASS" if len(set(servo_ids)) == 16 else "FAIL",
            },
            "servo_family_envelope": {
                "failures": family_failures,
                "gate": "PASS" if not family_failures else "FAIL",
            },
            "remaining_skeleton_link_mesh_count": {
                "value": len(skeleton_meshes),
                "expected": 20,
                "gate": "PASS" if len(skeleton_meshes) == 20 else "FAIL",
            },
            "actuated_joint_count": {
                "value": len(moving_joints),
                "expected": 16,
                "gate": "PASS" if len(moving_joints) == 16 else "FAIL",
            },
            "fixed_gripper_count": {
                "value": sum(
                    1
                    for joint in root.findall("joint")
                    if joint.get("name") in {"left_gripper", "right_gripper"}
                    and joint.get("type") == "fixed"
                ),
                "expected": 2,
                "gate": "PASS"
                if sum(
                    1
                    for joint in root.findall("joint")
                    if joint.get("name") in {"left_gripper", "right_gripper"}
                    and joint.get("type") == "fixed"
                )
                == 2
                else "FAIL",
            },
        },
        "claim_boundary": (
            "PASS proves that 16 actuator-shaped regions were extracted in "
            "their original assembled link coordinates and remain connected "
            "to the original carrier geometry through the source kinematic "
            "chain. Dynamic collision, fastener strength, cable routing and "
            "purchased STS3250 tolerance require their separate gates."
        ),
    }
    gate["overall"] = (
        "PASS"
        if all(
            item["gate"] == "PASS"
            for item in gate["checks"].values()
        )
        else "FAIL"
    )
    GATE_PATH.write_text(
        json.dumps(gate, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Split the original assembled Zeroth servo bodies from their "
            "carrier meshes and build a 16-DoF STS3250 physical-mount review."
        )
    )
    parser.add_argument("--frame-count", type=int, default=25)
    args = parser.parse_args()
    if args.frame_count < 3:
        raise ValueError("--frame-count must be at least 3")

    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8"))
    source_urdf = ROOT / str(config["source_urdf"])
    source_mesh_root = ROOT / str(config["source_mesh_root"])
    source_tree = ET.parse(source_urdf)
    source_root = source_tree.getroot()
    link_meshes = _source_link_meshes(source_root)

    for directory in (
        SKELETON_DIR,
        SERVO_DIR,
        URDF_SKELETON_DIR,
        URDF_SERVO_DIR,
        REPORT_ROOT,
        SNAPSHOT_ROOT,
        MOTION_ROOT,
    ):
        directory.mkdir(parents=True, exist_ok=True)

    manifest, skeleton_meshes, servos_by_link = split_source_geometry(
        config,
        source_mesh_root,
        link_meshes,
    )
    _write_manifest(manifest)
    copy_urdf_meshes(skeleton_meshes, manifest)
    root = build_urdf(
        config,
        source_urdf,
        skeleton_meshes,
        servos_by_link,
    )
    write_actuator_config(manifest)
    render_review(
        root,
        skeleton_meshes,
        servos_by_link,
        float(config["appearance"]["skeleton_xray_opacity"]),
        args.frame_count,
    )
    write_gate(manifest, skeleton_meshes, root)

    print(URDF_PATH)
    print(MANIFEST_PATH)
    print(ACTUATOR_PATH)
    print(FRONT_SNAPSHOT)
    print(XRAY_SNAPSHOT)
    print(MOTION_GIF)
    print(GATE_PATH)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
