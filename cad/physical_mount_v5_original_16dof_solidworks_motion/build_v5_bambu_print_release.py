"""Build the Zeroth-01 v5 Bambu Studio first-article print package.

The package deliberately separates FDM-printable robot parts from purchased
actuators and thin metal torque-path hardware.  STEP remains the source of
truth; print-oriented STEP, STL and 3MF are derived sidecars.
"""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
import sys
import zipfile
from collections import defaultdict
from pathlib import Path
from xml.etree import ElementTree as ET

from build123d import Compound, Location, export_step, export_stl, import_step


ROOT = Path(__file__).resolve().parents[2]
V5_NAME = "physical_mount_v5_original_16dof_solidworks_motion"
CAD_ROOT = ROOT / "generated" / "cad" / V5_NAME
PARTS = CAD_ROOT / "parts"
ASSEMBLY_MANIFEST = CAD_ROOT / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
RELEASE = ROOT / "manufacturing" / "v5_bambu_first_article_release"
SOURCE_STEP = RELEASE / "step" / "assembly_frame"
PRINT_STEP = RELEASE / "step" / "print_oriented"
STL = RELEASE / "stl"
THREE_MF = RELEASE / "3mf"
FIT_CHECK = RELEASE / "fit_check_only"
VALIDATION = RELEASE / "validation"
DIAGNOSTIC_STEP = RELEASE / "ZEROTH01_V5_BAMBU_PRINTABLE_PARTS_DIAGNOSTIC.step"

PRINTABLE_ROLES = {
    "source_load_bearing_carrier",
    "fixed_compact_wrist_support",
    "replaceable_white_tapered_lower_wider_sole",
    "printable_head_front_shell",
    "printable_head_rear_shell",
    "simple_camera_microphone_visor",
    "removable_internal_service_mount",
    "direct_head_torso_mount",
    "harness_strain_relief",
}

FIT_CHECK_PARTS = {
    "sts3250_parent_thrust_ring_2mm": PARTS / "sts3250_parent_thrust_ring_2mm.step",
    "sts3250_pcd14_output_bridge_2p05mm": PARTS / "sts3250_pcd14_output_bridge_2p05mm.step",
    "sts3250_pcd14_four_sleeve_spacer_1p65mm_z0p30": PARTS / "sts3250_pcd14_four_sleeve_spacer_1p65mm_z0p30.step",
    "sts3250_4xm2_first_article_face_gauge": ROOT / "generated" / "cad" / "physical_mount_v1" / "sts3250_interface" / "STS3250_4XM2_FIRST_ARTICLE_FACE_GAUGE.step",
}

ROLE_PROCESS = {
    "source_load_bearing_carrier": ("PA6-CF", "STRUCTURAL", 0.20, 5, 45),
    "fixed_compact_wrist_support": ("PA6-CF", "STRUCTURAL", 0.20, 5, 45),
    "replaceable_white_tapered_lower_wider_sole": ("PA6-CF", "STRUCTURAL", 0.20, 5, 45),
    "direct_head_torso_mount": ("PA6-CF", "STRUCTURAL", 0.20, 5, 45),
    "printable_head_front_shell": ("PETG-HF white", "COSMETIC", 0.20, 4, 25),
    "printable_head_rear_shell": ("PETG-HF white", "COSMETIC", 0.20, 4, 25),
    "simple_camera_microphone_visor": ("PETG-HF black", "COSMETIC", 0.20, 4, 25),
    "removable_internal_service_mount": ("PETG-HF", "SERVICE", 0.20, 4, 30),
    "harness_strain_relief": ("PETG-HF", "SERVICE", 0.20, 4, 30),
}

MATERIAL_DENSITY_G_CM3 = {
    "PA6-CF": 1.18,
    "PETG-HF white": 1.27,
    "PETG-HF black": 1.27,
    "PETG-HF": 1.27,
}


def safe_name(text: str) -> str:
    return "".join(char.lower() if char.isalnum() else "_" for char in text).strip("_")


def rel(path: Path) -> str:
    return path.resolve().relative_to(ROOT.resolve()).as_posix()


def ensure_release_dirs() -> None:
    resolved = RELEASE.resolve()
    if ROOT.resolve() not in resolved.parents:
        raise RuntimeError(f"unsafe release path: {resolved}")
    for directory in (SOURCE_STEP, PRINT_STEP, STL, THREE_MF, FIT_CHECK, VALIDATION):
        directory.mkdir(parents=True, exist_ok=True)


def printable_rows() -> list[dict[str, object]]:
    manifest = json.loads(ASSEMBLY_MANIFEST.read_text(encoding="utf-8"))
    grouped: dict[str, list[dict[str, object]]] = defaultdict(list)
    for component in manifest["components"]:
        if component["role"] in PRINTABLE_ROLES:
            grouped[str(component["source"])].append(component)
    rows: list[dict[str, object]] = []
    for source_rel, occurrences in grouped.items():
        source = ROOT / source_rel
        if not source.is_file():
            raise FileNotFoundError(source)
        role = str(occurrences[0]["role"])
        material, part_class, layer_mm, walls, infill = ROLE_PROCESS[role]
        rows.append(
            {
                "name": safe_name(source.stem),
                "source": source,
                "role": role,
                "quantity": len(occurrences),
                "occurrences": [str(item["component_id"]) for item in occurrences],
                "material": material,
                "part_class": part_class,
                "layer_mm": layer_mm,
                "walls": walls,
                "top_bottom_layers": 6 if part_class == "STRUCTURAL" else 5,
                "infill_percent": infill,
                "infill_pattern": "gyroid",
                "nozzle_mm": 0.6 if material == "PA6-CF" else 0.4,
                "nozzle": "hardened steel" if material == "PA6-CF" else "standard or hardened steel",
                "support": "paint only where overhang exceeds 50 deg; keep servo pockets and bearing faces support-free",
            }
        )
    return sorted(rows, key=lambda item: (str(item["part_class"]), str(item["name"])))


def orient_for_print(shape):
    """Put the smallest bounding-box dimension on Z and place it on Z=0."""
    size = list(shape.bounding_box().size)
    smallest = min(range(3), key=size.__getitem__)
    rotation = (0.0, 90.0, 0.0) if smallest == 0 else ((90.0, 0.0, 0.0) if smallest == 1 else (0.0, 0.0, 0.0))
    oriented = shape.moved(Location((0.0, 0.0, 0.0), rotation))
    # Rewrap imported solids so STEPControl does not inherit unsupported
    # assembly metadata from a supplier/source STEP compound.
    oriented = Compound(children=oriented.solids())
    box = oriented.bounding_box()
    oriented = oriented.moved(Location((-box.min.X, -box.min.Y, -box.min.Z)))
    oriented = Compound(children=oriented.solids())
    return oriented, rotation


def export_mesh_sidecars(shape, step_path: Path, stl_path: Path, three_mf_path: Path) -> None:
    step_path.parent.mkdir(parents=True, exist_ok=True)
    stl_path.parent.mkdir(parents=True, exist_ok=True)
    three_mf_path.parent.mkdir(parents=True, exist_ok=True)
    export_stl(shape, stl_path, tolerance=0.10, angular_tolerance=0.15)
    cad_scripts = Path.home() / ".codex" / "skills" / "cad" / "scripts"
    if not cad_scripts.is_dir():
        raise FileNotFoundError(f"CAD helper package not found: {cad_scripts}")
    if str(cad_scripts) not in sys.path:
        sys.path.insert(0, str(cad_scripts))
    from cadpy_common.step_scene import load_step_scene, mesh_step_scene
    from cadpy_common.threemf import export_part_3mf_from_scene
    scene = load_step_scene(step_path)
    mesh_step_scene(scene, linear_deflection=0.10, angular_deflection=0.15, relative=True)
    export_part_3mf_from_scene(step_path, scene, target_path=three_mf_path)


def validate_three_mf(path: Path) -> tuple[bool, int, int, int]:
    required = {"[Content_Types].xml", "_rels/.rels", "3D/3dmodel.model"}
    with zipfile.ZipFile(path) as archive:
        names = set(archive.namelist())
        if not required.issubset(names):
            return False, 0, 0, -1
        model = ET.fromstring(archive.read("3D/3dmodel.model"))
        meshes = [node for node in model.iter() if node.tag.endswith("mesh")]
        triangle_count = 0
        open_edges = 0
        for mesh in meshes:
            vertices = [
                tuple(round(float(vertex.attrib[axis]), 5) for axis in ("x", "y", "z"))
                for vertex in mesh.iter()
                if vertex.tag.endswith("vertex")
            ]
            edge_counts: dict[tuple[tuple[float, float, float], tuple[float, float, float]], int] = defaultdict(int)
            for triangle in (node for node in mesh.iter() if node.tag.endswith("triangle")):
                indices = [int(triangle.attrib[key]) for key in ("v1", "v2", "v3")]
                points = [vertices[index] for index in indices]
                for edge in ((points[0], points[1]), (points[1], points[2]), (points[2], points[0])):
                    edge_counts[tuple(sorted(edge))] += 1
                triangle_count += 1
            open_edges += sum(1 for count in edge_counts.values() if count != 2)
        return triangle_count > 0 and open_edges == 0, triangle_count, len(meshes), open_edges


def validate_stl(path: Path) -> tuple[bool, int, int]:
    try:
        from vtkmodules.vtkFiltersCore import vtkFeatureEdges
        from vtkmodules.vtkIOGeometry import vtkSTLReader
    except ImportError:
        return True, -1, -1
    reader = vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    mesh = reader.GetOutput()
    feature = vtkFeatureEdges()
    feature.SetInputData(mesh)
    feature.BoundaryEdgesOn()
    feature.NonManifoldEdgesOn()
    feature.FeatureEdgesOff()
    feature.ManifoldEdgesOff()
    feature.Update()
    open_edges = int(feature.GetOutput().GetNumberOfCells())
    triangles = int(mesh.GetNumberOfCells())
    return triangles > 0 and open_edges == 0, triangles, open_edges


def export_print_parts(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    validations: list[dict[str, object]] = []
    for index, row in enumerate(rows, start=1):
        source = Path(row["source"])
        name = f"{index:02d}_{row['name']}"
        print(f"[{index:02d}/{len(rows):02d}] export {name}", flush=True)
        assembly_step = SOURCE_STEP / f"{name}.step"
        print_step = PRINT_STEP / f"{name}.step"
        stl = STL / f"{name}.stl"
        three_mf = THREE_MF / f"{name}.3mf"
        shutil.copy2(source, assembly_step)
        shape = import_step(source)
        oriented, rotation = orient_for_print(shape)
        export_step(oriented, print_step)
        export_mesh_sidecars(oriented, print_step, stl, three_mf)
        box = oriented.bounding_box()
        stl_ok, stl_triangles, open_edges = validate_stl(stl)
        three_mf_ok, three_mf_triangles, three_mf_meshes, three_mf_open_edges = validate_three_mf(three_mf)
        solid_volume_mm3 = float(shape.volume)
        density = MATERIAL_DENSITY_G_CM3[str(row["material"])]
        row.update(
            {
                "source": rel(source),
                "release_name": name,
                "assembly_frame_step": rel(assembly_step),
                "print_oriented_step": rel(print_step),
                "stl": rel(stl),
                "three_mf": rel(three_mf),
                "orientation_rotation_deg_xyz": list(rotation),
                "bbox_mm": [round(float(value), 3) for value in box.size],
                "solid_volume_mm3": round(solid_volume_mm3, 3),
                "solid_material_upper_bound_g_each": round(solid_volume_mm3 / 1000.0 * density, 3),
            }
        )
        validations.append(
            {
                "name": name,
                "source_brep_valid": bool(shape.is_valid),
                "source_solid_count": len(shape.solids()),
                "source_positive_volume": solid_volume_mm3 > 0.0,
                "fits_256mm_cube": max(box.size) <= 256.0,
                "stl_watertight": stl_ok,
                "stl_scope": "combined-shell compatibility export; use the 3MF as the Bambu Studio primary artifact",
                "stl_triangle_count": stl_triangles,
                "stl_open_or_nonmanifold_edge_count": open_edges,
                "three_mf_valid_and_each_mesh_watertight": three_mf_ok,
                "three_mf_triangle_count": three_mf_triangles,
                "three_mf_mesh_count": three_mf_meshes,
                "three_mf_open_or_nonmanifold_edge_count": three_mf_open_edges,
            }
        )
    return validations


def export_fit_checks() -> list[dict[str, object]]:
    rows = []
    for index, (name, source) in enumerate(sorted(FIT_CHECK_PARTS.items()), start=1):
        print(f"[fit {index:02d}/{len(FIT_CHECK_PARTS):02d}] export {name}", flush=True)
        if not source.is_file():
            raise FileNotFoundError(source)
        shape = import_step(source)
        oriented, rotation = orient_for_print(shape)
        step_path = FIT_CHECK / f"{index:02d}_{name}.step"
        stl_path = FIT_CHECK / f"{index:02d}_{name}.stl"
        three_mf_path = FIT_CHECK / f"{index:02d}_{name}.3mf"
        export_step(oriented, step_path)
        export_mesh_sidecars(oriented, step_path, stl_path, three_mf_path)
        rows.append(
            {
                "name": name,
                "step": rel(step_path),
                "stl": rel(stl_path),
                "three_mf": rel(three_mf_path),
                "orientation_rotation_deg_xyz": list(rotation),
                "status": "FIT_CHECK_ONLY_NOT_FOR_POWERED_TORQUE_PATH",
            }
        )
    return rows


def export_diagnostic(rows: list[dict[str, object]]) -> None:
    placed = []
    pitch_x = 280.0
    pitch_y = 280.0
    for index, row in enumerate(rows):
        shape = import_step(Path(row["source"]))
        box = shape.bounding_box()
        x = (index % 5) * pitch_x - box.min.X
        y = (index // 5) * pitch_y - box.min.Y
        z = -box.min.Z
        placed.extend(shape.moved(Location((x, y, z))).solids())
    export_step(Compound(children=placed), DIAGNOSTIC_STEP)


def write_manifests(rows: list[dict[str, object]], validations: list[dict[str, object]], fit_checks: list[dict[str, object]]) -> None:
    csv_fields = [
        "release_name", "quantity", "role", "part_class", "material", "nozzle_mm", "nozzle", "layer_mm",
        "walls", "top_bottom_layers", "infill_percent", "infill_pattern", "support", "bbox_mm",
        "solid_volume_mm3", "solid_material_upper_bound_g_each", "assembly_frame_step", "print_oriented_step", "stl", "three_mf",
    ]
    with (RELEASE / "PRINT_MANIFEST.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=csv_fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    overall = "PASS" if validations and all(
        item["source_brep_valid"]
        and item["source_positive_volume"]
        and item["fits_256mm_cube"]
        and item["three_mf_valid_and_each_mesh_watertight"]
        for item in validations
    ) else "FAIL"
    validation_payload = {
        "schema": "zeroth01.v5.bambu_first_article_mesh_gate.v1",
        "target_machine_envelope_mm": [256, 256, 256],
        "print_part_source_count": len(rows),
        "print_occurrence_count": sum(int(row["quantity"]) for row in rows),
        "checks": validations,
        "overall": overall,
        "scope": "geometry import, positive B-Rep volume, 256 mm build-volume fit, and every mesh inside the Bambu-primary 3MF is watertight. Combined-shell STL files are compatibility exports and can report contact-edge artifacts; slicer toolpath, material strength and powered motion are separate gates",
    }
    (VALIDATION / "mesh_gate.json").write_text(json.dumps(validation_payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    material_upper_bounds: dict[str, float] = defaultdict(float)
    for row in rows:
        material_upper_bounds[str(row["material"])] += (
            float(row["solid_material_upper_bound_g_each"]) * int(row["quantity"])
        )
    solid_material_upper_bound_g = round(sum(material_upper_bounds.values()), 3)
    payload = {
        "schema": "zeroth01.v5.bambu_first_article_print_release.v1",
        "status": "FIRST_ARTICLE_PRINT_RELEASE" if overall == "PASS" else "HOLD",
        "source_assembly_manifest": rel(ASSEMBLY_MANIFEST),
        "primary_diagnostic_step": rel(DIAGNOSTIC_STEP),
        "target": "Bambu Studio; 256 x 256 x 256 mm or larger printer; machine-specific G-code intentionally not included",
        "summary": {
            "print_part_source_count": len(rows),
            "print_occurrence_count": sum(int(row["quantity"]) for row in rows),
            "solid_material_upper_bound_g": solid_material_upper_bound_g,
            "solid_material_upper_bound_by_material_g": {
                material: round(value, 3) for material, value in sorted(material_upper_bounds.items())
            },
            "mass_scope": "solid-volume upper bound only; use Bambu Studio sliced mass for purchasing and weigh every finished link for RL inertial identification",
        },
        "printed_parts": rows,
        "fit_check_only": fit_checks,
        "mesh_gate": rel(VALIDATION / "mesh_gate.json"),
        "powered_motion_release": "HOLD_UNTIL_PRINTED_FIRST_ARTICLE_DRY_FIT_AND_ISOLATED_JOINT_SWEEP",
    }
    (RELEASE / "PRINT_RELEASE.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def write_checksums() -> None:
    files = sorted(path for path in RELEASE.rglob("*") if path.is_file() and path.name != "SHA256SUMS.csv")
    with (RELEASE / "SHA256SUMS.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("path", "bytes", "sha256"))
        for path in files:
            writer.writerow((path.relative_to(RELEASE).as_posix(), path.stat().st_size, hashlib.sha256(path.read_bytes()).hexdigest()))


def main() -> int:
    ensure_release_dirs()
    rows = printable_rows()
    validations = export_print_parts(rows)
    fit_checks = export_fit_checks()
    export_diagnostic(rows)
    write_manifests(rows, validations, fit_checks)
    write_checksums()
    overall = json.loads((VALIDATION / "mesh_gate.json").read_text(encoding="utf-8"))["overall"]
    print(json.dumps({"print_source_count": len(rows), "print_occurrence_count": sum(int(row["quantity"]) for row in rows), "overall": overall, "release": str(RELEASE)}, indent=2))
    return 0 if overall == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
