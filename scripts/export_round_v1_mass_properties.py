from __future__ import annotations

import csv
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from build123d import CenterOf, import_step


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
PARTS_DIR = ROOT / "generated" / "cad" / "round_v1" / "parts"
BASE_URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_ready.urdf"
JSON_OUTPUT = (
    ROOT / "generated" / "config" / "round_v1_mass_properties.json"
)
PART_REPORT = ROOT / "reports" / "round_v1_printed_part_mass_properties.csv"
LINK_REPORT = ROOT / "reports" / "round_v1_link_inertial_overlay.csv"

# A transparent, reproducible nominal value. The final URDF must be updated
# from weighed printed parts before hardware deployment.
PETG_DENSITY_KG_PER_MM3 = 1.27e-6

PARTS = [
    {
        "name": "chest_front",
        "file": "ZEROTH01_ROUND_V1_CHEST_FRONT.step",
        "installed_link": "Torso",
        "installed_quantity": 1,
        "classification": "cosmetic_non_load_bearing_shell",
    },
    {
        "name": "chest_back",
        "file": "ZEROTH01_ROUND_V1_CHEST_BACK.step",
        "installed_link": "Torso",
        "installed_quantity": 1,
        "classification": "cosmetic_non_load_bearing_shell",
    },
    {
        "name": "head_front",
        "file": "ZEROTH01_ROUND_V1_HEAD_FRONT.step",
        "installed_link": "Torso",
        "installed_quantity": 1,
        "classification": "cosmetic_non_load_bearing_shell",
    },
    {
        "name": "head_back",
        "file": "ZEROTH01_ROUND_V1_HEAD_BACK.step",
        "installed_link": "Torso",
        "installed_quantity": 1,
        "classification": "cosmetic_non_load_bearing_shell",
    },
    {
        "name": "pelvis_front",
        "file": "ZEROTH01_ROUND_V1_PELVIS_FRONT.step",
        "installed_link": "Torso",
        "installed_quantity": 1,
        "classification": "cosmetic_non_load_bearing_shell",
    },
    {
        "name": "pelvis_back",
        "file": "ZEROTH01_ROUND_V1_PELVIS_BACK.step",
        "installed_link": "Torso",
        "installed_quantity": 1,
        "classification": "cosmetic_non_load_bearing_shell",
    },
    {
        "name": "muzzle_badge",
        "file": "ZEROTH01_ROUND_V1_MUZZLE_BADGE.step",
        "installed_link": "Torso",
        "installed_quantity": 1,
        "classification": "cosmetic_non_load_bearing_badge",
    },
    {
        "name": "visor_badge",
        "file": "ZEROTH01_ROUND_V1_VISOR_BADGE.step",
        "installed_link": "Torso",
        "installed_quantity": 1,
        "classification": "cosmetic_non_load_bearing_badge",
    },
    {
        "name": "left_sole",
        "file": "ZEROTH01_ROUND_V1_LEFT_SOLE.step",
        "installed_link": "foot_left",
        "installed_quantity": 1,
        "classification": "contact_prototype_not_structural_signoff",
    },
    {
        "name": "right_sole",
        "file": "ZEROTH01_ROUND_V1_RIGHT_SOLE.step",
        "installed_link": "foot_right",
        "installed_quantity": 1,
        "classification": "contact_prototype_not_structural_signoff",
    },
    {
        "name": "generic_joint_ring",
        "file": "ZEROTH01_ROUND_V1_JOINT_RING.step",
        "installed_link": None,
        "installed_quantity": 0,
        "classification": "appearance_fit_check_coupon_not_installed",
    },
]


def inertia_from_element(element: ET.Element) -> np.ndarray:
    return np.array(
        [
            [
                float(element.get("ixx", "0")),
                float(element.get("ixy", "0")),
                float(element.get("ixz", "0")),
            ],
            [
                float(element.get("ixy", "0")),
                float(element.get("iyy", "0")),
                float(element.get("iyz", "0")),
            ],
            [
                float(element.get("ixz", "0")),
                float(element.get("iyz", "0")),
                float(element.get("izz", "0")),
            ],
        ],
        dtype=float,
    )


def inertia_dict(matrix: np.ndarray) -> dict[str, float]:
    return {
        "ixx": float(matrix[0, 0]),
        "iyy": float(matrix[1, 1]),
        "izz": float(matrix[2, 2]),
        "ixy": float(matrix[0, 1]),
        "ixz": float(matrix[0, 2]),
        "iyz": float(matrix[1, 2]),
    }


def parallel_axis(mass: float, displacement: np.ndarray) -> np.ndarray:
    return mass * (
        float(displacement @ displacement) * np.eye(3)
        - np.outer(displacement, displacement)
    )


def aggregate(
    bodies: list[tuple[float, np.ndarray, np.ndarray]],
) -> tuple[float, np.ndarray, np.ndarray]:
    total_mass = sum(item[0] for item in bodies)
    if total_mass <= 0.0:
        raise ValueError("aggregate mass must be positive")
    center = sum(item[0] * item[1] for item in bodies) / total_mass
    inertia = sum(
        item[2] + parallel_axis(item[0], item[1] - center)
        for item in bodies
    )
    return total_mass, center, inertia


def read_urdf_inertials() -> dict[str, tuple[float, np.ndarray, np.ndarray]]:
    root = ET.parse(BASE_URDF).getroot()
    result: dict[str, tuple[float, np.ndarray, np.ndarray]] = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        mass_element = inertial.find("mass")
        inertia_element = inertial.find("inertia")
        origin = inertial.find("origin")
        if mass_element is None or inertia_element is None:
            raise ValueError(f"incomplete inertial on {link.get('name')}")
        com = np.array(
            [
                float(value)
                for value in (
                    origin.get("xyz", "0 0 0").split()
                    if origin is not None
                    else ("0", "0", "0")
                )
            ],
            dtype=float,
        )
        result[link.get("name", "")] = (
            float(mass_element.get("value", "0")),
            com,
            inertia_from_element(inertia_element),
        )
    return result


def fmt_vec(values: np.ndarray) -> list[float]:
    return [float(value) for value in values]


def main() -> None:
    baseline = read_urdf_inertials()
    part_rows: list[dict[str, object]] = []
    installed: dict[str, list[tuple[float, np.ndarray, np.ndarray]]] = {}
    part_payload: dict[str, dict[str, object]] = {}

    for definition in PARTS:
        step_path = PARTS_DIR / str(definition["file"])
        if not step_path.is_file():
            raise FileNotFoundError(step_path)
        shape = import_step(step_path)
        solids = list(shape.solids())
        if not solids or any(float(solid.volume) <= 0.0 for solid in solids):
            raise ValueError(f"non-solid or zero-volume STEP: {step_path}")
        volume_mm3 = float(shape.volume)
        mass_kg = volume_mm3 * PETG_DENSITY_KG_PER_MM3
        center_mm_vector = shape.center(CenterOf.MASS)
        center_mm = np.array(
            [center_mm_vector.X, center_mm_vector.Y, center_mm_vector.Z],
            dtype=float,
        )
        center_m = center_mm / 1000.0
        # build123d/OCC returns the unit-density centroidal tensor in mm^5.
        # kg/mm^3 * mm^5 * (m/mm)^2 -> kg*m^2.
        inertia_kg_m2 = (
            np.array(shape.matrix_of_inertia, dtype=float)
            * PETG_DENSITY_KG_PER_MM3
            * 1e-6
        )
        eigenvalues = np.linalg.eigvalsh(inertia_kg_m2)
        inertia_gate = bool(
            np.all(np.isfinite(inertia_kg_m2))
            and np.all(eigenvalues > 0.0)
        )
        installed_link = definition["installed_link"]
        quantity = int(definition["installed_quantity"])
        if installed_link and quantity:
            for _ in range(quantity):
                installed.setdefault(str(installed_link), []).append(
                    (mass_kg, center_m.copy(), inertia_kg_m2.copy())
                )

        row = {
            "part": definition["name"],
            "step_file": step_path.relative_to(ROOT).as_posix(),
            "classification": definition["classification"],
            "material_assumption": "PETG_nominal",
            "density_kg_m3": 1270.0,
            "volume_mm3": f"{volume_mm3:.9f}",
            "nominal_mass_kg": f"{mass_kg:.9f}",
            "com_x_m": f"{center_m[0]:.12g}",
            "com_y_m": f"{center_m[1]:.12g}",
            "com_z_m": f"{center_m[2]:.12g}",
            "ixx_kg_m2": f"{inertia_kg_m2[0, 0]:.12g}",
            "iyy_kg_m2": f"{inertia_kg_m2[1, 1]:.12g}",
            "izz_kg_m2": f"{inertia_kg_m2[2, 2]:.12g}",
            "ixy_kg_m2": f"{inertia_kg_m2[0, 1]:.12g}",
            "ixz_kg_m2": f"{inertia_kg_m2[0, 2]:.12g}",
            "iyz_kg_m2": f"{inertia_kg_m2[1, 2]:.12g}",
            "installed_link": installed_link or "",
            "installed_quantity": quantity,
            "positive_definite_inertia_gate": (
                "PASS" if inertia_gate else "FAIL"
            ),
            "hardware_override_required": "YES",
        }
        part_rows.append(row)
        part_payload[str(definition["name"])] = {
            "step_file": step_path.relative_to(ROOT).as_posix(),
            "classification": definition["classification"],
            "volume_mm3": volume_mm3,
            "nominal_mass_kg": mass_kg,
            "com_m": fmt_vec(center_m),
            "inertia_kg_m2_at_com": inertia_dict(inertia_kg_m2),
            "installed_link": installed_link,
            "installed_quantity": quantity,
            "inertia_eigenvalues_kg_m2": fmt_vec(eigenvalues),
            "gate": "PASS" if inertia_gate else "FAIL",
        }

    overlays: dict[str, dict[str, object]] = {}
    link_rows: list[dict[str, object]] = []
    for link_name, additions in sorted(installed.items()):
        if link_name not in baseline:
            raise KeyError(f"missing baseline inertial for {link_name}")
        base_mass, base_com, base_inertia = baseline[link_name]
        overlay_mass, overlay_com, overlay_inertia = aggregate(additions)
        combined_mass, combined_com, combined_inertia = aggregate(
            [(base_mass, base_com, base_inertia), *additions]
        )
        eigenvalues = np.linalg.eigvalsh(combined_inertia)
        gate = bool(
            combined_mass > 0.0
            and np.all(np.isfinite(combined_inertia))
            and np.all(eigenvalues > 0.0)
        )
        overlays[link_name] = {
            "baseline": {
                "mass_kg": base_mass,
                "com_m": fmt_vec(base_com),
                "inertia_kg_m2_at_com": inertia_dict(base_inertia),
            },
            "printed_overlay_only": {
                "mass_kg": overlay_mass,
                "com_m": fmt_vec(overlay_com),
                "inertia_kg_m2_at_com": inertia_dict(overlay_inertia),
            },
            "combined": {
                "mass_kg": combined_mass,
                "com_m": fmt_vec(combined_com),
                "inertia_kg_m2_at_com": inertia_dict(combined_inertia),
                "inertia_eigenvalues_kg_m2": fmt_vec(eigenvalues),
            },
            "gate": "PASS" if gate else "FAIL",
        }
        link_rows.append(
            {
                "link": link_name,
                "baseline_mass_kg": f"{base_mass:.12g}",
                "overlay_mass_kg": f"{overlay_mass:.12g}",
                "combined_mass_kg": f"{combined_mass:.12g}",
                "combined_com_x_m": f"{combined_com[0]:.12g}",
                "combined_com_y_m": f"{combined_com[1]:.12g}",
                "combined_com_z_m": f"{combined_com[2]:.12g}",
                "combined_ixx_kg_m2": f"{combined_inertia[0, 0]:.12g}",
                "combined_iyy_kg_m2": f"{combined_inertia[1, 1]:.12g}",
                "combined_izz_kg_m2": f"{combined_inertia[2, 2]:.12g}",
                "combined_ixy_kg_m2": f"{combined_inertia[0, 1]:.12g}",
                "combined_ixz_kg_m2": f"{combined_inertia[0, 2]:.12g}",
                "combined_iyz_kg_m2": f"{combined_inertia[1, 2]:.12g}",
                "positive_definite_inertia_gate": (
                    "PASS" if gate else "FAIL"
                ),
                "hardware_override_required": "YES",
            }
        )

    baseline_total = sum(item[0] for item in baseline.values())
    installed_overlay_mass = sum(
        float(value["nominal_mass_kg"])
        * int(value["installed_quantity"])
        for value in part_payload.values()
    )
    payload = {
        "schema": "zeroth01.round_v1.mass_properties.v1",
        "source_urdf": BASE_URDF.relative_to(ROOT).as_posix(),
        "material_assumption": {
            "name": "PETG_nominal",
            "density_kg_m3": 1270.0,
            "scope": (
                "CAD material volume at nominal density; excludes fasteners, "
                "adhesive, wiring and slicer/process variation"
            ),
        },
        "baseline_total_mass_kg": baseline_total,
        "installed_printed_overlay_mass_kg": installed_overlay_mass,
        "round_v1_nominal_total_mass_kg": (
            baseline_total + installed_overlay_mass
        ),
        "parts": part_payload,
        "link_overlays": overlays,
        "servo_mass_policy": (
            "Do not add 16 x 0.0745 kg: the baseline aggregate link inertials "
            "already represent the source assemblies. The vendor servo STEP is "
            "a placement/reference model, not an extra mass overlay."
        ),
        "hardware_override_required": True,
        "hardware_gate": (
            "FAIL_UNTIL_EACH_PRINTED_PART_AND_FINAL_ASSEMBLY_ARE_WEIGHED; "
            "replace nominal masses/COM/inertias before hardware deployment"
        ),
    }

    for rows, path in ((part_rows, PART_REPORT), (link_rows, LINK_REPORT)):
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader()
            writer.writerows(rows)
    JSON_OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    JSON_OUTPUT.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
