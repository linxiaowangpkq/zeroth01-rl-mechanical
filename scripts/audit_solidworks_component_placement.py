from __future__ import annotations

import csv
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pythoncom
import win32com.client as win32

from audit_mesh_frames import binary_stl_bounds, neutral_transforms, transformed_bounds


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_reference.urdf"
MANIFEST = ROOT / "reports" / "solidworks_component_manifest.csv"
REPORT_CSV = ROOT / "reports" / "solidworks_component_placement_audit.csv"
REPORT_JSON = ROOT / "reports" / "solidworks_component_placement_audit.json"


def center(bounds: list[float]) -> list[float]:
    return [(bounds[index] + bounds[index + 3]) / 2.0 for index in range(3)]


def norm(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values))


def value_or_call(obj, name: str, *args):
    value = getattr(obj, name)
    return value(*args) if callable(value) else value


def main() -> None:
    pythoncom.CoInitialize()
    sw = win32.GetActiveObject("SldWorks.Application")
    model = sw.ActiveDoc
    if model is None or int(value_or_call(model, "GetType")) != 2:
        raise RuntimeError("the active SolidWorks document is not an assembly")
    components = {
        str(component.Name2): component
        for component in list(value_or_call(model, "GetComponents", False) or [])
    }
    with MANIFEST.open(encoding="utf-8-sig", newline="") as stream:
        link_to_component = {
            row["link"]: row["component"] for row in csv.DictReader(stream)
        }

    root = ET.parse(URDF).getroot()
    transforms = neutral_transforms(root)
    rows: list[dict[str, object]] = []
    for link in root.findall("link"):
        name = link.get("name", "")
        mesh = link.find("./visual/geometry/mesh")
        if mesh is None:
            continue
        component_name = link_to_component[name]
        component = components.get(component_name)
        if component is None:
            raise RuntimeError(f"missing SolidWorks component: {component_name}")
        actual_bounds = [
            float(value)
            for value in list(value_or_call(component, "GetBox", False, False))
        ]
        actual_center = center(actual_bounds)
        mesh_path = (URDF.parent / mesh.get("filename", "")).resolve()
        mesh_minimum, mesh_maximum, _ = binary_stl_bounds(mesh_path)
        expected_minimum, expected_maximum = transformed_bounds(
            mesh_minimum, mesh_maximum, transforms[name]
        )
        expected_center = center(expected_minimum + expected_maximum)
        rotation, translation = transforms[name]
        transposed_rotation = [
            [rotation[column][row] for column in range(3)] for row in range(3)
        ]
        transposed_minimum, transposed_maximum = transformed_bounds(
            mesh_minimum,
            mesh_maximum,
            (transposed_rotation, translation),
        )
        delta = [
            actual_center[index] - expected_center[index] for index in range(3)
        ]
        expected_bounds = expected_minimum + expected_maximum
        transposed_bounds = transposed_minimum + transposed_maximum
        direct_bounds_error = max(
            abs(actual_bounds[index] - expected_bounds[index])
            for index in range(6)
        )
        transposed_bounds_error = max(
            abs(actual_bounds[index] - transposed_bounds[index])
            for index in range(6)
        )
        transform_data = [
            float(value) for value in list(component.Transform2.ArrayData)
        ]
        rows.append(
            {
                "link": name,
                "component": component_name,
                "actual_bbox_center_m": " ".join(
                    f"{value:.9f}" for value in actual_center
                ),
                "expected_bbox_center_m": " ".join(
                    f"{value:.9f}" for value in expected_center
                ),
                "delta_m": " ".join(f"{value:.9f}" for value in delta),
                "center_error_m": norm(delta),
                "direct_rotation_bbox_error_m": direct_bounds_error,
                "transposed_rotation_bbox_error_m": transposed_bounds_error,
                "solidworks_applied_rotation": (
                    "URDF_R"
                    if direct_bounds_error <= transposed_bounds_error
                    else "URDF_R_TRANSPOSE"
                ),
                "transform_translation_m": " ".join(
                    f"{value:.9f}" for value in transform_data[9:12]
                ),
            }
        )

    with REPORT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    payload = {
        "assembly": str(value_or_call(model, "GetPathName")),
        "component_count": len(rows),
        "maximum_bbox_center_error_m": max(
            float(row["center_error_m"]) for row in rows
        ),
        "maximum_direct_rotation_bbox_error_m": max(
            float(row["direct_rotation_bbox_error_m"]) for row in rows
        ),
        "maximum_transposed_rotation_bbox_error_m": max(
            float(row["transposed_rotation_bbox_error_m"]) for row in rows
        ),
        "applied_rotation_counts": {
            label: sum(row["solidworks_applied_rotation"] == label for row in rows)
            for label in ("URDF_R", "URDF_R_TRANSPOSE")
        },
        "rows_over_1mm": [
            row for row in rows if float(row["center_error_m"]) > 0.001
        ],
        "report_csv": str(REPORT_CSV),
    }
    REPORT_JSON.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(payload, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
