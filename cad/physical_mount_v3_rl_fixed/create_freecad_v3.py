"""Build self-contained FreeCAD v3 assemblies from the full truth manifest."""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import FreeCAD as App
import Part


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v3_rl_fixed"
    / "ZEROTH01_V3_RL_FIXED_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)
OUT = ROOT / "generated" / "freecad" / "physical_mount_v3_rl_fixed"
NORMAL = OUT / "OPEN_FIRST_ZEROTH01_V3_RL_FIXED_FULL_CONNECTED_WHITE_18_BLUE_STS3250.FCStd"
XRAY = OUT / "OPTIONAL_XRAY_ZEROTH01_V3_RL_FIXED_18_BLUE_STS3250.FCStd"
REPORT = ROOT / "reports" / "physical_mount_v3_rl_fixed" / "freecad_gate.json"


def safe(value: str) -> str:
    token = re.sub(r"[^A-Za-z0-9_]", "_", value)
    return token if not token[:1].isdigit() else f"C_{token}"


def rgb(value: str) -> tuple[float, float, float]:
    token = value.lstrip("#")
    return tuple(int(token[index : index + 2], 16) / 255.0 for index in (0, 2, 4))


def placement(values) -> App.Placement:
    matrix = App.Matrix()
    for row in range(4):
        for column in range(4):
            setattr(matrix, f"A{row + 1}{column + 1}", float(values[row][column]))
    return App.Placement(matrix)


def load_shape(path: Path):
    shape = Part.Shape()
    shape.read(str(path))
    if shape.isNull():
        raise RuntimeError(f"null shape: {path}")
    return shape


SOLID_XRAY_ROLES = {
    "dimension_controlled_sts3250",
    "source_load_bearing_carrier",
    "fixed_q_hand",
    "new_ankle_roll_parent_carrier",
    "new_ankle_roll_child_horn_adapter",
    "replaceable_7mm_perimeter_rib_sole",
    "reversible_slotted_inboard_adapter",
    "purchased_head_face_reference",
    "purchased_head_sensor_window_reference",
    "purchased_head_screen_ui_reference",
}


def build(target: Path, xray: bool) -> dict[str, object]:
    payload = json.loads(MANIFEST.read_text(encoding="utf-8"))
    target.parent.mkdir(parents=True, exist_ok=True)
    doc = App.newDocument(safe(target.stem))
    root_group = doc.addObject("App::Part", "ZEROTH01_V3_RL_FIXED_FULL_ASSEMBLY")
    root_group.Label = "ZEROTH01 v3 RL-FIXED 18DoF full assembly"
    cache = {}
    rows = []
    for index, row in enumerate(payload["components"], start=1):
        component_id = str(row["component_id"])
        source = ROOT / str(row["source"])
        if source not in cache:
            cache[source] = load_shape(source)
        obj = doc.addObject("Part::Feature", safe(component_id))
        obj.Label = component_id
        obj.Shape = cache[source].copy()
        obj.Placement = placement(row["transform_local_mm_to_world_mm"])
        for property_name, label, value in (
            ("Role", "Role", str(row["role"])),
            ("OwnerLink", "Owner link", str(row["owner_link"])),
            ("SourcePath", "Source path", str(row["source"])),
            ("ColorHex", "Color", str(row["color_hex"])),
        ):
            obj.addProperty("App::PropertyString", property_name, "v3 metadata", label)
            setattr(obj, property_name, value)
        try:
            obj.ViewObject.ShapeColor = rgb(str(row["color_hex"]))
            obj.ViewObject.LineColor = (0.15, 0.18, 0.22)
            obj.ViewObject.DisplayMode = "Flat Lines"
            obj.ViewObject.LineWidth = 1.5
            obj.ViewObject.Transparency = (
                0 if not xray or str(row["role"]) in SOLID_XRAY_ROLES else 72
            )
        except Exception:
            pass
        root_group.addObject(obj)
        rows.append(
            {
                "index": index,
                "component_id": component_id,
                "role": row["role"],
                "source": str(row["source"]),
                "shape_valid": bool(obj.Shape.isValid()),
                "solid_count": len(obj.Shape.Solids),
            }
        )
        print(f"FreeCAD {target.name}: {index}/{payload['component_count']} {component_id}", flush=True)
    doc.addObject("App::FeaturePython", "ASSEMBLY_METADATA")
    metadata = doc.getObject("ASSEMBLY_METADATA")
    metadata.addProperty("App::PropertyInteger", "ComponentCount")
    metadata.ComponentCount = int(payload["component_count"])
    metadata.addProperty("App::PropertyInteger", "BlueSTS3250Count")
    metadata.BlueSTS3250Count = int(payload["blue_sts3250_count"])
    metadata.addProperty("App::PropertyString", "MassNominalKg")
    metadata.MassNominalKg = "2.969171828"
    metadata.addProperty("App::PropertyString", "HeadModule")
    metadata.HeadModule = "M5Stack CoreS3 K128 purchased main unit"
    doc.recompute()
    doc.saveAs(str(target))
    App.closeDocument(doc.Name)
    return {
        "path": target.relative_to(ROOT).as_posix(),
        "bytes": target.stat().st_size if target.is_file() else 0,
        "component_count": len(rows),
        "valid_component_count": sum(item["shape_valid"] for item in rows),
        "all_sources_embedded": True,
        "xray": xray,
        "gate": "PASS" if target.is_file() and target.stat().st_size > 4096 and all(item["shape_valid"] for item in rows) else "FAIL",
    }


def main() -> int:
    results = [build(NORMAL, False), build(XRAY, True)]
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    report = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.freecad_gate.v1",
        "freecad_version": App.Version(),
        "manifest": MANIFEST.relative_to(ROOT).as_posix(),
        "manifest_component_count": manifest["component_count"],
        "assemblies": results,
        "overall": "PASS" if all(item["gate"] == "PASS" and item["component_count"] == manifest["component_count"] for item in results) else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    return 0 if report["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
