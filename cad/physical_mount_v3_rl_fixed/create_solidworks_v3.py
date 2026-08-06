"""Create the portable native SolidWorks v3 assembly from the v3 truth manifest.

The released v2 native carriers/cosmetics are copied, not remodeled.  Only the
nine new v3 STEP sources are translated.  All 18 servo occurrences reference
one dimension-controlled STS3250 SLDPRT and use the shaft transforms in the
same manifest consumed by the independent CAD evidence renderer.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import shutil
import time
from pathlib import Path

import pythoncom
import win32com.client


ROOT = Path(__file__).resolve().parents[2]
V1_SCRIPT = ROOT / "cad" / "physical_mount_v1" / "create_solidworks_physical_mount_v1.py"
ROUND_SCRIPT = ROOT / "scripts" / "create_solidworks_round_v1_review.py"
MANIFEST = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v3_rl_fixed"
    / "ZEROTH01_V3_RL_FIXED_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)
V2_PORTABLE = (
    ROOT / "generated" / "solidworks" / "physical_mount_v2_minimal" / "portable_flat"
)
SW_ROOT = ROOT / "generated" / "solidworks" / "physical_mount_v3_rl_fixed"
PORTABLE = SW_ROOT / "portable_flat"
NORMAL_ASM = PORTABLE / "OPEN_FIRST_ZEROTH01_V3_RL_FIXED_CONNECTED_WHITE_18_BLUE_STS3250.SLDASM"
TOP_ASM = PORTABLE / "OPTIONAL_XRAY_ZEROTH01_V3_RL_FIXED_18_BLUE_STS3250.SLDASM"
REPORT_ROOT = ROOT / "reports" / "physical_mount_v3_rl_fixed"
GATE_REPORT = REPORT_ROOT / "solidworks_gate.json"
COMPONENT_CSV = REPORT_ROOT / "solidworks_component_manifest.csv"
TRACE_LOG = REPORT_ROOT / "solidworks_trace.log"
SNAPSHOT_ROOT = ROOT / "snapshots" / "solidworks" / "physical_mount_v3_rl_fixed"

SW_DOC_PART = 1
SW_DOC_ASSEMBLY = 2
SW_SAVE_CURRENT = 0
SW_SAVE_SILENT = 1
WHITE = (0.969, 0.973, 0.980)
BLUE = (0.086, 0.467, 1.0)


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v1 = _load(V1_SCRIPT, "v3_solidworks_v1")
round_sw = _load(ROUND_SCRIPT, "v3_solidworks_step_import")


def trace(message: str) -> None:
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with TRACE_LOG.open("a", encoding="utf-8") as stream:
        stream.write(f"{stamp} {message}\n")
    print(message, flush=True)


def get_sw(startup_timeout: float):
    """Attach to the already-open same-integrity SolidWorks instance first."""

    pythoncom.CoInitialize()
    try:
        sw = win32com.client.Dispatch("SldWorks.Application")
        sw.Visible = True
        sw.CommandInProgress = False
        trace("SolidWorks attached through existing COM local server")
        return sw
    except Exception as exc:
        trace(f"existing SolidWorks COM attach failed: {exc!r}; falling back to ROT/start")
        return v1.get_or_start_sw(startup_timeout)


def safe(value: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in value)


def color(value: str) -> tuple[float, float, float]:
    token = value.lstrip("#")
    return tuple(int(token[index : index + 2], 16) / 255.0 for index in (0, 2, 4))


def data() -> dict[str, object]:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def v3_part_path(source: str) -> Path:
    return PORTABLE / f"ZEROTH01_V3_{safe(Path(source).stem).upper()}.SLDPRT"


NEW_ROLES = {
    "dimension_controlled_sts3250",
    "new_ankle_roll_parent_carrier",
    "new_ankle_roll_child_horn_adapter",
    "replaceable_9mm_perimeter_rib_sole",
    "reversible_slotted_inboard_adapter",
    "purchased_interaction_head_module",
    "reversible_purchased_head_torso_adapter",
    "purchased_head_face_reference",
    "purchased_head_sensor_window_reference",
    "purchased_head_screen_ui_reference",
}


def unique_new_sources() -> dict[str, str]:
    result: dict[str, str] = {}
    for row in data()["components"]:
        if row["role"] in NEW_ROLES:
            source = str(row["source"])
            result[Path(source).stem] = source
    return result


def v2_native_name(row: dict[str, object]) -> str:
    component_id = str(row["component_id"])
    source_stem = Path(str(row["source"])).stem
    role = str(row["role"])
    if role == "source_load_bearing_carrier":
        if source_stem.endswith("WRIST_TRIMMED"):
            return f"ZEROTH01_V2_MINIMAL_{safe(source_stem).upper()}.SLDPRT"
        return f"ZEROTH01_PHYSICAL_MOUNT_V1_{safe(source_stem).upper()}_CARRIER.SLDPRT"
    if role == "fixed_q_hand":
        side = "LEFT" if component_id.startswith("LEFT") else "RIGHT"
        return f"ZEROTH01_V2_MINIMAL_{side}_Q_HAND.SLDPRT"
    return f"ZEROTH01_V2_MINIMAL_{safe(source_stem).upper()}.SLDPRT"


def native_part_for(row: dict[str, object]) -> Path:
    if str(row["role"]) in NEW_ROLES:
        return v3_part_path(str(row["source"]))
    return PORTABLE / v2_native_name(row)


def prepare_portable() -> dict[str, object]:
    PORTABLE.mkdir(parents=True, exist_ok=True)
    copied: list[str] = []
    seen: set[str] = set()
    for row in data()["components"]:
        if str(row["role"]) in NEW_ROLES:
            continue
        name = v2_native_name(row)
        if name in seen:
            continue
        seen.add(name)
        source = V2_PORTABLE / name
        target = PORTABLE / name
        if not source.is_file():
            raise FileNotFoundError(source)
        if not target.is_file() or target.stat().st_mtime < source.stat().st_mtime:
            shutil.copy2(source, target)
        copied.append(name)
    shutil.copy2(MANIFEST, PORTABLE / MANIFEST.name)
    return {"copied_or_reused_v2_native_parts": len(copied), "status": "PASS"}


def close_task_documents(sw) -> None:
    output_root = str(SW_ROOT.resolve()).lower()
    for raw in list(v1.base.call(sw, "GetDocuments", []) or []):
        try:
            model = v1.base.as_model_doc(raw)
            path = str(v1.base.call(model, "GetPathName", "")).lower()
            if path and path.startswith(output_root):
                sw.CloseDoc(str(v1.base.call(model, "GetTitle", "")))
        except Exception:
            continue


def import_new(key: str, force: bool, startup_timeout: float) -> dict[str, object]:
    sources = unique_new_sources()
    if key not in sources:
        raise KeyError(f"unknown new part {key}; choose {sorted(sources)}")
    source = ROOT / sources[key]
    target = v3_part_path(sources[key])
    prepare_portable()
    sw = v1.typed_sldworks(get_sw(startup_timeout))
    close_task_documents(sw)
    previous = int(
        sw.GetUserPreferenceIntegerValue(round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING)
    )
    sw.SetUserPreferenceIntegerValue(
        round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING,
        round_sw.SW_IMPORT_NEUTRAL_AS_MULTIBODY_PART,
    )
    try:
        row = round_sw.import_step_part(sw, source, target, force=force)
    finally:
        sw.SetUserPreferenceIntegerValue(
            round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING, previous
        )
    row.update({"key": key, "status": "PASS"})
    print(json.dumps(row, ensure_ascii=False, indent=2), flush=True)
    return row


def transform(row: dict[str, object]):
    matrix = row["transform_local_mm_to_world_mm"]
    rotation = tuple(tuple(float(matrix[i][j]) for j in range(3)) for i in range(3))
    translation = tuple(float(matrix[i][3]) / 1000.0 for i in range(3))
    return rotation, translation


def save_view(model, path: Path, isometric: bool) -> bool:
    if isometric:
        return v1.save_view(model, 7, path, view_name="*Isometric")
    return v1.save_view(model, 4, path, view_name="*Right", rotate_degrees=-90.0)


def write_csv(rows: list[dict[str, object]]) -> None:
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    COMPONENT_CSV.parent.mkdir(parents=True, exist_ok=True)
    with COMPONENT_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def assemble(startup_timeout: float) -> dict[str, object]:
    manifest = data()
    prepare_portable()
    for key, source in unique_new_sources().items():
        target = v3_part_path(source)
        if not target.is_file() or target.stat().st_size < 1024:
            raise FileNotFoundError(f"import-new {key} first: {target}")
    for row in manifest["components"]:
        if not native_part_for(row).is_file():
            raise FileNotFoundError(native_part_for(row))

    pythoncom.CoInitialize()
    sw = v1.typed_sldworks(get_sw(startup_timeout))
    close_task_documents(sw)
    raw = v1.first(sw.NewDocument(str(v1.base.ASM_TEMPLATE), 0, 0, 0))
    if raw is None:
        raise RuntimeError("SolidWorks NewDocument failed")
    model = v1.base.as_model_doc(raw)
    assembly = v1.base.as_assembly_doc(raw)
    components: dict[str, object] = {}
    rows: list[dict[str, object]] = []

    for index, row in enumerate(manifest["components"], start=1):
        component_id = str(row["component_id"])
        part = native_part_for(row)
        trace(f"assembly component {index}/{manifest['component_count']}: {component_id}")
        component, error = v1.add_component(
            sw, model, assembly, part, SW_DOC_PART, component_id, transform(row)
        )
        v1.set_material(component, color(str(row["color_hex"])), 0.0)
        components[component_id] = component
        rows.append(
            {
                "component_id": component_id,
                "role": row["role"],
                "owner_link": row["owner_link"],
                "native_part": part.name,
                "transform_error": error,
                "status": "PASS" if error < 1.0e-8 else "FAIL",
            }
        )

    payload_ids = {
        str(row["component_id"])
        for row in manifest["components"]
        if row["role"] == "internal_payload_controlled_envelope"
    }
    for component_id in payload_ids:
        round_sw.set_component_visibility(components[component_id], False)
    round_sw.try_shaded(model)
    v1.base.refresh_assembly_display(model)
    normal_code = int(model.SaveAs3(str(NORMAL_ASM), SW_SAVE_CURRENT, SW_SAVE_SILENT))
    normal_views = {
        "front": save_view(model, SNAPSHOT_ROOT / "v3_solidworks_normal_front.png", False),
        "iso": save_view(model, SNAPSHOT_ROOT / "v3_solidworks_normal_iso.png", True),
    }

    for component_id in payload_ids:
        round_sw.set_component_visibility(components[component_id], True)
    for row in manifest["components"]:
        component_id = str(row["component_id"])
        role = str(row["role"])
        if role == "dimension_controlled_sts3250":
            v1.set_material(components[component_id], BLUE, 0.0)
        elif role == "internal_payload_controlled_envelope":
            v1.set_material(components[component_id], color(str(row["color_hex"])), 0.0)
        elif role in {
            "screen_ui_reference",
            "visible_camera_and_tof_windows",
            "purchased_head_face_reference",
            "purchased_head_sensor_window_reference",
            "purchased_head_screen_ui_reference",
        }:
            v1.set_material(components[component_id], color(str(row["color_hex"])), 0.15)
        else:
            v1.set_material(components[component_id], WHITE, 0.70)
    v1.base.refresh_assembly_display(model)
    xray_views = {
        "front": save_view(model, SNAPSHOT_ROOT / "v3_solidworks_xray_front.png", False),
        "iso": save_view(model, SNAPSHOT_ROOT / "v3_solidworks_xray_iso.png", True),
    }
    xray_code = int(model.SaveAs3(str(TOP_ASM), SW_SAVE_CURRENT, SW_SAVE_SILENT))
    write_csv(rows)

    component_paths = [str(v1.base.call(component, "GetPathName", "")) for component in components.values()]
    portable_root = str(PORTABLE.resolve()).lower()
    portable_ok = all(
        path and path.lower().startswith(portable_root) and Path(path).is_file()
        for path in component_paths
    )
    actual_count = int(v1.base.call(assembly, "GetComponentCount", len(components), True))
    gate = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.solidworks_gate.v1",
        "solidworks_revision": str(v1.base.call(sw, "RevisionNumber", "")),
        "open_first_xray_assembly": str(TOP_ASM),
        "normal_assembly": str(NORMAL_ASM),
        "manifest_component_count": manifest["component_count"],
        "assembly_component_count": actual_count,
        "separate_blue_sts3250_count": sum(
            row["role"] == "dimension_controlled_sts3250" for row in manifest["components"]
        ),
        "old_claw_component_count": manifest["old_claw_count"],
        "q_hand_component_count": sum(row["role"] == "fixed_q_hand" for row in manifest["components"]),
        "portable_dependency_gate": "PASS" if portable_ok else "FAIL",
        "component_transform_gate": "PASS" if all(row["status"] == "PASS" for row in rows) else "FAIL",
        "normal_save_gate": "PASS" if NORMAL_ASM.is_file() and normal_code >= 0 else "FAIL",
        "xray_save_gate": "PASS" if TOP_ASM.is_file() and xray_code >= 0 else "FAIL",
        "normal_view_gate": "PASS" if all(normal_views.values()) else "FAIL",
        "xray_view_gate": "PASS" if all(xray_views.values()) else "FAIL",
    }
    gate["overall"] = "PASS" if all(
        gate[key] == "PASS"
        for key in (
            "portable_dependency_gate",
            "component_transform_gate",
            "normal_save_gate",
            "xray_save_gate",
            "normal_view_gate",
            "xray_view_gate",
        )
    ) and actual_count == manifest["component_count"] else "FAIL"
    GATE_REPORT.write_text(json.dumps(gate, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(gate, ensure_ascii=False, indent=2), flush=True)
    return gate


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list-new")
    prep = sub.add_parser("prepare")
    imp = sub.add_parser("import-new")
    imp.add_argument("key")
    imp.add_argument("--force", action="store_true")
    imp.add_argument("--startup-timeout", type=float, default=45.0)
    asm = sub.add_parser("assemble")
    asm.add_argument("--startup-timeout", type=float, default=45.0)
    args = parser.parse_args()
    if args.command == "list-new":
        print("\n".join(sorted(unique_new_sources())))
    elif args.command == "prepare":
        print(json.dumps(prepare_portable(), indent=2))
    elif args.command == "import-new":
        import_new(args.key, args.force, args.startup_timeout)
    elif args.command == "assemble":
        assemble(args.startup_timeout)


if __name__ == "__main__":
    main()
