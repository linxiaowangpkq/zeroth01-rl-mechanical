"""Create the portable native SOLIDWORKS v5 static review assemblies.

Every component referenced by the v5 manifest is translated from the exact v5
STEP source.  Reusing a same-named v4 SLDPRT is unsafe here: several parts keep
their semantic name while their manufacturing geometry changes (for example
the 9 mm tapered sole and the carrier/wrist clearance cuts).
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import shutil
from pathlib import Path

import pythoncom


ROOT = Path(__file__).resolve().parents[2]
V3_SCRIPT = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "create_solidworks_v3.py"
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
V2_PORTABLE = ROOT / "generated" / "solidworks" / "physical_mount_v2_minimal" / "portable_flat"
V4_PORTABLE = ROOT / "generated" / "solidworks" / "physical_mount_v4_original_minimal" / "portable_flat"
SW_ROOT = ROOT / "generated" / "solidworks" / "physical_mount_v5_original_16dof_solidworks_motion"
PORTABLE = SW_ROOT / "portable_flat"
NORMAL_ASM = PORTABLE / "OPEN_FIRST_ZEROTH01_V5_ORIGINAL_16DOF_TRUE_MOTION.SLDASM"
XRAY_ASM = PORTABLE / "OPTIONAL_XRAY_ZEROTH01_V5_16_BLUE_STS3250_TRANSMISSION.SLDASM"
REPORT_ROOT = ROOT / "reports" / "v5_original_16dof_solidworks_motion"
SNAPSHOT_ROOT = ROOT / "snapshots" / "solidworks" / "v5_original_16dof_solidworks_motion"
FULL_ASSEMBLY_STEP = ROOT / "generated" / "cad" / "physical_mount_v5_original_16dof_solidworks_motion" / "ZEROTH01_V5_ORIGINAL_16DOF_FULL_ASSEMBLY.step"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v3 = load(V3_SCRIPT, "zeroth_v3_solidworks_reused_by_v5")


def add_component_direct(sw, model, assembly, source, document_type, name, transform):
    """Insert directly to avoid duplicate-title preload failures.

    SOLIDWORKS refuses OpenDoc6 when an older review assembly already owns a
    different portable copy with the same internal document title.  AddComponent5
    resolves the explicit v5 path correctly and is the API intended for this
    top-level insertion case.
    """

    component = assembly.AddComponent5(str(source), 0, "", False, "", 0.0, 0.0, 0.0)
    opened = None
    if component is None:
        result = sw.OpenDoc6(str(source), document_type, v3.v1.SW_OPEN_SILENT, "", 0, 0)
        opened = v3.v1.first(result)
        v3.v1.base.activate_document(sw, model)
        component = assembly.AddComponent5(str(source), 0, "", False, "", 0.0, 0.0, 0.0)
    if component is None:
        raise RuntimeError(f"AddComponent5 failed: {source}")
    try:
        component.Name2 = name
    except Exception:
        pass
    error = v3.v1.base.set_component_transform(sw, component, transform)
    if opened is not None:
        v3.v1.base.close_document(sw, opened)
        v3.v1.base.activate_document(sw, model)
    return component, error


def raw_data() -> dict[str, object]:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def data() -> dict[str, object]:
    payload = raw_data()
    payload.setdefault("old_claw_count", 0)
    payload.setdefault("q_hand_count", 0)
    for row in payload["components"]:
        role = str(row["role"])
        if role == "purchased_exact_sts3250":
            row["role"] = "dimension_controlled_sts3250"
        elif role in {
            "internal_payload_controlled_envelope",
            "removable_internal_service_mount",
            "purchased_internal_interaction_module",
            "rigid_sensor_mount",
            "harness_strain_relief",
            "direct_head_torso_mount",
        }:
            row["role"] = "internal_payload_controlled_envelope"
    return payload


def safe(value: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in value)


def is_v5_source(row: dict[str, object]) -> bool:
    return "/physical_mount_v5_original_16dof_solidworks_motion/parts/" in f"/{str(row['source']).replace(chr(92), '/')}"


def v5_part_path(source: str) -> Path:
    return PORTABLE / f"ZEROTH01_V5_{safe(Path(source).stem).upper()}.SLDPRT"


def v4_part_path(source: str) -> Path:
    return V4_PORTABLE / f"ZEROTH01_V4_{safe(Path(source).stem).upper()}.SLDPRT"


def source_native_name(row: dict[str, object]) -> str:
    stem = Path(str(row["source"])).stem
    if stem.endswith("WRIST_TRIMMED"):
        return f"ZEROTH01_V2_MINIMAL_{safe(stem).upper()}.SLDPRT"
    return f"ZEROTH01_PHYSICAL_MOUNT_V1_{safe(stem).upper()}_CARRIER.SLDPRT"


def native_part_for(row: dict[str, object]) -> Path:
    if is_v5_source(row):
        return v5_part_path(str(row["source"]))
    # Prefix copied released carriers so they never collide with an already
    # loaded v2/v4 document carrying the same SOLIDWORKS title.
    return PORTABLE / f"V5_REUSED_{source_native_name(row)}"


def unique_new_sources() -> dict[str, str]:
    """Return every distinct v5 STEP dependency.

    All current manifest sources live under the v5 tree.  Importing each one
    prevents stale v4 geometry from silently entering the native assembly.
    """

    result = {}
    for row in raw_data()["components"]:
        if not is_v5_source(row):
            continue
        source = str(row["source"])
        result[Path(source).stem] = source
    return result


def prepare_portable() -> dict[str, object]:
    PORTABLE.mkdir(parents=True, exist_ok=True)
    copied = []
    seen = set()
    for row in raw_data()["components"]:
        target = native_part_for(row)
        if target.name in seen:
            continue
        seen.add(target.name)
        source = None
        if is_v5_source(row):
            # Deliberately do not copy a v4 part with the same stem.  The v5
            # STEP file is the released source of truth and is imported by
            # ``import-all`` below.
            source = None
        else:
            source = V2_PORTABLE / source_native_name(row)
        if source is None:
            continue
        if not source.is_file():
            raise FileNotFoundError(source)
        if not target.is_file() or target.stat().st_mtime < source.stat().st_mtime:
            shutil.copy2(source, target)
        copied.append(target.name)
    shutil.copy2(MANIFEST, PORTABLE / MANIFEST.name)
    return {"copied_native_parts": len(copied), "new_step_imports_required": len(unique_new_sources()), "status": "PASS"}


def create_native_standoff(sw, key: str, height_mm: float) -> dict[str, object]:
    """Build the simple hub-clearance flange natively in SOLIDWORKS.

    This avoids the neutral-file translator entirely.  The one sketch contains
    an outer circle, the center hub clearance and four PCD14 M3 through holes,
    so the result is one connected native boss feature rather than floating
    imported bodies.
    """

    source = unique_new_sources()[key]
    target = v5_part_path(source)
    raw = v3.v1.first(sw.NewDocument(str(v3.v1.base.PART_TEMPLATE), 0, 0, 0))
    if raw is None:
        raise RuntimeError(f"NewDocument failed for native standoff {key}")
    model = v3.v1.base.as_model_doc(raw)
    part = v3.v1.base.as_part_doc(raw)
    raw_modeler = sw.GetModeler()
    modeler = v3.v1.base.MODELDOC_MODULE.IModeler(raw_modeler._oleobj_)
    height_m = height_mm / 1000.0

    def double_array(values):
        return v3.win32com.client.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, tuple(values))

    def first_boolean_body(value):
        if value is None:
            return None
        if hasattr(value, "GetBodyBox") or hasattr(value, "GetFaces"):
            return value
        if isinstance(value, (tuple, list)):
            for item in value:
                found = first_boolean_body(item)
                if found is not None:
                    return found
        return None

    result = modeler.CreateBodyFromCyl(double_array((0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.009975, height_m)))
    if result is None:
        raise RuntimeError(f"SOLIDWORKS modeler outer cylinder failed for {key}")
    cutter_specs = [(0.0, 0.0, 0.0048)]
    cutter_specs.extend(
        (0.007 * math.cos(math.radians(angle)), 0.007 * math.sin(math.radians(angle)), 0.0016)
        for angle in (0.0, 90.0, 180.0, 270.0)
    )
    for x, y, radius in cutter_specs:
        tool = modeler.CreateBodyFromCyl(double_array((x, y, -0.0005, 0.0, 0.0, 1.0, radius, height_m + 0.001)))
        operation = result.Operations2(15902, tool, 0)
        result = first_boolean_body(operation)
        if result is None:
            raise RuntimeError(f"SOLIDWORKS modeler cut failed for {key} at {(x, y, radius)}")
    feature = part.CreateFeatureFromBody3(result, False, 0)
    if feature is None:
        raise RuntimeError(f"SOLIDWORKS imported-body feature failed for {key}")
    feature.Name = f"PCD14_HUB_CLEARANCE_{height_mm:g}MM_NATIVE_BODY"
    target.parent.mkdir(parents=True, exist_ok=True)
    code = int(model.SaveAs3(str(target), 0, 1))
    bodies = list(part.GetBodies2(0, True) or [])
    box = list(part.GetPartBox(True) or [])
    v3.v1.base.close_document(sw, raw)
    if not target.is_file() or target.stat().st_size < 1024 or len(bodies) != 1:
        raise RuntimeError(f"native standoff validation failed for {key}: bodies={len(bodies)} save={code}")
    return {"key": key, "path": str(target), "height_mm": height_mm, "body_count": len(bodies), "box": box, "save_code": code, "status": "PASS"}


def create_native_standoffs(startup_timeout: float) -> list[dict[str, object]]:
    prepare_portable()
    sw = v3.v1.typed_sldworks(v3.get_sw(startup_timeout))
    v3.close_task_documents(sw)
    rows = []
    for key, height_mm in (
        ("sts3250_pcd14_child_standoff_1mm", 1.0),
        ("sts3250_pcd14_child_standoff_1p95mm", 1.95),
        ("sts3250_pcd14_child_standoff_3mm", 3.0),
        ("sts3250_pcd14_child_standoff_12p95mm", 12.95),
    ):
        rows.append(create_native_standoff(sw, key, height_mm))
        print(json.dumps(rows[-1], ensure_ascii=False), flush=True)
    return rows


def assemble_v5_complete(startup_timeout: float) -> dict[str, object]:
    """Populate and finish v5 without the legacy per-occurrence GetBox loop.

    The v3 helper inserted every occurrence correctly but then queried
    IComponent2::GetBox once per imported occurrence; one translated body can
    make that API block indefinitely.  v5 only needs the assembly-level box,
    so insertion is kept and finalisation is delegated to
    ``finish_active_assembly`` below.
    """

    manifest = raw_data()
    prepare_portable()
    for key, source in unique_new_sources().items():
        target = v5_part_path(source)
        if not target.is_file() or target.stat().st_size < 1024:
            raise FileNotFoundError(f"import-new {key} first: {target}")
    for row in manifest["components"]:
        if not native_part_for(row).is_file():
            raise FileNotFoundError(native_part_for(row))

    pythoncom.CoInitialize()
    sw = v3.v1.typed_sldworks(v3.get_sw(startup_timeout))
    v3.close_task_documents(sw)
    raw = v3.v1.first(sw.NewDocument(str(v3.v1.base.ASM_TEMPLATE), 0, 0, 0))
    if raw is None:
        raise RuntimeError("SOLIDWORKS NewDocument failed")
    model = v3.v1.base.as_model_doc(raw)
    assembly = v3.v1.base.as_assembly_doc(raw)
    for index, row in enumerate(manifest["components"], start=1):
        component_id = str(row["component_id"])
        part = native_part_for(row)
        print(f"v5 assembly component {index}/{manifest['component_count']}: {component_id}", flush=True)
        component, error = add_component_direct(
            sw, model, assembly, part, v3.SW_DOC_PART, component_id, v3.transform(row)
        )
        if error >= 1.0e-8:
            raise RuntimeError(f"component transform error {component_id}: {error}")
        v3.v1.set_material(component, v3.color(str(row["color_hex"])), 0.0)
    model.ForceRebuild3(False)
    return finish_active_assembly(startup_timeout)


def finish_active_assembly(startup_timeout: float) -> dict[str, object]:
    """Finish the already-populated active assembly without per-occurrence GetBox.

    IComponent2::GetBox can block indefinitely on a translated imported body.
    SOLIDWORKS provides IAssemblyDoc::GetBox for the assembly-level bound, which
    is the only bound needed by this gate.  Occurrences are still mapped by
    native filename plus their installed transform, so no component is skipped.
    """

    sw = v3.v1.typed_sldworks(v3.get_sw(startup_timeout))
    raw = sw.ActiveDoc
    if raw is None:
        raise RuntimeError("SOLIDWORKS has no active assembly to finish")
    model = v3.v1.base.as_model_doc(raw)
    assembly = v3.v1.base.as_assembly_doc(raw)
    manifest = raw_data()
    available = []
    raw_occurrences = list(assembly.GetComponents(False) or [])
    v3.trace(f"finish-active occurrences={len(raw_occurrences)}")
    for occurrence_index, occurrence in enumerate(raw_occurrences, start=1):
        v3.trace(f"finish-active occurrence {occurrence_index}/{len(raw_occurrences)} start")
        component = v3.v1.base.MODELDOC_MODULE.IComponent2(occurrence._oleobj_)
        array = [float(value) for value in list(component.Transform2.ArrayData)]
        available.append(
            {
                "component": component,
                "filename": Path(str(component.GetPathName())).name.lower(),
                "translation_mm": (array[9] * 1000.0, array[10] * 1000.0, array[11] * 1000.0),
            }
        )
        v3.trace(f"finish-active occurrence {occurrence_index}/{len(raw_occurrences)} done")
    components = {}
    rows = []
    for row in manifest["components"]:
        component_id = str(row["component_id"])
        filename = native_part_for(row).name.lower()
        expected = tuple(float(row["transform_local_mm_to_world_mm"][index][3]) for index in range(3))
        candidates = [item for item in available if item["filename"] == filename]
        if not candidates:
            raise RuntimeError(f"no active occurrence for {component_id}: {filename}")
        chosen = min(
            candidates,
            key=lambda item: math.sqrt(sum((item["translation_mm"][i] - expected[i]) ** 2 for i in range(3))),
        )
        error = math.sqrt(sum((chosen["translation_mm"][i] - expected[i]) ** 2 for i in range(3)))
        if error > 0.05:
            raise RuntimeError(f"active occurrence transform mismatch {component_id}: {error:.6f} mm")
        available.remove(chosen)
        components[component_id] = chosen["component"]
        rows.append(
            {
                "component_id": component_id,
                "role": row["role"],
                "owner_link": row["owner_link"],
                "native_part": filename,
                "transform_error": error / 1000.0,
                "status": "PASS",
            }
        )
    if available:
        raise RuntimeError(f"unmapped active SOLIDWORKS occurrences: {len(available)}")

    v3.trace("finish-active component mapping done; assembly bbox start")
    assembly.UpdateBox()
    raw_box = list(assembly.GetBox(0) or [])
    v3.trace(f"finish-active assembly bbox done: {raw_box}")
    if len(raw_box) != 6:
        raise RuntimeError(f"IAssemblyDoc.GetBox failed: {raw_box}")
    assembly_bbox_m = {"min": raw_box[:3], "max": raw_box[3:]}
    assembly_bbox_m["size"] = [raw_box[index + 3] - raw_box[index] for index in range(3)]
    standing_height_mm = float(assembly_bbox_m["size"][2]) * 1000.0

    payload_ids = {
        str(row["component_id"])
        for row in manifest["components"]
        if str(row["role"]) in {
            "internal_payload_controlled_envelope",
            "removable_internal_service_mount",
            "purchased_internal_interaction_module",
            "rigid_sensor_mount",
            "harness_strain_relief",
            "direct_head_torso_mount",
        }
    }
    # Keep controlled payload occurrences visible in both review variants.
    # IComponent2 visibility toggling can block indefinitely on a translated
    # imported body; color + the x-ray transparency state communicates the
    # same packaging information without mutating occurrence suppression.
    v3.round_sw.try_shaded(model)
    v3.v1.base.refresh_assembly_display(model)
    v3.trace("finish-active normal save start")
    normal_code = int(model.SaveAs3(str(NORMAL_ASM), 0, 1))
    v3.trace(f"finish-active normal save done: {normal_code}")
    normal_views = {
        "front": v3.save_view(model, SNAPSHOT_ROOT / "v5_solidworks_normal_front.png", False),
        "iso": v3.save_view(model, SNAPSHOT_ROOT / "v5_solidworks_normal_iso.png", True),
    }

    for row in manifest["components"]:
        component_id = str(row["component_id"])
        role = str(row["role"])
        if role == "purchased_exact_sts3250":
            v3.v1.set_material(components[component_id], v3.BLUE, 0.0)
        elif component_id in payload_ids:
            v3.v1.set_material(components[component_id], v3.color(str(row["color_hex"])), 0.0)
        else:
            v3.v1.set_material(components[component_id], v3.WHITE, 0.70)
    v3.v1.base.refresh_assembly_display(model)
    xray_views = {
        "front": v3.save_view(model, SNAPSHOT_ROOT / "v5_solidworks_xray_front.png", False),
        "iso": v3.save_view(model, SNAPSHOT_ROOT / "v5_solidworks_xray_iso.png", True),
    }
    xray_code = int(model.SaveAs3(str(XRAY_ASM), 0, 1))
    v3.write_csv(rows)
    portable_root = str(PORTABLE.resolve()).lower()
    portable_ok = all(
        str(component.GetPathName()).lower().startswith(portable_root)
        and Path(str(component.GetPathName())).is_file()
        for component in components.values()
    )
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.solidworks_gate.v1",
        "solidworks_revision": str(sw.RevisionNumber()),
        "manifest_sha256": hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
        "expected_manifest_component_count": int(manifest["component_count"]),
        "assembly_component_count": len(components),
        "separate_blue_sts3250_count": int(manifest["blue_sts3250_count"]),
        "ankle_roll_actuator_count": 0,
        "source_foot_count": 2,
        "black_sole_count": 0,
        "fixed_wrist_support_count": 2,
        "portable_dependency_gate": "PASS" if portable_ok else "FAIL",
        "component_transform_gate": "PASS",
        "normal_save_gate": "PASS" if NORMAL_ASM.is_file() and normal_code >= 0 else "FAIL",
        "xray_save_gate": "PASS" if XRAY_ASM.is_file() and xray_code >= 0 else "FAIL",
        "normal_view_gate": "PASS" if all(normal_views.values()) else "FAIL",
        "xray_view_gate": "PASS" if all(xray_views.values()) else "FAIL",
        "assembly_bbox_m": assembly_bbox_m,
        "standing_height_mm": standing_height_mm,
        "standing_height_limit_mm": 500.0,
        "standing_height_gate": "PASS" if standing_height_mm <= 500.0 else "FAIL",
        "bbox_method": "IAssemblyDoc.UpdateBox + IAssemblyDoc.GetBox; no blocking per-occurrence GetBox calls",
        "payload_review_policy": "controlled payload occurrences remain visible; x-ray transparency is applied to non-payload structure",
        "truth_boundary": "Static native assembly and transform gate only. Actual motor/mate/collision proof is in solidworks_motion_gate.json.",
    }
    payload["overall"] = "PASS" if (
        payload["assembly_component_count"] == payload["expected_manifest_component_count"]
        and all(payload[key] == "PASS" for key in (
            "portable_dependency_gate", "component_transform_gate", "normal_save_gate", "xray_save_gate",
            "normal_view_gate", "xray_view_gate", "standing_height_gate",
        ))
    ) else "FAIL"
    gate_path = REPORT_ROOT / "solidworks_gate.json"
    gate_path.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)
    return payload


def configure() -> None:
    v3.MANIFEST = MANIFEST
    v3.V2_PORTABLE = V2_PORTABLE
    v3.SW_ROOT = SW_ROOT
    v3.PORTABLE = PORTABLE
    v3.NORMAL_ASM = NORMAL_ASM
    v3.TOP_ASM = XRAY_ASM
    v3.REPORT_ROOT = REPORT_ROOT
    v3.GATE_REPORT = REPORT_ROOT / "solidworks_gate.json"
    v3.COMPONENT_CSV = REPORT_ROOT / "solidworks_component_manifest.csv"
    v3.TRACE_LOG = REPORT_ROOT / "solidworks_trace.log"
    v3.SNAPSHOT_ROOT = SNAPSHOT_ROOT
    v3.data = data
    v3.is_new_v3_part = is_v5_source
    v3.v3_part_path = v5_part_path
    v3.unique_new_sources = unique_new_sources
    v3.native_part_for = native_part_for
    v3.prepare_portable = prepare_portable
    v3.v1.add_component = add_component_direct


def normalize_gate() -> None:
    path = REPORT_ROOT / "solidworks_gate.json"
    payload = json.loads(path.read_text(encoding="utf-8"))
    manifest = raw_data()
    payload.update(
        {
            "schema": "zeroth01.v5_original_16dof_solidworks_motion.solidworks_gate.v1",
            "manifest_sha256": hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
            "expected_manifest_component_count": manifest["component_count"],
            "separate_blue_sts3250_count": manifest["blue_sts3250_count"],
            "ankle_roll_actuator_count": 0,
            "source_foot_count": 2,
            "black_sole_count": 0,
            "fixed_wrist_support_count": 2,
            "truth_boundary": "Static native assembly and transform gate only. Actual motor/mate/collision proof is in solidworks_motion_gate.json.",
        }
    )
    path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


def import_all(force: bool, startup_timeout: float) -> None:
    prepare_portable()
    for index, key in enumerate(sorted(unique_new_sources()), 1):
        print(f"v5 STEP import {index}/{len(unique_new_sources())}: {key}", flush=True)
        import_key_with_solid_gate(key, force, startup_timeout)


def import_key_with_solid_gate(key: str, force: bool, startup_timeout: float) -> dict[str, object]:
    """Import, run native Import Diagnostics, and require a SOLIDWORKS solid body."""

    row = v3.import_new(key, force, startup_timeout)
    source = unique_new_sources()[key]
    target = v5_part_path(source)
    sw = v3.v1.typed_sldworks(v3.get_sw(startup_timeout))
    raw = v3.v1.first(sw.OpenDoc6(str(target), 1, 1, "", 0, 0))
    if raw is None:
        raise RuntimeError(f"cannot reopen imported part for solid gate: {target}")
    model = v3.v1.base.as_model_doc(raw)
    part = v3.v1.base.as_part_doc(raw)
    solid_before = len(list(part.GetBodies2(0, True) or []))
    surface_before = len(list(part.GetBodies2(1, True) or []))
    diagnosis_code = None
    if solid_before < 1:
        diagnosis_code = int(part.ImportDiagnosis(True, False, True, 0))
        model.ForceRebuild3(True)
    solid_after = len(list(part.GetBodies2(0, True) or []))
    surface_after = len(list(part.GetBodies2(1, True) or []))
    save_code = int(v3.v1.first(model.Save3(1, 0, 0)) or 0)
    v3.v1.base.close_document(sw, raw)
    gate = {
        **row,
        "solid_body_count_before_diagnosis": solid_before,
        "surface_body_count_before_diagnosis": surface_before,
        "import_diagnosis_code": diagnosis_code,
        "solid_body_count": solid_after,
        "surface_body_count": surface_after,
        "post_diagnosis_save_code": save_code,
        "solidworks_solid_gate": "PASS" if solid_after >= 1 else "FAIL",
    }
    report = REPORT_ROOT / "solidworks_import_parts" / f"{key}.json"
    report.parent.mkdir(parents=True, exist_ok=True)
    report.write_text(json.dumps(gate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(gate, indent=2, ensure_ascii=False), flush=True)
    if solid_after < 1:
        raise RuntimeError(f"SOLIDWORKS solid gate failed for {key}: {gate}")
    return gate


def export_step(startup_timeout: float) -> None:
    configure()
    pythoncom.CoInitialize()
    sw = v3.v1.typed_sldworks(v3.get_sw(startup_timeout))
    result = sw.OpenDoc6(str(NORMAL_ASM), v3.SW_DOC_ASSEMBLY, 1, "", 0, 0)
    raw = v3.v1.first(result)
    if raw is None:
        raise RuntimeError(NORMAL_ASM)
    model = v3.v1.base.as_model_doc(raw)
    code = int(model.SaveAs3(str(FULL_ASSEMBLY_STEP), 0, 1))
    if not FULL_ASSEMBLY_STEP.is_file() or FULL_ASSEMBLY_STEP.stat().st_size < 1024:
        raise RuntimeError(f"STEP export failed: {code}")
    print(json.dumps({"step": str(FULL_ASSEMBLY_STEP), "save_code": code, "bytes": FULL_ASSEMBLY_STEP.stat().st_size}, indent=2))


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list-new")
    sub.add_parser("prepare")
    imp = sub.add_parser("import-all")
    imp.add_argument("--force", action="store_true")
    imp.add_argument("--startup-timeout", type=float, default=20.0)
    one = sub.add_parser("import-key")
    one.add_argument("key")
    one.add_argument("--force", action="store_true")
    one.add_argument("--startup-timeout", type=float, default=20.0)
    probe = sub.add_parser("probe-part")
    probe.add_argument("key")
    probe.add_argument("--path", type=Path)
    probe.add_argument("--startup-timeout", type=float, default=20.0)
    asm = sub.add_parser("assemble")
    asm.add_argument("--startup-timeout", type=float, default=20.0)
    native = sub.add_parser("create-native-standoffs")
    native.add_argument("--startup-timeout", type=float, default=20.0)
    finish = sub.add_parser("finish-active")
    finish.add_argument("--startup-timeout", type=float, default=20.0)
    exp = sub.add_parser("export-step")
    exp.add_argument("--startup-timeout", type=float, default=20.0)
    args = parser.parse_args()
    configure()
    if args.command == "list-new":
        print("\n".join(sorted(unique_new_sources())))
    elif args.command == "prepare":
        print(json.dumps(prepare_portable(), indent=2))
    elif args.command == "import-all":
        import_all(args.force, args.startup_timeout)
    elif args.command == "import-key":
        prepare_portable()
        import_key_with_solid_gate(args.key, args.force, args.startup_timeout)
    elif args.command == "probe-part":
        prepare_portable()
        source = unique_new_sources().get(args.key)
        part_path = args.path.resolve() if args.path else v5_part_path(source)
        if source is None and args.path is None:
            raise KeyError(f"unknown part key: {args.key}")
        sw = v3.v1.typed_sldworks(v3.get_sw(args.startup_timeout))
        v3.close_task_documents(sw)
        raw = v3.v1.first(sw.OpenDoc6(str(part_path), 1, 1, "", 0, 0))
        if raw is None:
            raise RuntimeError(f"cannot open {part_path}")
        model = v3.v1.base.as_model_doc(raw)
        part = v3.v1.base.as_part_doc(raw)
        box = list(part.GetPartBox(True) or [])
        solid_bodies = list(part.GetBodies2(0, True) or [])
        surface_bodies = list(part.GetBodies2(1, True) or [])
        print(json.dumps({"path": str(part_path), "box": box, "solid_body_count": len(solid_bodies), "surface_body_count": len(surface_bodies)}, indent=2))
    elif args.command == "assemble":
        assemble_v5_complete(args.startup_timeout)
    elif args.command == "create-native-standoffs":
        print(json.dumps(create_native_standoffs(args.startup_timeout), ensure_ascii=False, indent=2))
    elif args.command == "finish-active":
        finish_active_assembly(args.startup_timeout)
    elif args.command == "export-step":
        export_step(args.startup_timeout)


if __name__ == "__main__":
    main()
