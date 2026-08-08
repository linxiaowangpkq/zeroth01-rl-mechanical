"""Identify every v5 interference at the 97-component SOLIDWORKS level.

The 17-link Motion assembly intentionally groups each rigid link into one
multibody SLDPRT.  That is the correct Motion topology, but a link-level
interference report cannot identify whether the colliding solid is the
released carrier, purchased STS3250, output bridge, spacer, or fastener.
This diagnostic opens the manufacturing assembly and maps every SOLIDWORKS
interference back to the assembly-manifest component IDs.
"""

from __future__ import annotations

import json
from pathlib import Path

import pythoncom
import win32com.client as win32

import create_solidworks_motion_v5 as motion


REPORT = motion.REPORT_ROOT / "solidworks_static_component_interference.json"
SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1


def value(obj, name, default=None):
    try:
        item = getattr(obj, name)
        return item() if callable(item) else item
    except Exception:
        return default


def component_key(component) -> tuple[str, str, tuple[float, float, float]]:
    name = str(value(component, "Name2", ""))
    path = str(value(component, "GetPathName", ""))
    try:
        transform = motion.component_transform_array(component)
        translation = tuple(round(v, 5) for v in motion.r31.arr_translation_mm(transform))
    except Exception:
        translation = (0.0, 0.0, 0.0)
    return name, Path(path).name.lower(), translation


def main() -> int:
    pythoncom.CoInitialize()
    motion.v5sw.configure()
    # A cold SOLIDWORKS start on this workstation regularly needs more than
    # 20 seconds to register its COM server.  This timeout covers startup
    # only; the interference calculation itself remains externally watched.
    sw = motion.v5sw.v3.v1.typed_sldworks(motion.v5sw.v3.get_sw(90.0))
    sw.Visible = True
    motion.v5sw.v3.close_task_documents(sw)
    raw = motion.v5sw.v3.v1.first(
        sw.OpenDoc6(str(motion.SOURCE_ASM), SW_DOC_ASSEMBLY, SW_OPEN_SILENT, "", 0, 0)
    )
    if raw is None:
        raise RuntimeError(f"cannot open manufacturing assembly: {motion.SOURCE_ASM}")
    model = motion.r31.swp.as_model_doc(raw)
    assembly = motion.r31.swp.as_assembly_doc(raw)
    assembly.ResolveAllLightWeightComponents(True)
    model.ForceRebuild3(False)

    data = motion.manifest_data()
    by_manifest_id = motion.components_by_manifest_id(assembly)
    metadata = {str(row["component_id"]): row for row in data["components"]}
    key_to_id = {
        component_key(component): component_id
        for component_id, component in by_manifest_id.items()
    }
    # Name2 is unique inside this flat assembly and survives the interference
    # API handoff more reliably than COM wrapper identity.
    name_to_id = {
        component_key(component)[0]: component_id
        for component_id, component in by_manifest_id.items()
    }

    manager = assembly.InterferenceDetectionManager
    manager.TreatCoincidenceAsInterference = False
    manager.TreatSubAssembliesAsComponents = False
    manager.IncludeMultibodyPartInterferences = False
    manager.IgnoreHiddenBodies = False
    assembly.ToolsCheckInterference2(0, None, False)
    rows = []
    try:
        for index, interference in enumerate(list(value(manager, "GetInterferences", []) or []), start=1):
            raw_components = list(value(interference, "Components", []) or [])
            ids = []
            component_debug = []
            for component in raw_components:
                key = component_key(component)
                component_id = key_to_id.get(key) or name_to_id.get(key[0])
                ids.append(component_id or "UNMAPPED")
                component_debug.append({
                    "name": key[0],
                    "native_file": key[1],
                    "translation_mm": key[2],
                })
            a = metadata.get(ids[0], {}) if len(ids) > 0 else {}
            b = metadata.get(ids[1], {}) if len(ids) > 1 else {}
            rows.append({
                "index": index,
                "volume_mm3": float(value(interference, "Volume", 0.0)) * 1.0e9,
                "component_ids": ids,
                "roles": [a.get("role", ""), b.get("role", "")],
                "owner_links": [a.get("owner_link", ""), b.get("owner_link", "")],
                "possible_interference": bool(value(interference, "IsPossibleInterference", False)),
                "components": component_debug,
            })
    finally:
        value(manager, "Done")

    unmapped = sum("UNMAPPED" in row["component_ids"] for row in rows)
    payload = {
        "schema": "zeroth01.v5_original_16dof_solidworks_motion.static_component_interference.v1",
        "solidworks_revision": str(sw.RevisionNumber()),
        "assembly": str(motion.SOURCE_ASM),
        "component_count": len(by_manifest_id),
        "raw_interference_count": len(rows),
        "total_volume_mm3": sum(row["volume_mm3"] for row in rows),
        "unmapped_interference_count": unmapped,
        "rows": rows,
        "truth_boundary": "No overlap is filtered or allow-listed; every positive-volume pair is reported by physical manifest component ID.",
        "overall": "PASS" if not rows and not unmapped else "FAIL",
    }
    REPORT.write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, ensure_ascii=False, indent=2), flush=True)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
