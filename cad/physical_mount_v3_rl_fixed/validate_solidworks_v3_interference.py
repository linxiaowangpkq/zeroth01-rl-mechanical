"""Run the release interference gate in the installed SolidWorks instance."""

from __future__ import annotations

import json
from pathlib import Path

import pythoncom
import win32com.client


ROOT = Path(__file__).resolve().parents[2]
ASSEMBLY = (
    ROOT
    / "generated"
    / "solidworks"
    / "physical_mount_v3_rl_fixed"
    / "portable_flat"
    / "OPEN_FIRST_ZEROTH01_V3_RL_FIXED_CONNECTED_WHITE_18_BLUE_STS3250.SLDASM"
)
XRAY_ASSEMBLY = ASSEMBLY.with_name(
    "OPTIONAL_XRAY_ZEROTH01_V3_RL_FIXED_18_BLUE_STS3250.SLDASM"
)
REPORT = (
    ROOT
    / "reports"
    / "physical_mount_v3_rl_fixed"
    / "solidworks_interference_gate.json"
)
REFERENCE_TOKENS = (
    "FACE_GLASS_REFERENCE",
    "EXPRESSION_REFERENCE",
    "CAMERA_REFERENCE",
)


def value(obj, name):
    item = getattr(obj, name)
    return item() if callable(item) else item


def is_reference_overlay(components: list[str]) -> bool:
    return bool(components) and all(
        any(token in component for token in REFERENCE_TOKENS)
        for component in components
    )


def main() -> int:
    pythoncom.CoInitialize()
    sw = win32com.client.Dispatch("SldWorks.Application")
    document = sw.ActiveDoc
    active_path = Path(str(value(document, "GetPathName"))) if document else None
    valid_paths = {ASSEMBLY.resolve(), XRAY_ASSEMBLY.resolve()}
    if active_path is None or active_path.resolve() not in valid_paths:
        document = None
        for candidate in value(sw, "GetDocuments") or []:
            candidate_path = Path(str(value(candidate, "GetPathName")))
            if candidate_path.resolve() in valid_paths:
                document = candidate
                active_path = candidate_path
                break
    if document is None:
        raise RuntimeError(
            "open either the normal or optional-xray v3 SolidWorks assembly before validation"
        )

    manager = document.InterferenceDetectionManager
    manager.TreatCoincidenceAsInterference = False
    manager.TreatSubAssembliesAsComponents = False
    manager.IncludeMultibodyPartInterferences = True
    manager.IgnoreHiddenBodies = False
    try:
        rows = []
        for interference in value(manager, "GetInterferences") or []:
            components = [
                str(value(component, "Name2"))
                for component in (value(interference, "Components") or [])
            ]
            allowed = is_reference_overlay(components)
            rows.append(
                {
                    "components": components,
                    "volume_mm3": float(value(interference, "Volume")) * 1.0e9,
                    "possible_interference": bool(
                        value(interference, "IsPossibleInterference")
                    ),
                    "classification": (
                        "allowed_nonmanufacturing_reference_overlay"
                        if allowed
                        else "physical_interference"
                    ),
                }
            )
    finally:
        value(manager, "Done")

    physical = [row for row in rows if row["classification"] == "physical_interference"]
    payload = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.solidworks_interference_gate.v1",
        "solidworks_revision": str(value(sw, "RevisionNumber")),
        "assembly": str(active_path),
        "settings": {
            "treat_coincidence_as_interference": False,
            "treat_subassemblies_as_components": False,
            "include_multibody_part_interferences": True,
            "ignore_hidden_bodies": False,
        },
        "raw_interference_count": len(rows),
        "allowed_reference_overlay_count": len(rows) - len(physical),
        "physical_interference_count": len(physical),
        "rows": rows,
        "truth_boundary": (
            "Only StackChan face-glass/expression/camera display references may overlap; "
            "all load-bearing, actuator, adapter and shell components must have zero volume intersection."
        ),
        "overall": "PASS" if not physical else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0 if not physical else 1


if __name__ == "__main__":
    raise SystemExit(main())
