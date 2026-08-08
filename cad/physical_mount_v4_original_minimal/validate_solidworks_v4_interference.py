"""Run and classify SolidWorks interference for the released v4 assembly."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pythoncom
import win32com.client


ROOT = Path(__file__).resolve().parents[2]
PORTABLE = ROOT / "generated" / "solidworks" / "physical_mount_v4_original_minimal" / "portable_flat"
NORMAL = PORTABLE / "OPEN_FIRST_ZEROTH01_V4_ORIGINAL_MINIMAL_WHITE_18_BLUE_STS3250.SLDASM"
XRAY = PORTABLE / "OPTIONAL_XRAY_ZEROTH01_V4_ORIGINAL_MINIMAL_INTERNAL_LAYOUT.SLDASM"
REPORT = ROOT / "reports" / "v4_original_minimal" / "solidworks_interference_gate.json"
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"


def value(obj, name):
    item = getattr(obj, name)
    return item() if callable(item) else item


def classification(components: list[str], volume_mm3: float, possible: bool) -> str:
    if components and len(set(components)) == 1:
        return "same_component_multibody_union_overlap"
    if possible and volume_mm3 <= 1.0e-9:
        return "zero_volume_contact_only"
    upper = [component.upper() for component in components]
    if (
        len(upper) == 2
        and any("STS3250_STEP_PARTS_EXACT_SHAFT_FRAME" in component for component in upper)
        and any("STS3250_PCD14_4XM3_TIE_RODS_1P95MM" in component for component in upper)
        and volume_mm3 <= 1.25
    ):
        # The purchased STEP simplifies the four tapped M3 output holes.  The
        # modeled screw shanks intentionally enter those threads; only this
        # tightly bounded same-interface engagement is allowed.
        return "intentional_threaded_fastener_engagement"
    return "physical_interference"


def main() -> int:
    pythoncom.CoInitialize()
    sw = win32com.client.Dispatch("SldWorks.Application")
    document = sw.ActiveDoc
    valid = {NORMAL.resolve(), XRAY.resolve()}
    active = Path(str(value(document, "GetPathName"))) if document else None
    if active is None or active.resolve() not in valid:
        document = None
        for candidate in value(sw, "GetDocuments") or []:
            path = Path(str(value(candidate, "GetPathName")))
            if path.resolve() in valid:
                document = candidate
                active = path
                break
    if document is None:
        raise RuntimeError("open the v4 normal or xray SolidWorks assembly")

    manager = document.InterferenceDetectionManager
    manager.TreatCoincidenceAsInterference = False
    manager.TreatSubAssembliesAsComponents = False
    manager.IncludeMultibodyPartInterferences = False
    manager.IgnoreHiddenBodies = False
    try:
        rows = []
        for interference in value(manager, "GetInterferences") or []:
            components = [
                str(value(component, "Name2"))
                for component in (value(interference, "Components") or [])
            ]
            volume_mm3 = float(value(interference, "Volume")) * 1.0e9
            possible = bool(value(interference, "IsPossibleInterference"))
            rows.append(
                {
                    "components": components,
                    "volume_mm3": volume_mm3,
                    "possible_interference": possible,
                    "classification": classification(components, volume_mm3, possible),
                }
            )
    finally:
        value(manager, "Done")
    physical = [row for row in rows if row["classification"] == "physical_interference"]
    payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.solidworks_interference_gate.v1",
        "solidworks_revision": str(value(sw, "RevisionNumber")),
        "assembly": str(active),
        "manifest_sha256": hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
        "settings": {
            "treat_coincidence_as_interference": False,
            "include_multibody_part_interferences": False,
            "ignore_hidden_bodies": False,
        },
        "raw_interference_count": len(rows),
        "same_component_union_overlap_count": sum(
            row["classification"] == "same_component_multibody_union_overlap" for row in rows
        ),
        "zero_volume_contact_only_count": sum(
            row["classification"] == "zero_volume_contact_only" for row in rows
        ),
        "intentional_threaded_fastener_engagement_count": sum(
            row["classification"] == "intentional_threaded_fastener_engagement" for row in rows
        ),
        "physical_interference_count": len(physical),
        "rows": rows,
        "truth_boundary": (
            "Cross-component intersection must be zero except the explicitly modeled M3 screw "
            "shanks entering the purchased STS3250 tapped PCD14 holes (each <=1.25 mm^3). "
            "All other actuators, shells, torque bridges, mounts, carriers and feet must not intersect."
        ),
        "overall": "PASS" if not physical else "FAIL",
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(
        json.dumps(
            {
                "assembly": payload["assembly"],
                "raw_interference_count": payload["raw_interference_count"],
                "physical_interference_count": payload["physical_interference_count"],
                "physical_rows": physical,
                "overall": payload["overall"],
            },
            indent=2,
            ensure_ascii=False,
        )
    )
    return 0 if not physical else 1


if __name__ == "__main__":
    raise SystemExit(main())
