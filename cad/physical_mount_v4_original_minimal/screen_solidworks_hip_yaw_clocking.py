"""Screen symmetric hip-yaw STS3250 clocking and axial shim offsets in SolidWorks."""

from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import pythoncom
import win32com.client


ROOT = Path(__file__).resolve().parents[2]
ASSEMBLY = (
    ROOT
    / "generated"
    / "solidworks"
    / "physical_mount_v4_original_minimal"
    / "portable_flat"
    / "OPTIONAL_XRAY_ZEROTH01_V4_ORIGINAL_MINIMAL_INTERNAL_LAYOUT.SLDASM"
)
MANIFEST = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v4_original_minimal"
    / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)
KIN_SCRIPT = ROOT / "scripts" / "create_solidworks_kinematic_review.py"
V3_SW_SCRIPT = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "create_solidworks_v3.py"
OUT = ROOT / "reports" / "v4_original_minimal" / "hip_yaw_clocking_screening.json"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


kin = load(KIN_SCRIPT, "zeroth_v4_hip_screen_kin_helpers")
sw_helpers = load(V3_SW_SCRIPT, "zeroth_v4_hip_screen_sw_helpers")


def value(obj, name):
    item = getattr(obj, name)
    return item() if callable(item) else item


def mm_transform(row):
    matrix = row["transform_local_mm_to_world_mm"]
    rotation = tuple(tuple(float(matrix[i][j]) for j in range(3)) for i in range(3))
    translation = tuple(float(matrix[i][3]) / 1000.0 for i in range(3))
    return rotation, translation


def mat_mul(a, b):
    return tuple(
        tuple(sum(a[row][k] * b[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def rz(degrees):
    angle = math.radians(degrees)
    c, s = math.cos(angle), math.sin(angle)
    return ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))


def find_component(components, exact_name):
    for component in components:
        if str(value(component, "Name2")) == exact_name:
            return component
    raise KeyError(exact_name)


def set_transform(sw, component, rotation, translation):
    return kin.set_component_transform(sw, component, (rotation, translation))


def interference_volume(document, body_name, servo_names):
    manager = document.InterferenceDetectionManager
    manager.TreatCoincidenceAsInterference = False
    manager.TreatSubAssembliesAsComponents = False
    manager.IncludeMultibodyPartInterferences = False
    manager.IgnoreHiddenBodies = True
    total = 0.0
    counts = {name: 0 for name in servo_names}
    try:
        for interference in value(manager, "GetInterferences") or []:
            names = [str(value(component, "Name2")) for component in (value(interference, "Components") or [])]
            if body_name not in names:
                continue
            for servo_name in servo_names:
                if servo_name in names:
                    total += float(value(interference, "Volume")) * 1.0e9
                    counts[servo_name] += 1
    finally:
        value(manager, "Done")
    return total, counts


def cross_component_interferences(document):
    manager = document.InterferenceDetectionManager
    manager.TreatCoincidenceAsInterference = False
    manager.TreatSubAssembliesAsComponents = False
    manager.IncludeMultibodyPartInterferences = False
    manager.IgnoreHiddenBodies = False
    rows = []
    try:
        for interference in value(manager, "GetInterferences") or []:
            names = [
                str(value(component, "Name2"))
                for component in (value(interference, "Components") or [])
            ]
            volume_mm3 = float(value(interference, "Volume")) * 1.0e9
            if len(set(names)) > 1 and volume_mm3 > 1.0e-9:
                rows.append({"components": names, "volume_mm3": volume_mm3})
    finally:
        value(manager, "Done")
    return rows


def main() -> int:
    pythoncom.CoInitialize()
    sw = sw_helpers.v1.typed_sldworks(sw_helpers.get_sw(30.0))
    document = None
    for candidate in value(sw, "GetDocuments") or []:
        if Path(str(value(candidate, "GetPathName"))).resolve() == ASSEMBLY.resolve():
            document = candidate
            break
    if document is None:
        raise RuntimeError(f"open the xray v4 assembly first: {ASSEMBLY}")
    components = list(document.GetComponents(False) or [])
    body_name = "ZEROTH01_V4_BODY_ORIGINAL_HEAD_INTERFACE_TRIMMED_2P5MM-1"
    left_name = "ZEROTH01_V4_STS3250_STEP_PARTS_EXACT_SHAFT_FRAME-7"
    right_name = "ZEROTH01_V4_STS3250_STEP_PARTS_EXACT_SHAFT_FRAME-8"
    body = find_component(components, body_name)
    left = find_component(components, left_name)
    right = find_component(components, right_name)
    keep = {body_name, left_name, right_name}
    suppression = {}
    for component in components:
        name = str(value(component, "Name2"))
        try:
            suppression[name] = int(value(component, "GetSuppression2"))
            component.SetSuppression2(2 if name in keep else 0)
        except Exception:
            pass

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    rows_by_id = {row["component_id"]: row for row in manifest["components"]}
    left_base = mm_transform(rows_by_id["S08_STS3250_left_hip_yaw"])
    right_base = mm_transform(rows_by_id["S07_STS3250_right_hip_yaw"])
    original_left = list(left.Transform2.ArrayData)
    original_right = list(right.Transform2.ArrayData)
    results = []
    try:
        for angle_deg in (0,):
            for shim_mm in range(-14, 15, 2):
                left_rotation = mat_mul(rz(angle_deg), left_base[0])
                right_rotation = mat_mul(rz(-angle_deg), right_base[0])
                left_translation = (left_base[1][0], left_base[1][1], left_base[1][2] + shim_mm / 1000.0)
                right_translation = (right_base[1][0], right_base[1][1], right_base[1][2] + shim_mm / 1000.0)
                error = max(
                    set_transform(sw, left, left_rotation, left_translation),
                    set_transform(sw, right, right_rotation, right_translation),
                )
                value(document, "EditRebuild3")
                volume, counts = interference_volume(document, body_name, (left_name, right_name))
                results.append(
                    {
                        "symmetric_clocking_deg": angle_deg,
                        "axial_shim_mm": shim_mm,
                        "body_servo_interference_volume_mm3": volume,
                        "body_servo_interference_body_pair_count": sum(counts.values()),
                        "transform_error": error,
                    }
                )
    finally:
        left.Transform2 = kin.create_math_transform(sw, original_left)
        right.Transform2 = kin.create_math_transform(sw, original_right)
        for component in components:
            name = str(value(component, "Name2"))
            if name in suppression:
                try:
                    component.SetSuppression2(suppression[name])
                except Exception:
                    pass
        value(document, "EditRebuild3")

    results.sort(key=lambda row: (row["body_servo_interference_volume_mm3"], abs(row["axial_shim_mm"]), abs(row["symmetric_clocking_deg"])))
    best = results[0]
    full_rows = []
    try:
        shim_m = float(best["axial_shim_mm"]) / 1000.0
        set_transform(sw, left, left_base[0], (left_base[1][0], left_base[1][1], left_base[1][2] + shim_m))
        set_transform(sw, right, right_base[0], (right_base[1][0], right_base[1][1], right_base[1][2] + shim_m))
        value(document, "EditRebuild3")
        full_rows = cross_component_interferences(document)
    finally:
        left.Transform2 = kin.create_math_transform(sw, original_left)
        right.Transform2 = kin.create_math_transform(sw, original_right)
        value(document, "EditRebuild3")
    payload = {
        "schema": "zeroth01.v4.solidworks_hip_yaw_clocking_screening.v1",
        "assembly": str(ASSEMBLY),
        "tested_count": len(results),
        "best_20": results[:20],
        "zero_interference_count": sum(row["body_servo_interference_volume_mm3"] <= 1.0e-6 for row in results),
        "best_full_assembly_cross_component_interference_count": len(full_rows),
        "best_full_assembly_cross_component_interferences": full_rows,
    }
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
