"""Build a printer-neutral, right-arm-first manufacturing/test subset.

The robot CAD remains common between supported Bambu printers.  This builder
does not reslice geometry or claim a physical fit.  It copies the released
V5 parts into an RA-numbered subset, exports a coloured neutral-pose right-arm
reference from the authoritative occurrence manifest, and writes the gates
needed before a low-current powered test.
"""

from __future__ import annotations

import csv
import hashlib
import json
import shutil
from pathlib import Path

from build123d import Color, Compound, Location, export_gltf, export_step, import_step
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.gp import gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
V5_NAME = "physical_mount_v5_original_16dof_solidworks_motion"
MANIFEST = ROOT / "generated" / "cad" / V5_NAME / "ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json"
PRINT_RELEASE = ROOT / "manufacturing" / "v5_bambu_first_article_release"
OUT = PRINT_RELEASE / "arm_first_article"
GENERATED_PART_SUBDIRS = (
    "required_arm",
    "optional_unpowered_wrist_support",
    "optional_real_torso_anchor",
    "fit_check_only",
)


# The upstream source names are mirrored.  L_ARM_MIRROR_1 is the physical
# right forearm in the released V5 kinematic chain; keep that fact explicit.
ARM_PRINT_PARTS = [
    {
        "arm_id": "RA01",
        "release_name": "29_z_bot2_master_shoulder2_2_sts3250_clearanced",
        "robot_link": "Z_BOT2_MASTER_SHOULDER2_2",
        "physical_name_zh": "右肩双轴转接架",
        "function": "S03 shoulder-yaw child and S01 shoulder-pitch parent interface",
        "color_hex": "#FFB000",
    },
    {
        "arm_id": "RA02",
        "release_name": "09_3215_1flange_2_sts3250_clearanced",
        "robot_link": "3215_1Flange_2",
        "physical_name_zh": "右上臂承力架",
        "function": "carries the S01 shoulder-pitch and S09 elbow-yaw servo housings",
        "color_hex": "#FF6B35",
    },
    {
        "arm_id": "RA03",
        "release_name": "20_l_arm_mirror_1_sts3250_wrist_mount",
        "robot_link": "L_ARM_MIRROR_1",
        "physical_name_zh": "右前臂/腕端承力架",
        "function": "S09 elbow-yaw child; legacy mirrored source name is L_ARM_MIRROR_1",
        "color_hex": "#26A269",
    },
]

# The current wrist bumper sits outside the forearm and depends on two long
# through-bolts that are not yet frozen in the released occurrence manifest.
# Keep it available for a later hand/bumper study, but do not put it in the
# powered first-article torque-chain package or its coloured reference.
OPTIONAL_WRIST_SUPPORT = {
    "arm_id": "RA04",
    "release_name": "24_right_fixed_wrist_support",
    "robot_link": "L_ARM_MIRROR_1",
    "physical_name_zh": "右腕固定圆角支撑（可选，暂不上电）",
    "function": "optional endpoint study; excluded from powered first article until its through-bolt stack is frozen",
    "color_hex": "#00A6D6",
}

OPTIONAL_ANCHOR = {
    "arm_id": "RA00",
    "release_name": "26_torso_mounted_solid",
    "robot_link": "Z_BOT2_MASTER_BODY_SKELETON",
    "physical_name_zh": "原机身右肩固定端（可选）",
    "function": "required only when the real torso is used as the S03 housing anchor; otherwise use a rigid metal bench anchor",
    "color_hex": "#D0D3D8",
}

FIT_CHECKS = [
    ("FC01", "01_sts3250_4xm2_first_article_face_gauge"),
    ("FC02", "02_sts3250_parent_thrust_ring_2mm"),
    ("FC03", "04_sts3250_pcd14_output_bridge_2p05mm"),
]

RIGHT_ARM_JOINTS = ("S03", "S01", "S09")
RIGHT_ARM_COMPONENT_IDS = {
    "CARRIER_Z_BOT2_MASTER_SHOULDER2_2",
    "CARRIER_3215_1Flange_2",
    "CARRIER_L_ARM_MIRROR_1",
}
RIGHT_ARM_COMPONENT_PREFIXES = tuple(f"{joint}_" for joint in RIGHT_ARM_JOINTS)

PRINT_COLOR_BY_COMPONENT = {
    "CARRIER_Z_BOT2_MASTER_SHOULDER2_2": "#FFB000",
    "CARRIER_3215_1Flange_2": "#FF6B35",
    "CARRIER_L_ARM_MIRROR_1": "#26A269",
}


def rel(path: Path) -> str:
    return path.resolve().relative_to(ROOT.resolve()).as_posix()


def transformed(shape, matrix):
    transform = gp_Trsf()
    transform.SetValues(
        float(matrix[0][0]), float(matrix[0][1]), float(matrix[0][2]), float(matrix[0][3]),
        float(matrix[1][0]), float(matrix[1][1]), float(matrix[1][2]), float(matrix[1][3]),
        float(matrix[2][0]), float(matrix[2][1]), float(matrix[2][2]), float(matrix[2][3]),
    )
    return shape.moved(Location(gp_trsf=transform))


def load_print_manifest() -> dict[str, dict[str, str]]:
    with (PRINT_RELEASE / "PRINT_MANIFEST.csv").open(newline="", encoding="utf-8-sig") as stream:
        return {row["release_name"]: row for row in csv.DictReader(stream)}


def reset_generated_part_subsets() -> None:
    """Remove only the four fixed, generator-owned part subdirectories."""
    OUT.mkdir(parents=True, exist_ok=True)
    for name in GENERATED_PART_SUBDIRS:
        target = OUT / name
        if target.exists():
            shutil.rmtree(target)


def copy_print_part(item: dict[str, str], source_row: dict[str, str], category: str) -> dict[str, object]:
    copied: dict[str, str] = {}
    for fmt, source_key, output_key in (
        ("3mf", "three_mf", "three_mf"),
        ("assembly_frame_step", "assembly_frame_step", "assembly_frame_step"),
        ("print_oriented_step", "print_oriented_step", "print_oriented_step"),
    ):
        source = ROOT / source_row[source_key]
        suffix = source.suffix.lower()
        target = OUT / category / fmt / f"{item['arm_id']}_{item['release_name']}{suffix}"
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, target)
        copied[output_key] = rel(target)
    return {
        **item,
        "quantity": 1,
        "material": source_row["material"],
        "nozzle_mm": float(source_row["nozzle_mm"]),
        "layer_mm": float(source_row["layer_mm"]),
        "walls": int(source_row["walls"]),
        "top_bottom_layers": int(source_row["top_bottom_layers"]),
        "infill_percent": int(source_row["infill_percent"]),
        "bbox_mm": source_row["bbox_mm"],
        "solid_material_upper_bound_g": float(source_row["solid_material_upper_bound_g_each"]),
        "category": category,
        **copied,
    }


def copy_fit_checks() -> list[dict[str, str]]:
    rows = []
    for fit_id, stem in FIT_CHECKS:
        copied = {}
        for suffix in (".3mf", ".step"):
            source = PRINT_RELEASE / "fit_check_only" / f"{stem}{suffix}"
            target = OUT / "fit_check_only" / f"{fit_id}_{stem}{suffix}"
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, target)
            copied[suffix.removeprefix(".")] = rel(target)
        rows.append(
            {
                "fit_id": fit_id,
                "name": stem,
                "status": "FIT_CHECK_ONLY_NOT_FOR_POWERED_TORQUE_PATH",
                **copied,
            }
        )
    return rows


def export_coloured_arm_reference(manifest: dict[str, object]) -> dict[str, object]:
    selected = [
        row
        for row in manifest["components"]
        if row["component_id"] in RIGHT_ARM_COMPONENT_IDS
        or str(row["component_id"]).startswith(RIGHT_ARM_COMPONENT_PREFIXES)
    ]
    cached = {}
    children = []
    source_checks = []
    for row in selected:
        source = ROOT / str(row["source"])
        if source not in cached:
            cached[source] = import_step(source)
        source_shape = cached[source]
        placed = transformed(source_shape, row["transform_local_mm_to_world_mm"])
        placed.label = str(row["component_id"])
        color = PRINT_COLOR_BY_COMPONENT.get(str(row["component_id"]), str(row["color_hex"]))
        try:
            placed.color = Color(color)
        except Exception:
            pass
        children.append(placed)
        source_checks.append(
            {
                "component_id": row["component_id"],
                "role": row["role"],
                "owner_link": row["owner_link"],
                "source": rel(source),
                "source_brep_valid": bool(BRepCheck_Analyzer(source_shape.wrapped).IsValid()),
                "source_solid_count": len(source_shape.solids()),
            }
        )
    assembly = Compound(label="ZEROTH01_V5_RIGHT_ARM_FIRST_ARTICLE_REFERENCE", children=children)
    step_path = OUT / "RIGHT_ARM_NEUTRAL_ASSEMBLY_REFERENCE.step"
    glb_path = OUT / "RIGHT_ARM_NEUTRAL_ASSEMBLY_REFERENCE.glb"
    export_step(assembly, step_path)
    gltf_ok = export_gltf(assembly, glb_path, binary=True, linear_deflection=0.08, angular_deflection=0.15)
    verified = import_step(step_path)
    box = assembly.bounding_box()
    counts_by_role: dict[str, int] = {}
    for row in selected:
        role = str(row["role"])
        counts_by_role[role] = counts_by_role.get(role, 0) + 1
    return {
        "selected_component_count": len(selected),
        "unique_step_source_count": len(cached),
        "counts_by_role": counts_by_role,
        "joint_ids": list(RIGHT_ARM_JOINTS),
        "assembly_reference_step": rel(step_path),
        "assembly_reference_glb": rel(glb_path),
        "glb_export_ok": bool(gltf_ok),
        "reimported_step_brep_valid": bool(BRepCheck_Analyzer(verified.wrapped).IsValid()),
        "reimported_step_solid_count": len(verified.solids()),
        "bbox_mm": [round(float(value), 3) for value in box.size],
        "source_checks": source_checks,
    }


def write_print_manifest(rows: list[dict[str, object]]) -> None:
    fields = [
        "arm_id", "physical_name_zh", "robot_link", "release_name", "category", "quantity", "function",
        "material", "nozzle_mm", "layer_mm", "walls", "top_bottom_layers", "infill_percent", "bbox_mm",
        "solid_material_upper_bound_g", "color_hex", "three_mf", "assembly_frame_step", "print_oriented_step",
    ]
    with (OUT / "RIGHT_ARM_PRINT_MANIFEST.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def write_torque_chain(manifest: dict[str, object]) -> list[dict[str, object]]:
    rows = []
    for joint_id in RIGHT_ARM_JOINTS:
        spec = next(row for row in manifest["joint_specs"] if row["id"] == joint_id)
        components = [row for row in manifest["components"] if str(row["component_id"]).startswith(f"{joint_id}_")]
        servo = next(row for row in components if row["role"] == "purchased_exact_sts3250")
        bridge = next(row for row in components if row["role"] == "sts3250_pcd14_output_bridge_to_child")
        ring = next(row for row in components if row["role"] == "sts3250_parent_axis_and_thrust_mate")
        rows.append(
            {
                "joint_id": joint_id,
                "joint_name": spec["name"],
                "parent_link": spec["parent"],
                "child_link": spec["child"],
                "axis": json.dumps(spec["axis"]),
                "lower_rad": spec["limits"][0],
                "upper_rad": spec["limits"][1],
                "servo_case_fixed_to": servo["owner_link"],
                "output_bridge_fixed_to": bridge["owner_link"],
                "thrust_ring_fixed_to": ring["owner_link"],
                "servo_component": servo["component_id"],
                "bridge_component": bridge["component_id"],
                "mounting_note": servo["notes"],
            }
        )
    fields = list(rows[0])
    with (OUT / "RIGHT_ARM_TORQUE_CHAIN.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return rows


def write_test_checklist() -> None:
    rows = [
        ("G00", "printed parts", "FC01 actual STS3250 case/4xM2/shaft fit recorded", "HOLD"),
        ("G01", "printed parts", "RA01-RA03 deburred; no cracks, warp or support residue on servo/axis faces", "HOLD"),
        ("G02", "torque path", "three final metal PCD14 bridges and three thrust rings fitted; FDM samples removed", "HOLD"),
        ("G03", "mechanical", "S03 housing rigidly fixed to actual torso or metal bench anchor; arm is not hand-held", "HOLD"),
        ("G04", "mechanical", "all three joints manually sweep through manifest limits with no hard contact", "HOLD"),
        ("G05", "wiring", "TTL/power polarity checked; cable service loops clear every shear plane", "HOLD"),
        ("G06", "power", "12 V current-limited bench supply, 5 A fuse and physical emergency cut-off installed", "HOLD"),
        ("G07", "single servo", "one servo at a time: readback only, then +/-5 deg and +/-15 deg slow jog passes", "HOLD"),
        ("G08", "arm chain", "three servos connected, one commanded at a time, total supply limited to 3 A", "HOLD"),
        ("G09", "telemetry", "position/current/voltage/temperature logged; no reset, abnormal current, click or heating", "HOLD"),
    ]
    with (OUT / "RIGHT_ARM_POWER_TEST_CHECKLIST.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("gate_id", "category", "acceptance", "initial_status"))
        writer.writerows(rows)


def write_checksums() -> None:
    files = sorted(path for path in OUT.rglob("*") if path.is_file() and path.name != "SHA256SUMS.csv")
    with (OUT / "SHA256SUMS.csv").open("w", newline="", encoding="utf-8-sig") as stream:
        writer = csv.writer(stream)
        writer.writerow(("path", "bytes", "sha256"))
        for path in files:
            writer.writerow((path.relative_to(OUT).as_posix(), path.stat().st_size, hashlib.sha256(path.read_bytes()).hexdigest()))


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    reset_generated_part_subsets()
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    print_manifest = load_print_manifest()
    rows = [copy_print_part(item, print_manifest[item["release_name"]], "required_arm") for item in ARM_PRINT_PARTS]
    rows.append(copy_print_part(OPTIONAL_WRIST_SUPPORT, print_manifest[OPTIONAL_WRIST_SUPPORT["release_name"]], "optional_unpowered_wrist_support"))
    rows.append(copy_print_part(OPTIONAL_ANCHOR, print_manifest[OPTIONAL_ANCHOR["release_name"]], "optional_real_torso_anchor"))
    fit_checks = copy_fit_checks()
    digital_gate = export_coloured_arm_reference(manifest)
    torque_chain = write_torque_chain(manifest)
    write_print_manifest(rows)
    write_test_checklist()
    required_rows = [row for row in rows if row["category"] == "required_arm"]
    overall = (
        len(required_rows) == 3
        and digital_gate["counts_by_role"].get("purchased_exact_sts3250") == 3
        and digital_gate["counts_by_role"].get("sts3250_pcd14_output_bridge_to_child") == 3
        and digital_gate["counts_by_role"].get("sts3250_parent_axis_and_thrust_mate") == 3
        and digital_gate["reimported_step_brep_valid"]
        and digital_gate["glb_export_ok"]
        and all(row["source_brep_valid"] and row["source_solid_count"] > 0 for row in digital_gate["source_checks"])
    )
    payload = {
        "schema": "zeroth01.v5.right_arm_first_article.v1",
        "status": "ARM_PRINT_SUBSET_READY_POWERED_TEST_HOLD" if overall else "HOLD",
        "source_manifest": rel(MANIFEST),
        "printer_geometry_policy": {
            "same_cad_and_3mf_for_x2d_and_p2s": True,
            "machine_specific_bambu_studio_profile_required": True,
            "machine_specific_gcode_included": False,
            "reason": "printer kinematics/process compensation differ; nominal robot geometry must not fork by printer",
        },
        "required_printed_parts": required_rows,
        "optional_unpowered_wrist_support": next(row for row in rows if row["category"] == "optional_unpowered_wrist_support"),
        "optional_real_torso_anchor": next(row for row in rows if row["category"] == "optional_real_torso_anchor"),
        "fit_check_only": fit_checks,
        "torque_chain": torque_chain,
        "digital_reference_gate": digital_gate,
        "mass_summary": {
            "required_arm_solid_material_upper_bound_g": round(sum(float(row["solid_material_upper_bound_g"]) for row in required_rows), 3),
            "scope": "solid PA6-CF volume upper bound, not sliced mass; excludes servos, metal interfaces, fasteners and anchor",
        },
        "powered_test_release": "HOLD_UNTIL_G00_TO_G09_ARE_RECORDED_PASS",
        "truth_boundary": "This package identifies and validates the digital right-arm subset. It cannot certify printer shrinkage, purchased-servo tolerances, fastener engagement, wiring, or powered motion before the physical checklist passes.",
        "overall": "PASS" if overall else "FAIL",
    }
    (OUT / "RIGHT_ARM_RELEASE.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    write_checksums()
    print(json.dumps({"out": str(OUT), "overall": payload["overall"], "required_printed_parts": len(required_rows), "assembly_components": digital_gate["selected_component_count"]}, ensure_ascii=False, indent=2), flush=True)
    return 0 if overall else 2


if __name__ == "__main__":
    raise SystemExit(main())
