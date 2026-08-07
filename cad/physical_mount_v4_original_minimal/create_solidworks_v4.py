"""Create the portable native SolidWorks v4 original-minimal assembly."""

from __future__ import annotations

import argparse
import importlib.util
import json
import shutil
from pathlib import Path

import pythoncom


ROOT = Path(__file__).resolve().parents[2]
V3_SCRIPT = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "create_solidworks_v3.py"
MANIFEST = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v4_original_minimal"
    / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
)
V3_PORTABLE = ROOT / "generated" / "solidworks" / "physical_mount_v3_rl_fixed" / "portable_flat"
SW_ROOT = ROOT / "generated" / "solidworks" / "physical_mount_v4_original_minimal"
PORTABLE = SW_ROOT / "portable_flat"
NORMAL_ASM = PORTABLE / "OPEN_FIRST_ZEROTH01_V4_ORIGINAL_MINIMAL_WHITE_18_BLUE_STS3250.SLDASM"
XRAY_ASM = PORTABLE / "OPTIONAL_XRAY_ZEROTH01_V4_ORIGINAL_MINIMAL_INTERNAL_LAYOUT.SLDASM"
REPORT_ROOT = ROOT / "reports" / "v4_original_minimal"
SNAPSHOT_ROOT = ROOT / "snapshots" / "solidworks" / "v4_original_minimal"
FULL_ASSEMBLY_STEP = (
    ROOT
    / "generated"
    / "cad"
    / "physical_mount_v4_original_minimal"
    / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY.step"
)


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v3 = load(V3_SCRIPT, "zeroth_v3_solidworks_reused_by_v4")


def data() -> dict[str, object]:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))


def safe(value: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in value)


def is_v4_source(row: dict[str, object]) -> bool:
    return "/physical_mount_v4_original_minimal/parts/" in f"/{str(row['source']).replace(chr(92), '/')}"


def v4_part_path(source: str) -> Path:
    return PORTABLE / f"ZEROTH01_V4_{safe(Path(source).stem).upper()}.SLDPRT"


def v3_part_path(source: str) -> Path:
    return PORTABLE / f"ZEROTH01_V3_{safe(Path(source).stem).upper()}.SLDPRT"


def source_native_name(row: dict[str, object]) -> str:
    source_stem = Path(str(row["source"])).stem
    if str(row["role"]) == "source_load_bearing_carrier":
        if source_stem.endswith("WRIST_TRIMMED"):
            return f"ZEROTH01_V2_MINIMAL_{safe(source_stem).upper()}.SLDPRT"
        return f"ZEROTH01_PHYSICAL_MOUNT_V1_{safe(source_stem).upper()}_CARRIER.SLDPRT"
    return f"ZEROTH01_V3_{safe(source_stem).upper()}.SLDPRT"


def native_part_for(row: dict[str, object]) -> Path:
    source = str(row["source"]).replace("\\", "/")
    if is_v4_source(row):
        return v4_part_path(source)
    return PORTABLE / f"V4_REUSED_{source_native_name(row)}"


def unique_new_sources() -> dict[str, str]:
    return {
        Path(str(row["source"])).stem: str(row["source"])
        for row in data()["components"]
        if is_v4_source(row)
    }


def prepare_portable() -> dict[str, object]:
    PORTABLE.mkdir(parents=True, exist_ok=True)
    copied = []
    for row in data()["components"]:
        if is_v4_source(row):
            continue
        target = native_part_for(row)
        if target.name in copied:
            continue
        source = V3_PORTABLE / source_native_name(row)
        if not source.is_file():
            raise FileNotFoundError(source)
        if not target.is_file() or target.stat().st_mtime < source.stat().st_mtime:
            shutil.copy2(source, target)
        copied.append(target.name)
    shutil.copy2(MANIFEST, PORTABLE / MANIFEST.name)
    return {"copied_or_reused_v1_v2_v3_native_parts": len(copied), "status": "PASS"}


def configure_v3_module() -> None:
    v3.MANIFEST = MANIFEST
    v3.V2_PORTABLE = V3_PORTABLE
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
    v3.is_new_v3_part = is_v4_source
    v3.v3_part_path = v4_part_path
    v3.unique_new_sources = unique_new_sources
    v3.native_part_for = native_part_for
    v3.prepare_portable = prepare_portable


def normalize_gate_schema() -> None:
    """Label the reused v3 SolidWorks automation report as a v4 result."""

    gate = REPORT_ROOT / "solidworks_gate.json"
    payload = json.loads(gate.read_text(encoding="utf-8"))
    payload["schema"] = "zeroth01.physical_mount_v4_original_minimal.solidworks_gate.v1"
    payload["truth_boundary"] = (
        "Native SolidWorks save, component count, transform and assembly-envelope gate. "
        "Use solidworks_interference_gate.json for cross-component B-Rep interference."
    )
    gate.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


INTERNAL_ROLES = {
    "internal_payload_controlled_envelope",
    "purchased_internal_interaction_module",
    "removable_internal_service_mount",
    "rigid_sensor_mount",
    "harness_strain_relief",
    "direct_head_torso_mount",
}


def sw_data() -> dict[str, object]:
    payload = data()
    for row in payload["components"]:
        if str(row["role"]) in INTERNAL_ROLES:
            row["role"] = "internal_payload_controlled_envelope"
    return payload


def restyle_and_save_views(startup_timeout: float) -> None:
    """Save a six-view evidence grid from the already styled assemblies.

    ``assemble`` is the source of truth for visibility: its v4 manifest maps
    every service-only component to the hidden payload role in the normal
    assembly.  Reopening an assembly and trying to rediscover occurrences by
    COM name proved brittle when another Zeroth assembly was already open, so
    this command deliberately does not mutate occurrence visibility.
    """
    configure_v3_module()
    pythoncom.CoInitialize()
    sw = v3.v1.typed_sldworks(v3.get_sw(startup_timeout))
    view_specs = (
        ("front", 1, "*Front"),
        ("back", 2, "*Back"),
        ("left", 3, "*Left"),
        ("right", 4, "*Right"),
        ("top", 5, "*Top"),
        ("iso", 7, "*Isometric"),
    )
    for assembly_path, prefix in ((NORMAL_ASM, "normal"), (XRAY_ASM, "xray")):
        result = sw.OpenDoc6(str(assembly_path), v3.SW_DOC_ASSEMBLY, 1, "", 0, 0)
        raw = v3.v1.first(result)
        if raw is None:
            raise RuntimeError(f"cannot open {assembly_path}")
        model = v3.v1.base.as_model_doc(raw)
        for label, view_id, view_name in view_specs:
            v3.v1.save_view(
                model,
                view_id,
                SNAPSHOT_ROOT / f"v4_{prefix}_{label}.png",
                view_name=view_name,
            )
    print(json.dumps({"normal": str(NORMAL_ASM), "xray": str(XRAY_ASM), "views": len(view_specs) * 2}, indent=2))


def save_rl_review_views(startup_timeout: float) -> None:
    """Save upright RL-convention views without altering assembly transforms."""

    configure_v3_module()
    pythoncom.CoInitialize()
    sw = v3.v1.typed_sldworks(v3.get_sw(startup_timeout))
    outputs = []
    for assembly_path, prefix in ((NORMAL_ASM, "normal"), (XRAY_ASM, "xray")):
        result = sw.OpenDoc6(str(assembly_path), v3.SW_DOC_ASSEMBLY, 1, "", 0, 0)
        raw = v3.v1.first(result)
        if raw is None:
            raise RuntimeError(f"cannot open {assembly_path}")
        model = v3.v1.base.as_model_doc(raw)
        front = SNAPSHOT_ROOT / f"v4_{prefix}_rl_front_upright.png"
        rear = SNAPSHOT_ROOT / f"v4_{prefix}_rl_rear_upright.png"
        v3.v1.save_view(model, 4, front, view_name="*Right", rotate_degrees=-90.0)
        v3.v1.save_view(model, 3, rear, view_name="*Left", rotate_degrees=90.0)
        outputs.extend((str(front), str(rear)))
    print(json.dumps({"outputs": outputs}, indent=2))


def export_full_assembly_step(startup_timeout: float) -> None:
    """Export the SolidWorks-gated normal assembly as one portable STEP."""

    configure_v3_module()
    pythoncom.CoInitialize()
    sw = v3.v1.typed_sldworks(v3.get_sw(startup_timeout))
    result = sw.OpenDoc6(str(NORMAL_ASM), v3.SW_DOC_ASSEMBLY, 1, "", 0, 0)
    raw = v3.v1.first(result)
    if raw is None:
        raise RuntimeError(f"cannot open {NORMAL_ASM}")
    model = v3.v1.base.as_model_doc(raw)
    FULL_ASSEMBLY_STEP.parent.mkdir(parents=True, exist_ok=True)
    save_code = int(model.SaveAs3(str(FULL_ASSEMBLY_STEP), 0, 1))
    if not FULL_ASSEMBLY_STEP.is_file() or FULL_ASSEMBLY_STEP.stat().st_size < 1024:
        raise RuntimeError(f"SolidWorks STEP export failed with code {save_code}")
    print(json.dumps({"step": str(FULL_ASSEMBLY_STEP), "save_code": save_code, "bytes": FULL_ASSEMBLY_STEP.stat().st_size}, indent=2))


def import_all(force: bool, startup_timeout: float) -> None:
    configure_v3_module()
    prepare_portable()
    sources = unique_new_sources()
    for index, key in enumerate(sorted(sources), start=1):
        print(f"v4 STEP import {index}/{len(sources)}: {key}", flush=True)
        v3.import_new(key, force, startup_timeout)


def main() -> None:
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("list-new")
    sub.add_parser("prepare")
    sub.add_parser("normalize-report")
    imp = sub.add_parser("import-all")
    imp.add_argument("--force", action="store_true")
    imp.add_argument("--startup-timeout", type=float, default=45.0)
    selected = sub.add_parser("import-keys")
    selected.add_argument("keys", nargs="+")
    selected.add_argument("--force", action="store_true")
    selected.add_argument("--startup-timeout", type=float, default=45.0)
    asm = sub.add_parser("assemble")
    asm.add_argument("--startup-timeout", type=float, default=45.0)
    style = sub.add_parser("restyle")
    style.add_argument("--startup-timeout", type=float, default=45.0)
    review = sub.add_parser("rl-review")
    review.add_argument("--startup-timeout", type=float, default=45.0)
    export_step = sub.add_parser("export-step")
    export_step.add_argument("--startup-timeout", type=float, default=45.0)
    args = parser.parse_args()
    configure_v3_module()
    if args.command == "list-new":
        print("\n".join(sorted(unique_new_sources())))
    elif args.command == "prepare":
        print(json.dumps(prepare_portable(), indent=2))
    elif args.command == "normalize-report":
        normalize_gate_schema()
        print(REPORT_ROOT / "solidworks_gate.json")
    elif args.command == "import-all":
        import_all(args.force, args.startup_timeout)
    elif args.command == "import-keys":
        prepare_portable()
        available = unique_new_sources()
        for key in args.keys:
            if key not in available:
                raise KeyError(f"unknown v4 source {key}; choose {sorted(available)}")
            v3.import_new(key, args.force, args.startup_timeout)
    elif args.command == "assemble":
        v3.data = sw_data
        v3.assemble(args.startup_timeout)
        normalize_gate_schema()
    elif args.command == "restyle":
        restyle_and_save_views(args.startup_timeout)
    elif args.command == "rl-review":
        save_rl_review_views(args.startup_timeout)
    elif args.command == "export-step":
        export_full_assembly_step(args.startup_timeout)


if __name__ == "__main__":
    main()
