from __future__ import annotations

import argparse
import csv
import gc
import importlib.util
import json
import math
import subprocess
import sys
import time
import traceback
from collections import defaultdict
from pathlib import Path

import pythoncom
import win32com.client as win32
import win32com.client.dynamic as win32_dynamic
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
BASE_SCRIPT = ROOT / "scripts" / "create_solidworks_kinematic_review.py"
BUILD_SCRIPT = Path(__file__).with_name("build_physical_mount_v1.py")
CAD_SITE_PACKAGES = (
    ROOT.parents[1] / ".venv-cad" / "Lib" / "site-packages"
)
if str(CAD_SITE_PACKAGES) not in sys.path:
    sys.path.insert(0, str(CAD_SITE_PACKAGES))


def _load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = _load_module("zeroth01_sw_kinematic_base", BASE_SCRIPT)
physical = _load_module("zeroth01_physical_mount_build", BUILD_SCRIPT)

URDF_PATH = physical.URDF_PATH
MANIFEST_PATH = physical.MANIFEST_PATH
SKELETON_DIR = physical.SKELETON_DIR
SERVO_DIR = physical.SERVO_DIR
STEP_SOURCE_ROOT = (
    ROOT / "generated" / "cad" / "physical_mount_v1" / "step"
)
SW_ROOT = ROOT / "generated" / "solidworks" / "physical_mount_v1"
PART_ROOT = SW_ROOT / "parts"
SKELETON_PART_ROOT = PART_ROOT / "skeleton"
SERVO_PART_ROOT = PART_ROOT / "servos"
LINK_ASM_ROOT = SW_ROOT / "links"
TOP_ASM = SW_ROOT / "OPEN_FIRST_ZEROTH01_PHYSICAL_MOUNT_V1_16_BLUE_SERVOS.SLDASM"
XRAY_ASM = SW_ROOT / "ZEROTH01_PHYSICAL_MOUNT_V1_16_BLUE_SERVOS_XRAY.SLDASM"
SNAPSHOT_ROOT = ROOT / "snapshots" / "solidworks" / "physical_mount_v1"
FRAME_ROOT = SNAPSHOT_ROOT / "motion_frames"
MOTION_GIF = SNAPSHOT_ROOT / "solidworks_physical_mount_v1_16dof_motion.gif"
REPORT_ROOT = ROOT / "reports" / "physical_mount_v1"
TRACE_LOG = REPORT_ROOT / "solidworks_physical_mount_trace.log"
PART_REPORT = REPORT_ROOT / "solidworks_physical_mount_parts.csv"
LINK_REPORT = REPORT_ROOT / "solidworks_physical_mount_link_subassemblies.csv"
COMPONENT_REPORT = REPORT_ROOT / "solidworks_physical_mount_components.csv"
GATE_REPORT = REPORT_ROOT / "solidworks_physical_mount_gate.json"

SW_DOC_PART = 1
SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1
SW_SAVE_AS_CURRENT_VERSION = 0
SW_SAVE_AS_SILENT = 1
SW_IMPORT_STL_VRML_MODEL_TYPE = 208
SW_IMPORT_STL_VRML_UNITS = 210
SW_IMPORT_AS_SURFACE = 1
SW_LENGTH_UNIT_METER = 2
SW_EXE = Path(r"E:\SolidWorks\SOLIDWORKS\SLDWORKS.exe")
SW_WARMUP_PART = (
    SKELETON_PART_ROOT
    / "ZEROTH01_PHYSICAL_MOUNT_V1_3215_1FLANGE_CARRIER.SLDPRT"
)
WHITE = (0.93, 0.95, 0.98)
BLUE = (0.086, 0.467, 1.0)


def trace(message: str) -> None:
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with TRACE_LOG.open("a", encoding="utf-8") as stream:
        stream.write(f"{stamp} {message}\n")
    print(message, flush=True)


def typed_sldworks(sw):
    module = win32.gencache.EnsureModule(
        base.SW_MAIN_TYPELIB,
        0,
        33,
        0,
    )
    return module.ISldWorks(sw._oleobj_)


def iter_sw_apps():
    """Yield SolidWorks sessions registered in the Running Object Table.

    GetActiveObject/Dispatch can block indefinitely while SolidWorks is
    starting. Enumerating the ROT keeps startup observable and lets this
    script enforce its own timeout.
    """
    rot = pythoncom.GetRunningObjectTable()
    enum = rot.EnumRunning()
    context = pythoncom.CreateBindCtx(0)
    while True:
        monikers = enum.Next(1)
        if not monikers:
            break
        moniker = monikers[0]
        try:
            name = moniker.GetDisplayName(context, None)
        except Exception:
            continue
        if not name.startswith("SolidWorks_PID_"):
            continue
        try:
            # Force a no-type-info wrapper.  Once the SolidWorks makepy
            # modules are loaded, both Dispatch and GetActiveObject select
            # an early-bound wrapper whose ActiveDoc is null for a
            # just-opened STL in SW 2025.  DumbDispatch preserves the ROT
            # object's working late-bound behavior.
            obj = pythoncom.GetActiveObject("SldWorks.Application")
            app = win32_dynamic.DumbDispatch(
                obj.QueryInterface(pythoncom.IID_IDispatch),
                userName=name,
            )
        except Exception:
            continue
        yield name, app


def get_or_start_sw(
    timeout_s: float = 240.0,
    startup_file: Path | None = None,
):
    apps = list(iter_sw_apps())
    if apps:
        trace(f"SolidWorks ROT attach: {apps[0][0]}")
        sw = apps[0][1]
        sw.Visible = True
        sw.CommandInProgress = False
        return sw

    if not SW_EXE.is_file():
        raise FileNotFoundError(SW_EXE)
    trace(f"SolidWorks launch requested: {SW_EXE}")
    launch_command = [str(SW_EXE)]
    selected_startup_file = (
        startup_file
        if startup_file is not None and startup_file.is_file()
        else SW_WARMUP_PART
    )
    if selected_startup_file.is_file():
        launch_command.append(str(selected_startup_file))
    process = subprocess.Popen(
        launch_command,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        close_fds=True,
        creationflags=subprocess.CREATE_NEW_PROCESS_GROUP,
    )
    trace(f"SolidWorks spawned pid={process.pid}")
    deadline = time.monotonic() + timeout_s
    next_status = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        apps = list(iter_sw_apps())
        if apps:
            trace(
                f"SolidWorks ROT ready: {apps[0][0]} "
                f"after_pid={process.pid}"
            )
            sw = apps[0][1]
            sw.Visible = True
            sw.CommandInProgress = False
            return sw
        return_code = process.poll()
        if return_code is not None:
            raise RuntimeError(
                "SolidWorks exited before COM registration: "
                f"pid={process.pid} code={return_code}"
            )
        if time.monotonic() >= next_status:
            trace(
                "SolidWorks startup pending: "
                f"pid={process.pid} elapsed_s="
                f"{timeout_s - (deadline - time.monotonic()):.1f}"
            )
            next_status = time.monotonic() + 5.0
        time.sleep(0.5)

    trace(
        f"SolidWorks startup timeout: pid={process.pid} "
        f"timeout_s={timeout_s:.1f}"
    )
    process.terminate()
    try:
        process.wait(timeout=8.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=8.0)
    raise TimeoutError(
        f"SolidWorks did not register COM within {timeout_s:.1f}s"
    )


def safe_name(value: str) -> str:
    return "".join(
        character if character.isalnum() else "_"
        for character in value
    )


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    rows = rows or [{"status": "EMPTY"}]
    fields: list[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=fields,
            extrasaction="ignore",
        )
        writer.writeheader()
        writer.writerows(rows)


def first(value):
    return value[0] if isinstance(value, tuple) else value


def byref_i4(value: int = 0):
    return win32.VARIANT(
        pythoncom.VT_BYREF | pythoncom.VT_I4,
        int(value),
    )


def close_task_documents(sw) -> None:
    output_root = str(SW_ROOT.resolve()).lower()
    for raw in list(base.call(sw, "GetDocuments", []) or []):
        try:
            model = base.as_model_doc(raw)
            path = str(base.call(model, "GetPathName", "")).lower()
            if path and path.startswith(output_root):
                sw.CloseDoc(str(base.call(model, "GetTitle", "")))
        except Exception:
            continue


def skeleton_part_path(link: str) -> Path:
    return SKELETON_PART_ROOT / (
        f"ZEROTH01_PHYSICAL_MOUNT_V1_{safe_name(link).upper()}_CARRIER.SLDPRT"
    )


def servo_part_path(servo: dict[str, object]) -> Path:
    return SERVO_PART_ROOT / (
        f"{servo['id']}_INSTALLED_STS3215_FAMILY_REFERENCE_"
        f"{safe_name(str(servo['joint'])).upper()}.SLDPRT"
    )


def link_assembly_path(link: str) -> Path:
    return LINK_ASM_ROOT / (
        f"ZEROTH01_PHYSICAL_MOUNT_V1_LINK_{safe_name(link).upper()}.SLDASM"
    )


def part_box(sw, path: Path) -> list[float]:
    errors = byref_i4()
    warnings = byref_i4()
    result = sw.OpenDoc6(
        str(path),
        SW_DOC_PART,
        SW_OPEN_SILENT,
        "",
        errors,
        warnings,
    )
    model = first(result)
    if model is None:
        raise RuntimeError(f"cannot open generated part {path}")
    try:
        return [
            float(value)
            for value in base.as_part_doc(model).GetPartBox(True)
        ]
    finally:
        base.close_document(sw, model)


def active_document(sw):
    # Keep the late-bound document wrapper here.  The strongly-typed
    # IModelDoc2 wrapper generated by makepy reports GetTitle/GetPathName
    # inconsistently for a just-opened STL (empty strings in SW 2025), while
    # the late-bound wrapper already exposes the correct import document.
    raw = base.call(sw, "ActiveDoc", None)
    if raw is not None:
        return raw
    # SW 2025's generated application wrapper can return null for ActiveDoc
    # while GetDocuments already contains the imported mesh.  A cold import
    # is guaranteed to own a single document, so the last document is the
    # correct deterministic fallback.
    documents = list(base.call(sw, "GetDocuments", []) or [])
    return documents[-1] if documents else None


def document_title(model) -> str:
    return str(base.call(model, "GetTitle", ""))


def document_path(model) -> Path | None:
    path = str(base.call(model, "GetPathName", ""))
    return Path(path).resolve() if path else None


def close_exact_document_if_open(sw, target: Path) -> None:
    if sw is None:
        return
    target = target.resolve()
    for raw in list(base.call(sw, "GetDocuments", []) or []):
        try:
            model = base.as_model_doc(raw)
            if document_path(model) == target:
                base.close_document(sw, model)
        except Exception:
            continue


def wait_for_interactive_import(
    source: Path,
    timeout_s: float = 120.0,
):
    deadline = time.monotonic() + timeout_s
    next_status = time.monotonic() + 5.0
    source_token = source.stem.lower()
    last_title = ""
    last_path = ""
    while time.monotonic() < deadline:
        for name, candidate_sw in iter_sw_apps():
            model = active_document(candidate_sw)
            if model is None:
                continue
            last_title = document_title(model)
            path = document_path(model)
            last_path = str(path or "")
            haystack = f"{last_title} {last_path}".lower()
            if source_token in haystack:
                return name, candidate_sw, model
        if time.monotonic() >= next_status:
            trace(
                f"interactive STL import pending: {source.name} "
                f"last_title={last_title!r}"
            )
            next_status = time.monotonic() + 5.0
        time.sleep(0.5)
    raise TimeoutError(
        f"interactive STL import timed out: {source}; "
        f"last_title={last_title!r}; last_path={last_path!r}"
    )


def import_stl_part(
    sw,
    source: Path,
    target: Path,
    *,
    rebuild: bool,
) -> dict[str, object]:
    if not source.is_file():
        raise FileNotFoundError(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    start = time.time()
    method = "reuse_existing_interactive_stl_surface_part"
    load_error = 0
    bounds: list[float] | None = None
    if rebuild or not target.is_file() or target.stat().st_size < 1024:
        close_exact_document_if_open(sw, target)
        owner_name = ""
        owner_sw = None
        model = None
        source_token = source.stem.lower()
        for name, candidate_sw in iter_sw_apps():
            candidate_model = active_document(candidate_sw)
            if candidate_model is None:
                continue
            title = document_title(candidate_model)
            path = str(document_path(candidate_model) or "")
            trace(
                f"active import candidate: session={name} "
                f"title={title!r} path={path!r}"
            )
            if source_token in f"{title} {path}".lower():
                owner_name = name
                owner_sw = candidate_sw
                model = candidate_model
                break
        if model is None:
            existing = list(iter_sw_apps())
            if existing:
                raise RuntimeError(
                    "Cold interactive STL import requires no unrelated "
                    "SolidWorks session; active_sessions="
                    + ",".join(name for name, _app in existing)
                )
            subprocess.run(
                [
                    "cmd.exe",
                    "/d",
                    "/c",
                    "start",
                    "",
                    str(SW_EXE),
                    str(source),
                ],
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                close_fds=True,
                timeout=10.0,
                check=True,
            )
            owner_name, owner_sw, model = wait_for_interactive_import(
                source
            )
        owner_sw.Visible = True
        owner_sw.UserControl = True
        owner_sw.CommandInProgress = False
        owner_sw.SetUserPreferenceIntegerValue(
            SW_IMPORT_STL_VRML_MODEL_TYPE,
            SW_IMPORT_AS_SURFACE,
        )
        owner_sw.SetUserPreferenceIntegerValue(
            SW_IMPORT_STL_VRML_UNITS,
            SW_LENGTH_UNIT_METER,
        )
        bounds = [
            float(value)
            for value in base.as_part_doc(model).GetPartBox(True)
        ]
        method = "cold_SLDWORKS_exe_interactive_STL_surface_metre"
        model.SaveAs3(
            str(target),
            SW_SAVE_AS_CURRENT_VERSION,
            SW_SAVE_AS_SILENT,
        )
        base.close_document(owner_sw, model)
        if not target.is_file() or target.stat().st_size < 1024:
            raise RuntimeError(f"SolidWorks part save failed: {target}")
        try:
            owner_sw.ExitApp()
        except Exception as error:
            active_names = {name for name, _app in iter_sw_apps()}
            if owner_name in active_names:
                raise
            trace(
                "SolidWorks per-file session already exited before "
                f"ExitApp acknowledgement: {type(error).__name__}"
            )
        exit_deadline = time.monotonic() + 20.0
        while time.monotonic() < exit_deadline:
            if owner_name not in {
                name for name, _app in iter_sw_apps()
            }:
                break
            time.sleep(0.5)
        else:
            process_id = int(owner_name.rsplit("_", 1)[-1])
            trace(
                f"SolidWorks per-file exit timeout; terminating "
                f"task-owned pid={process_id}"
            )
            subprocess.run(
                ["taskkill", "/PID", str(process_id), "/F"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                check=False,
                timeout=10.0,
            )
    else:
        bounds = part_box(sw, target)
    extents = [None, None, None]
    if bounds is not None and len(bounds) == 6:
        extents = [
            bounds[3] - bounds[0],
            bounds[4] - bounds[1],
            bounds[5] - bounds[2],
        ]
    maximum_extent = max(
        (float(value) for value in extents if value is not None),
        default=0.0,
    )
    scale_gate = "PASS" if 0.01 <= maximum_extent <= 0.50 else "FAIL"
    return {
        "source": str(source),
        "part": str(target),
        "method": method,
        "load_error": load_error,
        "extent_x_m": extents[0],
        "extent_y_m": extents[1],
        "extent_z_m": extents[2],
        "bytes": target.stat().st_size,
        "seconds": time.time() - start,
        "scale_gate": scale_gate,
        "status": scale_gate,
    }


def set_material(
    component,
    color: tuple[float, float, float],
    transparency: float = 0.0,
) -> bool:
    payload = win32.VARIANT(
        pythoncom.VT_ARRAY | pythoncom.VT_R8,
        [
            color[0],
            color[1],
            color[2],
            0.35,
            0.75,
            0.25,
            0.35,
            max(0.0, min(1.0, transparency)),
            0.0,
        ],
    )
    for setter in (
        lambda: setattr(component, "MaterialPropertyValues", payload),
        lambda: component.SetMaterialPropertyValues2(payload, 1, None),
    ):
        try:
            setter()
            return True
        except Exception:
            continue
    return False


def add_component(
    sw,
    model,
    assembly,
    source: Path,
    document_type: int,
    name: str,
    transform,
):
    result = sw.OpenDoc6(
        str(source),
        document_type,
        SW_OPEN_SILENT,
        "",
        0,
        0,
    )
    opened = first(result)
    if opened is None:
        raise RuntimeError(f"cannot preload component {source}")
    base.activate_document(sw, model)
    component = assembly.AddComponent5(
        str(source),
        0,
        "",
        False,
        "",
        0.0,
        0.0,
        0.0,
    )
    if component is None:
        base.close_document(sw, opened)
        raise RuntimeError(f"AddComponent5 failed: {source}")
    try:
        component.Name2 = name
    except Exception:
        pass
    error = base.set_component_transform(sw, component, transform)
    base.close_document(sw, opened)
    base.activate_document(sw, model)
    return component, error


def fix_component(model, assembly, component) -> bool:
    try:
        model.ClearSelection2(True)
        component.Select4(False, None, False)
        assembly.FixComponent()
        model.ClearSelection2(True)
        return True
    except Exception:
        return False


def load_model():
    root = physical.ET.parse(URDF_PATH).getroot()
    base_link, joints = physical._load_kinematic_model(root)
    skeleton_meshes = physical._source_link_meshes(root)
    manifest = json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))["servos"]
    servos_by_link: dict[str, list[dict[str, object]]] = defaultdict(list)
    for servo in manifest:
        servos_by_link[str(servo["owning_link"])].append(servo)
    return root, base_link, joints, skeleton_meshes, manifest, servos_by_link


def import_all_parts(
    sw,
    skeleton_meshes: dict[str, str],
    manifest: list[dict[str, object]],
    *,
    rebuild: bool,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    # Import a small alphabetical carrier first so a translator failure is
    # detected before the high-facet torso/arm files consume time.
    ordered_links = sorted(skeleton_meshes)
    total = len(ordered_links) + len(manifest)
    index = 0
    for link in ordered_links:
        index += 1
        trace(f"part import {index}/{total}: carrier {link}")
        source = SKELETON_DIR / skeleton_meshes[link]
        target = skeleton_part_path(link)
        needs_cold_import = (
            rebuild or not target.is_file() or target.stat().st_size < 1024
        )
        if needs_cold_import and sw is not None:
            close_task_documents(sw)
            try:
                sw.ExitApp()
            except Exception:
                pass
            deadline = time.monotonic() + 20.0
            while list(iter_sw_apps()) and time.monotonic() < deadline:
                time.sleep(0.5)
            if list(iter_sw_apps()):
                raise RuntimeError(
                    "Could not close the task SolidWorks session before "
                    f"cold import of {source}"
                )
            sw = None
        if not needs_cold_import and sw is None:
            sw = get_or_start_sw()
        row = import_stl_part(
            sw,
            source,
            target,
            rebuild=rebuild,
        )
        if needs_cold_import:
            sw = None
        row.update({"kind": "carrier", "link": link})
        rows.append(row)
    for servo in manifest:
        index += 1
        source = ROOT / str(servo["output_mesh"])
        trace(f"part import {index}/{total}: {servo['id']} {servo['joint']}")
        target = servo_part_path(servo)
        needs_cold_import = (
            rebuild or not target.is_file() or target.stat().st_size < 1024
        )
        if needs_cold_import and sw is not None:
            close_task_documents(sw)
            try:
                sw.ExitApp()
            except Exception:
                pass
            deadline = time.monotonic() + 20.0
            while list(iter_sw_apps()) and time.monotonic() < deadline:
                time.sleep(0.5)
            if list(iter_sw_apps()):
                raise RuntimeError(
                    "Could not close the task SolidWorks session before "
                    f"cold import of {source}"
                )
            sw = None
        if not needs_cold_import and sw is None:
            sw = get_or_start_sw()
        row = import_stl_part(
            sw,
            source,
            target,
            rebuild=rebuild,
        )
        if needs_cold_import:
            sw = None
        row.update(
            {
                "kind": "servo",
                "id": servo["id"],
                "joint": servo["joint"],
                "owning_link": servo["owning_link"],
            }
        )
        rows.append(row)
    return rows


def create_link_subassemblies(
    sw,
    skeleton_meshes: dict[str, str],
    servos_by_link: dict[str, list[dict[str, object]]],
    *,
    rebuild: bool,
) -> list[dict[str, object]]:
    LINK_ASM_ROOT.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    ordered_links = sorted(
        skeleton_meshes,
        key=lambda link: (
            0 if link == "Z_BOT2_MASTER_BODY_SKELETON" else 1,
            link,
        ),
    )
    identity = (physical.IDENTITY_R, (0.0, 0.0, 0.0))
    for index, link in enumerate(ordered_links, start=1):
        target = link_assembly_path(link)
        if not rebuild and target.is_file() and target.stat().st_size > 1024:
            rows.append(
                {
                    "link": link,
                    "subassembly": str(target),
                    "servo_ids": " ".join(
                        str(servo["id"])
                        for servo in servos_by_link.get(link, [])
                    ),
                    "method": "reuse_existing_link_subassembly",
                    "status": "PASS",
                }
            )
            continue
        trace(
            f"link subassembly {index}/{len(ordered_links)}: {link} "
            f"servos={len(servos_by_link.get(link, []))}"
        )
        raw = first(
            sw.NewDocument(
                str(base.ASM_TEMPLATE),
                0,
                0,
                0,
            )
        )
        if raw is None:
            raise RuntimeError(f"NewDocument failed for link {link}")
        model = base.as_model_doc(raw)
        assembly = base.as_assembly_doc(raw)
        carrier, carrier_error = add_component(
            sw,
            model,
            assembly,
            skeleton_part_path(link),
            SW_DOC_PART,
            f"{safe_name(link)}_CARRIER_WHITE",
            identity,
        )
        set_material(carrier, WHITE, 0.0)
        carrier_fixed = fix_component(model, assembly, carrier)
        servo_errors: list[float] = []
        servo_fixed = True
        for servo in servos_by_link.get(link, []):
            component, error = add_component(
                sw,
                model,
                assembly,
                servo_part_path(servo),
                SW_DOC_PART,
                f"{servo['id']}_{safe_name(str(servo['joint']))}_"
                "BLUE_INSTALLED_STS3215_FAMILY_REFERENCE",
                identity,
            )
            set_material(component, BLUE, 0.0)
            servo_fixed = fix_component(model, assembly, component) and servo_fixed
            servo_errors.append(error)
        base.refresh_assembly_display(model)
        model.SaveAs3(
            str(target),
            SW_SAVE_AS_CURRENT_VERSION,
            SW_SAVE_AS_SILENT,
        )
        if not target.is_file() or target.stat().st_size < 1024:
            raise RuntimeError(f"link subassembly save failed: {target}")
        base.close_document(sw, model)
        rows.append(
            {
                "link": link,
                "subassembly": str(target),
                "servo_ids": " ".join(
                    str(servo["id"])
                    for servo in servos_by_link.get(link, [])
                ),
                "component_count": 1 + len(servos_by_link.get(link, [])),
                "carrier_fixed": carrier_fixed,
                "servo_components_fixed": servo_fixed,
                "max_identity_transform_error": max(
                    [carrier_error, *servo_errors],
                    default=carrier_error,
                ),
                "method": "fixed_carrier_plus_in_place_extracted_servo_components",
                "status": (
                    "PASS"
                    if carrier_fixed
                    and servo_fixed
                    and max([carrier_error, *servo_errors], default=carrier_error)
                    < 1e-8
                    else "FAIL"
                ),
            }
        )
    return rows


def save_view(
    model,
    view_id: int,
    path: Path,
    *,
    view_name: str = "",
    rotate_degrees: float = 0.0,
) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        base.refresh_assembly_display(model)
        model.ShowNamedView2(view_name, view_id)
        model.ViewZoomtofit2()
        model.GraphicsRedraw2()
    except Exception:
        pass
    model.SaveAs3(
        str(path),
        SW_SAVE_AS_CURRENT_VERSION,
        SW_SAVE_AS_SILENT,
    )
    if rotate_degrees and path.is_file():
        # The source robot uses +X as forward, while its authored vertical
        # direction appears 90 degrees clockwise in a SolidWorks Right view.
        # Normalize only the exported bitmap; component transforms and the
        # assembly/motion evidence remain untouched.
        with Image.open(path) as source_image:
            rotated_image = source_image.rotate(
                rotate_degrees,
                expand=True,
            )
            try:
                rotated_image.save(path)
            finally:
                rotated_image.close()
    return path.is_file() and path.stat().st_size > 0


def create_motion_gif(paths: list[Path]) -> bool:
    images = [
        Image.open(path).convert("P", palette=Image.Palette.ADAPTIVE)
        for path in paths
    ]
    if not images:
        return False
    try:
        images[0].save(
            MOTION_GIF,
            save_all=True,
            append_images=images[1:],
            duration=90,
            loop=0,
            optimize=False,
        )
    finally:
        for image in images:
            image.close()
    return MOTION_GIF.is_file() and MOTION_GIF.stat().st_size > 0


def create_top_assembly(
    sw,
    base_link: str,
    joints: list[dict[str, object]],
    skeleton_meshes: dict[str, str],
    frame_count: int,
) -> tuple[list[dict[str, object]], bool]:
    for path in (TOP_ASM, XRAY_ASM):
        try:
            sw.CloseDoc(path.name)
        except Exception:
            pass
    raw = first(sw.NewDocument(str(base.ASM_TEMPLATE), 0, 0, 0))
    if raw is None:
        raise RuntimeError("top-level SolidWorks NewDocument failed")
    model = base.as_model_doc(raw)
    assembly = base.as_assembly_doc(raw)
    neutral = physical._forward_kinematics(base_link, joints, {})
    components = {}
    rows: list[dict[str, object]] = []
    ordered_links = sorted(
        skeleton_meshes,
        key=lambda link: (
            0 if link == "Z_BOT2_MASTER_BODY_SKELETON" else 1,
            link,
        ),
    )
    for index, link in enumerate(ordered_links, start=1):
        trace(f"top assembly component {index}/{len(ordered_links)}: {link}")
        component, error = add_component(
            sw,
            model,
            assembly,
            link_assembly_path(link),
            SW_DOC_ASSEMBLY,
            f"ZEROTH01_LINK_{safe_name(link)}",
            neutral[link],
        )
        components[link] = component
        rows.append(
            {
                "link": link,
                "component": str(base.call(component, "Name2", "")),
                "subassembly": str(link_assembly_path(link)),
                "transform_error": error,
                "status": "PASS" if error < 1e-8 else "FAIL",
            }
        )
    base.refresh_assembly_display(model)
    model.SaveAs3(
        str(TOP_ASM),
        SW_SAVE_AS_CURRENT_VERSION,
        SW_SAVE_AS_SILENT,
    )
    if not TOP_ASM.is_file() or TOP_ASM.stat().st_size < 1024:
        raise RuntimeError(f"top assembly save failed: {TOP_ASM}")
    # The Zeroth source uses +X as its forward axis. A SolidWorks Right
    # standard view looks along X and therefore produces the robot front.
    save_view(
        model,
        4,
        SNAPSHOT_ROOT / "solidworks_physical_mount_front.png",
        view_name="*Right",
        rotate_degrees=-90.0,
    )
    save_view(
        model,
        7,
        SNAPSHOT_ROOT / "solidworks_physical_mount_isometric.png",
        view_name="*Isometric",
    )

    # Save a second review assembly. Component colours live in the fixed link
    # subassemblies; the top-level copy preserves those source identities.
    model.SaveAs3(
        str(XRAY_ASM),
        SW_SAVE_AS_CURRENT_VERSION,
        SW_SAVE_AS_SILENT,
    )

    moving = [joint for joint in joints if joint["type"] == "revolute"]
    FRAME_ROOT.mkdir(parents=True, exist_ok=True)
    for stale in FRAME_ROOT.glob("solidworks_physical_mount_*.png"):
        stale.unlink()
    frame_paths: list[Path] = []
    for frame_index in range(frame_count):
        phase = 2.0 * math.pi * frame_index / max(1, frame_count - 1)
        positions: dict[str, float] = {}
        for joint_index, joint in enumerate(moving):
            lower = float(joint["lower"])
            upper = float(joint["upper"])
            negative_room = max(0.0, -lower)
            positive_room = max(0.0, upper)
            amplitude = min(
                math.radians(8.0),
                0.6 * negative_room
                if negative_room
                else math.radians(3.0),
                0.6 * positive_room
                if positive_room
                else math.radians(3.0),
            )
            positions[str(joint["name"])] = amplitude * math.sin(
                phase + (joint_index % 4) * math.pi / 2.0
            )
        transforms = physical._forward_kinematics(
            base_link,
            joints,
            positions,
        )
        maximum_error = 0.0
        for link, component in components.items():
            maximum_error = max(
                maximum_error,
                base.set_component_transform(sw, component, transforms[link]),
            )
        if maximum_error >= 1e-8:
            raise RuntimeError(
                f"frame {frame_index} transform error {maximum_error}"
            )
        base.refresh_assembly_display(model)
        frame_path = (
            FRAME_ROOT
            / f"solidworks_physical_mount_{frame_index:03d}.png"
        )
        save_view(
            model,
            4,
            frame_path,
            view_name="*Right",
            rotate_degrees=-90.0,
        )
        frame_paths.append(frame_path)
        trace(f"motion frame {frame_index + 1}/{frame_count}")

    gif_ok = create_motion_gif(frame_paths)
    for link, component in components.items():
        base.set_component_transform(sw, component, neutral[link])
    base.refresh_assembly_display(model)
    model.SaveAs3(
        str(XRAY_ASM),
        SW_SAVE_AS_CURRENT_VERSION,
        SW_SAVE_AS_SILENT,
    )
    save_view(
        model,
        4,
        SNAPSHOT_ROOT / "solidworks_physical_mount_final_front.png",
        view_name="*Right",
        rotate_degrees=-90.0,
    )
    return rows, gif_ok


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Build fixed per-link SolidWorks subassemblies from the original "
            "Zeroth carrier geometry and 16 in-place extracted blue servos."
        )
    )
    parser.add_argument("--frame-count", type=int, default=25)
    parser.add_argument("--rebuild-parts", action="store_true")
    parser.add_argument("--rebuild-links", action="store_true")
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=240.0,
        help="Maximum observable SolidWorks startup time in seconds.",
    )
    parser.add_argument(
        "--probe-only",
        action="store_true",
        help="Only verify a bounded SolidWorks COM launch/attach.",
    )
    args = parser.parse_args()
    if args.frame_count < 3:
        raise ValueError("--frame-count must be at least 3")
    for required in (
        URDF_PATH,
        MANIFEST_PATH,
        base.PART_TEMPLATE,
        base.ASM_TEMPLATE,
    ):
        if not required.is_file():
            raise FileNotFoundError(required)
    if ROOT.drive.upper() != "E:":
        raise RuntimeError(f"SolidWorks generation must remain on E:, got {ROOT}")

    TRACE_LOG.unlink(missing_ok=True)
    pythoncom.CoInitialize()
    sw = None
    try:
        trace("SolidWorks physical-mount build start")
        (
            _,
            base_link,
            joints,
            skeleton_meshes,
            manifest,
            servos_by_link,
        ) = load_model()
        first_link = sorted(skeleton_meshes)[0]
        first_part = skeleton_part_path(first_link)
        startup_file = (
            SKELETON_DIR / skeleton_meshes[first_link]
            if args.rebuild_parts
            or not first_part.is_file()
            or first_part.stat().st_size < 1024
            else None
        )
        sw = get_or_start_sw(
            args.startup_timeout,
            startup_file=startup_file,
        )
        revision = str(base.call(sw, "RevisionNumber", ""))
        trace(f"SolidWorks COM responsive: revision={revision}")
        if args.probe_only:
            probe_document = active_document(sw)
            trace(
                "probe active document: "
                f"title={document_title(probe_document)!r} "
                f"path={str(document_path(probe_document) or '')!r}"
                if probe_document is not None
                else "probe active document: <none>"
            )
            trace("probe-only PASS")
            return 0
        close_task_documents(sw)
        part_rows = import_all_parts(
            sw,
            skeleton_meshes,
            manifest,
            rebuild=args.rebuild_parts,
        )
        write_csv(PART_REPORT, part_rows)
        sw = get_or_start_sw(args.startup_timeout)
        sw = typed_sldworks(sw)
        trace(f"SolidWorks assembly wrapper: {type(sw).__name__}")
        close_task_documents(sw)
        link_rows = create_link_subassemblies(
            sw,
            skeleton_meshes,
            servos_by_link,
            rebuild=args.rebuild_links,
        )
        write_csv(LINK_REPORT, link_rows)
        component_rows, gif_ok = create_top_assembly(
            sw,
            base_link,
            joints,
            skeleton_meshes,
            args.frame_count,
        )
        write_csv(COMPONENT_REPORT, component_rows)
        gate = {
            "schema": "zeroth01.physical_mount_v1.solidworks_gate.v1",
            "solidworks_revision": revision,
            "native_surface_part_count": len(part_rows),
            "fixed_link_subassembly_count": len(link_rows),
            "top_level_link_component_count": len(component_rows),
            "blue_servo_part_count": sum(
                1 for row in part_rows if row.get("kind") == "servo"
            ),
            "part_gate": (
                "PASS"
                if len(part_rows) == 36
                and all(row["status"] == "PASS" for row in part_rows)
                else "FAIL"
            ),
            "link_subassembly_gate": (
                "PASS"
                if len(link_rows) == 20
                and all(row["status"] == "PASS" for row in link_rows)
                else "FAIL"
            ),
            "top_level_transform_gate": (
                "PASS"
                if len(component_rows) == 20
                and all(row["status"] == "PASS" for row in component_rows)
                else "FAIL"
            ),
            "motion_gif_gate": "PASS" if gif_ok else "FAIL",
            "assembly": str(TOP_ASM),
            "xray_assembly": str(XRAY_ASM),
            "motion_gif": str(MOTION_GIF),
            "claim_boundary": (
                "Each link is a fixed SolidWorks subassembly containing its "
                "carrier surface part plus every extracted servo surface part "
                "at the original identity transform. The 20 rigid link "
                "subassemblies are moved by the URDF forward-kinematic "
                "transforms. Because upstream supplied triangulated STL rather "
                "than native B-Rep, this does not claim editable "
                "cylindrical-face mates or a native SOLIDWORKS Motion contact "
                "solution."
            ),
        }
        gate["overall"] = (
            "PASS"
            if all(
                gate[key] == "PASS"
                for key in (
                    "part_gate",
                    "link_subassembly_gate",
                    "top_level_transform_gate",
                    "motion_gif_gate",
                )
            )
            else "FAIL"
        )
        GATE_REPORT.write_text(
            json.dumps(gate, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
        trace(f"complete overall={gate['overall']} assembly={TOP_ASM}")
        print(GATE_REPORT)
        print(TOP_ASM)
        print(MOTION_GIF)
        return 0 if gate["overall"] == "PASS" else 2
    except Exception as error:
        trace(f"FAILED {type(error).__name__}: {error}")
        with TRACE_LOG.open("a", encoding="utf-8") as stream:
            stream.write(traceback.format_exc())
        raise
    finally:
        gc.collect()
        pythoncom.CoFreeUnusedLibraries()
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    raise SystemExit(main())
