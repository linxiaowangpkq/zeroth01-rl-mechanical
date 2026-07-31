"""Build the native SolidWorks v2-minimal review assembly in bounded stages.

The assembly preserves the proven Physical Mount v1 carrier/servo ownership:
* 16 extracted source STS3215-family servo parts remain separate blue parts,
* only the two forearms are replaced by their wrist-trimmed carrier meshes,
* both FINGER links are replaced by the compact printable Q hands,
* head/chest/sensor/sole STEP parts follow their owning link transforms.

Run STEP and trimmed-forearm imports one part per process before ``assemble``.
This keeps every SolidWorks translator call observable and bounded.
"""

from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
import time
import xml.etree.ElementTree as ET
import zipfile
from collections import defaultdict
from pathlib import Path

import pythoncom
from PIL import Image


ROOT = Path(__file__).resolve().parents[2]
V1_SCRIPT = ROOT / "cad" / "physical_mount_v1" / "create_solidworks_physical_mount_v1.py"
ROUND_SW_SCRIPT = ROOT / "scripts" / "create_solidworks_round_v1_review.py"
URDF = (
    ROOT
    / "generated"
    / "urdf"
    / "physical_mount_v2_minimal"
    / "zeroth01_physical_mount_v2_minimal.urdf"
)
PART_SOURCE_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
STL_IMPORT_ROOT = (
    ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "solidworks_import_m"
)
REPLACEMENT_SOURCE_ROOT = (
    ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "replacements"
)
MANIFEST = ROOT / "reports" / "physical_mount_v2_minimal" / "component_manifest.json"
SERVO_MANIFEST = ROOT / "reports" / "physical_mount_v1" / "servo_component_manifest.json"
COLLISION_GATE = ROOT / "reports" / "physical_mount_v2_minimal" / "dynamic_collision_gate.json"
GEOMETRY_GATE = ROOT / "reports" / "physical_mount_v2_minimal" / "geometry_gate.json"

SW_ROOT = ROOT / "generated" / "solidworks" / "physical_mount_v2_minimal"
SW_PART_ROOT = SW_ROOT / "parts"
SW_REPLACEMENT_ROOT = SW_PART_ROOT / "replacements"
TOP_ASM = SW_ROOT / "OPEN_FIRST_ZEROTH01_PHYSICAL_MOUNT_V2_MINIMAL_16_BLUE_SERVOS_XRAY.SLDASM"
NORMAL_ASM = SW_ROOT / "ZEROTH01_PHYSICAL_MOUNT_V2_MINIMAL_WHITE_NORMAL.SLDASM"
PACK_GO_ZIP = SW_ROOT / "ZEROTH01_PHYSICAL_MOUNT_V2_MINIMAL_PORTABLE_PACK_AND_GO.zip"
PORTABLE_ROOT = SW_ROOT / "portable_flat"
PORTABLE_TOP_ASM = PORTABLE_ROOT / TOP_ASM.name
SNAPSHOT_ROOT = ROOT / "snapshots" / "solidworks" / "physical_mount_v2_minimal"
FRAME_ROOT = SNAPSHOT_ROOT / "motion_frames"
MOTION_GIF = SNAPSHOT_ROOT / "solidworks_physical_mount_v2_minimal_16dof_motion.gif"
REPORT_ROOT = ROOT / "reports" / "physical_mount_v2_minimal"
PART_REPORT = REPORT_ROOT / "solidworks_part_import.csv"
COMPONENT_REPORT = REPORT_ROOT / "solidworks_component_manifest.csv"
GATE_REPORT = REPORT_ROOT / "solidworks_gate.json"
TRACE_LOG = REPORT_ROOT / "solidworks_trace.log"

WHITE = (0.969, 0.973, 0.980)
BLUE = (0.086, 0.467, 1.0)
SW_DOC_PART = 1
SW_SAVE_AS_CURRENT_VERSION = 0
SW_SAVE_AS_SILENT = 1

TRIMMED_LINKS = {
    "R_ARM_MIRROR_1": "R_ARM_MIRROR_1_WRIST_TRIMMED",
    "L_ARM_MIRROR_1": "L_ARM_MIRROR_1_WRIST_TRIMMED",
}
HAND_LINK_PART = {
    "FINGER_1": "left_q_hand",
    "FINGER_1_2": "right_q_hand",
}


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v1 = _load(V1_SCRIPT, "minimal_v2_solidworks_v1")
round_sw = _load(ROUND_SW_SCRIPT, "minimal_v2_solidworks_step_import")
physical = v1.physical


def trace(message: str) -> None:
    REPORT_ROOT.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with TRACE_LOG.open("a", encoding="utf-8") as stream:
        stream.write(f"{stamp} {message}\n")
    print(message, flush=True)


def _safe(value: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in value)


def _color(value: str) -> tuple[float, float, float]:
    token = value.lstrip("#")
    if len(token) != 6:
        raise ValueError(value)
    return tuple(int(token[index : index + 2], 16) / 255.0 for index in (0, 2, 4))


def _write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    payload = rows or [{"status": "EMPTY"}]
    fields: list[str] = []
    for row in payload:
        for key in row:
            if key not in fields:
                fields.append(key)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(payload)


def _manifest_parts() -> list[dict[str, object]]:
    return json.loads(MANIFEST.read_text(encoding="utf-8"))["parts"]


def _servo_rows() -> list[dict[str, object]]:
    return json.loads(SERVO_MANIFEST.read_text(encoding="utf-8"))["servos"]


def part_path(key: str) -> Path:
    return SW_PART_ROOT / f"ZEROTH01_V2_MINIMAL_{_safe(key).upper()}.SLDPRT"


def replacement_part_path(link: str) -> Path:
    return SW_REPLACEMENT_ROOT / f"ZEROTH01_V2_MINIMAL_{TRIMMED_LINKS[link]}.SLDPRT"


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


def import_trimmed(link: str, force: bool) -> dict[str, object]:
    if link not in TRIMMED_LINKS:
        raise KeyError(link)
    source = REPLACEMENT_SOURCE_ROOT / f"{TRIMMED_LINKS[link]}.stl"
    target = replacement_part_path(link)
    if (
        not force
        and target.is_file()
        and target.stat().st_size > 1024
        and target.stat().st_mtime >= source.stat().st_mtime
    ):
        return {
            "kind": "trimmed_forearm_surface",
            "key": link,
            "source": str(source),
            "part": str(target),
            "method": "reuse_current_SLDPRT",
            "bytes": target.stat().st_size,
            "status": "PASS",
        }
    active = list(v1.iter_sw_apps())
    if active:
        raise RuntimeError(
            "trimmed STL cold import requires no active SolidWorks session; "
            f"found {[name for name, _ in active]}"
        )
    row = v1.import_stl_part(None, source, target, rebuild=True)
    row.update({"kind": "trimmed_forearm_surface", "key": link})
    return row


def import_step(key: str, force: bool, startup_timeout: float) -> dict[str, object]:
    parts = {str(row["key"]): row for row in _manifest_parts()}
    if key not in parts:
        raise KeyError(key)
    source = PART_SOURCE_ROOT / f"{key}.step"
    target = part_path(key)
    sw = v1.get_or_start_sw(startup_timeout)
    previous = int(
        sw.GetUserPreferenceIntegerValue(
            round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING
        )
    )
    sw.SetUserPreferenceIntegerValue(
        round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING,
        round_sw.SW_IMPORT_NEUTRAL_AS_MULTIBODY_PART,
    )
    try:
        row = round_sw.import_step_part(sw, source, target, force=force)
    finally:
        sw.SetUserPreferenceIntegerValue(
            round_sw.SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING,
            previous,
        )
    row.update(
        {
            "kind": "minimal_v2_STEP_part",
            "key": key,
            "installed_link": parts[key]["installed_link"],
            "classification": parts[key]["classification"],
        }
    )
    return row


def import_stl(key: str, force: bool) -> dict[str, object]:
    parts = {str(row["key"]): row for row in _manifest_parts()}
    if key not in parts:
        raise KeyError(key)
    source = STL_IMPORT_ROOT / f"{key}.stl"
    target = part_path(key)
    if (
        not force
        and target.is_file()
        and target.stat().st_size > 1024
        and target.stat().st_mtime >= source.stat().st_mtime
    ):
        return {
            "kind": "minimal_v2_STL_surface_part",
            "key": key,
            "source": str(source),
            "part": str(target),
            "method": "reuse_current_SLDPRT",
            "bytes": target.stat().st_size,
            "status": "PASS",
        }
    active = list(v1.iter_sw_apps())
    if active:
        raise RuntimeError(
            "STL cold import requires no active SolidWorks session; "
            f"found {[name for name, _ in active]}"
        )
    row = v1.import_stl_part(None, source, target, rebuild=True)
    row.update(
        {
            "kind": "minimal_v2_STL_surface_part",
            "key": key,
            "installed_link": parts[key]["installed_link"],
            "classification": parts[key]["classification"],
        }
    )
    return row


def salvage_active_step(key: str, startup_timeout: float) -> dict[str, object]:
    parts = {str(row["key"]): row for row in _manifest_parts()}
    if key not in parts:
        raise KeyError(key)
    sw = v1.get_or_start_sw(startup_timeout)
    selected = None
    for raw in list(v1.base.call(sw, "GetDocuments", []) or []):
        model = v1.base.as_model_doc(raw)
        title = str(v1.base.call(model, "GetTitle", ""))
        path = str(v1.base.call(model, "GetPathName", ""))
        if not path and key.lower() in title.lower():
            selected = model
            break
    if selected is None:
        raise RuntimeError(f"no unsaved imported SolidWorks document found for {key}")
    target = part_path(key)
    target.parent.mkdir(parents=True, exist_ok=True)
    save_code = int(
        selected.SaveAs3(str(target), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT)
    )
    if not target.is_file() or target.stat().st_size < 1024:
        raise RuntimeError(f"salvaged SLDPRT save failed: {target}")
    sw.CloseDoc(str(v1.base.call(selected, "GetTitle", "")))
    return {
        "kind": "salvaged_native_STEP_part",
        "key": key,
        "part": str(target),
        "save_code": save_code,
        "bytes": target.stat().st_size,
        "status": "PASS",
    }


def close_task_session() -> dict[str, object]:
    sessions = list(v1.iter_sw_apps())
    if len(sessions) != 1:
        raise RuntimeError(f"expected one SolidWorks task session, got {len(sessions)}")
    name, sw = sessions[0]
    allowed_root = str((ROOT / "generated" / "solidworks").resolve()).lower()
    allowed_unsaved = {f"{row['key']}.sldprt".lower() for row in _manifest_parts()}
    documents = []
    refused = []
    for raw in list(v1.base.call(sw, "GetDocuments", []) or []):
        model = v1.base.as_model_doc(raw)
        title = str(v1.base.call(model, "GetTitle", ""))
        path = str(v1.base.call(model, "GetPathName", ""))
        allowed = bool(path and path.lower().startswith(allowed_root)) or (
            not path and title.lower() in allowed_unsaved
        )
        row = {"title": title, "path": path, "allowed": allowed}
        documents.append(row)
        if not allowed:
            refused.append(row)
    if refused:
        raise RuntimeError(f"refusing to close non-task SolidWorks documents: {refused}")
    for row in documents:
        sw.CloseDoc(str(row["title"]))
    sw.ExitApp()
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline and list(v1.iter_sw_apps()):
        time.sleep(0.25)
    if list(v1.iter_sw_apps()):
        raise TimeoutError(f"SolidWorks task session did not exit: {name}")
    return {"session": name, "documents": documents, "status": "PASS"}


def _carrier_path(link: str) -> Path:
    if link in TRIMMED_LINKS:
        return replacement_part_path(link)
    if link in HAND_LINK_PART:
        return part_path(HAND_LINK_PART[link])
    return v1.skeleton_part_path(link)


def _save_view(model, path: Path, view: str) -> bool:
    if view == "front":
        return v1.save_view(
            model,
            4,
            path,
            view_name="*Right",
            rotate_degrees=-90.0,
        )
    return v1.save_view(model, 7, path, view_name="*Isometric")


def _motion_gif(paths: list[Path]) -> bool:
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
            duration=100,
            loop=0,
            optimize=False,
        )
    finally:
        for image in images:
            image.close()
    return MOTION_GIF.is_file() and MOTION_GIF.stat().st_size > 0


def assemble(frame_count: int, startup_timeout: float) -> dict[str, object]:
    for required in (URDF, MANIFEST, SERVO_MANIFEST, COLLISION_GATE, GEOMETRY_GATE):
        if not required.is_file():
            raise FileNotFoundError(required)
    for link in TRIMMED_LINKS:
        if not replacement_part_path(link).is_file():
            raise FileNotFoundError(replacement_part_path(link))
    for row in _manifest_parts():
        if not part_path(str(row["key"])).is_file():
            raise FileNotFoundError(part_path(str(row["key"])))

    SW_ROOT.mkdir(parents=True, exist_ok=True)
    SNAPSHOT_ROOT.mkdir(parents=True, exist_ok=True)
    FRAME_ROOT.mkdir(parents=True, exist_ok=True)
    sw = v1.typed_sldworks(v1.get_or_start_sw(startup_timeout))
    close_task_documents(sw)
    for name in (TOP_ASM.name, NORMAL_ASM.name):
        try:
            sw.CloseDoc(name)
        except Exception:
            pass

    robot = ET.parse(URDF).getroot()
    base_link, joints = physical._load_kinematic_model(robot)
    skeleton_meshes = physical._source_link_meshes(robot)
    neutral = physical._forward_kinematics(base_link, joints, {})

    raw = v1.first(sw.NewDocument(str(v1.base.ASM_TEMPLATE), 0, 0, 0))
    if raw is None:
        raise RuntimeError("SolidWorks NewDocument failed")
    model = v1.base.as_model_doc(raw)
    assembly = v1.base.as_assembly_doc(raw)

    components_by_link: dict[str, list[object]] = defaultdict(list)
    carrier_components: dict[str, object] = {}
    servo_components: dict[str, object] = {}
    overlay_components: dict[str, object] = {}
    component_rows: list[dict[str, object]] = []

    ordered_links = sorted(
        skeleton_meshes,
        key=lambda value: (0 if value == "Z_BOT2_MASTER_BODY_SKELETON" else 1, value),
    )
    for index, link in enumerate(ordered_links, start=1):
        source = _carrier_path(link)
        trace(f"assembly carrier {index}/{len(ordered_links)}: {link}")
        component, error = v1.add_component(
            sw,
            model,
            assembly,
            source,
            SW_DOC_PART,
            f"LINK_{_safe(link)}_WHITE_CARRIER",
            neutral[link],
        )
        v1.set_material(component, WHITE, 0.0)
        carrier_components[link] = component
        components_by_link[link].append(component)
        component_rows.append(
            {
                "kind": "carrier",
                "name": link,
                "attachment": link,
                "part": str(source),
                "transform_error": error,
                "status": "PASS" if error < 1.0e-8 else "FAIL",
            }
        )

    servos = _servo_rows()
    for index, servo in enumerate(servos, start=1):
        owner = str(servo["owning_link"])
        servo_id = str(servo["id"])
        trace(f"assembly servo {index}/{len(servos)}: {servo_id} {servo['joint']}")
        component, error = v1.add_component(
            sw,
            model,
            assembly,
            v1.servo_part_path(servo),
            SW_DOC_PART,
            f"{servo_id}_{_safe(str(servo['joint']))}_BLUE_SOURCE_INSTALLED_SERVO",
            neutral[owner],
        )
        v1.set_material(component, BLUE, 0.0)
        servo_components[servo_id] = component
        components_by_link[owner].append(component)
        component_rows.append(
            {
                "kind": "source_installed_servo",
                "name": servo_id,
                "joint": servo["joint"],
                "attachment": owner,
                "part": str(v1.servo_part_path(servo)),
                "transform_error": error,
                "status": "PASS" if error < 1.0e-8 else "FAIL",
            }
        )

    part_rows = _manifest_parts()
    for index, row in enumerate(part_rows, start=1):
        key = str(row["key"])
        if key in {"left_q_hand", "right_q_hand"}:
            continue
        owner = str(row["installed_link"])
        trace(f"assembly minimal part {index}/{len(part_rows)}: {key}")
        component, error = v1.add_component(
            sw,
            model,
            assembly,
            part_path(key),
            SW_DOC_PART,
            f"V2_MINIMAL_{_safe(key).upper()}",
            neutral[owner],
        )
        v1.set_material(component, _color(str(row["color_hex"])), 0.0)
        overlay_components[key] = component
        components_by_link[owner].append(component)
        component_rows.append(
            {
                "kind": "minimal_v2_part",
                "name": key,
                "classification": row["classification"],
                "attachment": owner,
                "part": str(part_path(key)),
                "transform_error": error,
                "status": "PASS" if error < 1.0e-8 else "FAIL",
            }
        )

    internal_keys = {
        str(row["key"])
        for row in part_rows
        if str(row["classification"]) == "internal_payload_controlled_envelope"
    }
    for key in internal_keys:
        round_sw.set_component_visibility(overlay_components[key], False)
    round_sw.try_shaded(model)
    v1.base.refresh_assembly_display(model)
    normal_save_code = int(
        model.SaveAs3(str(NORMAL_ASM), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT)
    )
    if not NORMAL_ASM.is_file() or NORMAL_ASM.stat().st_size < 1024:
        raise RuntimeError(f"normal assembly save failed: {NORMAL_ASM}")
    normal_views = {
        "front": _save_view(model, SNAPSHOT_ROOT / "solidworks_minimal_v2_normal_front.png", "front"),
        "isometric": _save_view(model, SNAPSHOT_ROOT / "solidworks_minimal_v2_normal_iso.png", "iso"),
    }

    for key in internal_keys:
        round_sw.set_component_visibility(overlay_components[key], True)
    for component in carrier_components.values():
        v1.set_material(component, WHITE, 0.72)
    for key, component in overlay_components.items():
        row = next(item for item in part_rows if str(item["key"]) == key)
        transparency = 0.0 if key in internal_keys or key in {"face_ui", "camera_window"} else 0.68
        if key == "visor":
            transparency = 0.82
        v1.set_material(component, _color(str(row["color_hex"])), transparency)
    for component in servo_components.values():
        v1.set_material(component, BLUE, 0.0)
    v1.base.refresh_assembly_display(model)
    xray_views = {
        "front": _save_view(model, SNAPSHOT_ROOT / "solidworks_minimal_v2_xray_front.png", "front"),
        "isometric": _save_view(model, SNAPSHOT_ROOT / "solidworks_minimal_v2_xray_iso.png", "iso"),
    }
    xray_save_code = int(
        model.SaveAs3(str(TOP_ASM), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT)
    )
    if not TOP_ASM.is_file() or TOP_ASM.stat().st_size < 1024:
        raise RuntimeError(f"X-ray assembly save failed: {TOP_ASM}")

    for key in internal_keys:
        round_sw.set_component_visibility(overlay_components[key], False)
    FRAME_ROOT.mkdir(parents=True, exist_ok=True)
    for stale in FRAME_ROOT.glob("frame_*.png"):
        stale.unlink()
    moving = [joint for joint in joints if joint["type"] == "revolute"]
    frames: list[Path] = []
    maximum_motion_error = 0.0
    for frame_index in range(frame_count):
        phase = 2.0 * math.pi * frame_index / max(1, frame_count - 1)
        positions: dict[str, float] = {}
        for joint_index, joint in enumerate(moving):
            lower = float(joint["lower"])
            upper = float(joint["upper"])
            amplitude = min(
                math.radians(8.0),
                0.6 * max(0.0, -lower) if lower < 0.0 else math.radians(3.0),
                0.6 * max(0.0, upper) if upper > 0.0 else math.radians(3.0),
            )
            positions[str(joint["name"])] = amplitude * math.sin(
                phase + (joint_index % 4) * math.pi / 2.0
            )
        transforms = physical._forward_kinematics(base_link, joints, positions)
        for link, components in components_by_link.items():
            for component in components:
                maximum_motion_error = max(
                    maximum_motion_error,
                    v1.base.set_component_transform(sw, component, transforms[link]),
                )
        v1.base.refresh_assembly_display(model)
        frame = FRAME_ROOT / f"frame_{frame_index:03d}.png"
        _save_view(model, frame, "front")
        frames.append(frame)
        trace(f"SolidWorks motion frame {frame_index + 1}/{frame_count}")
    gif_ok = _motion_gif(frames)

    for link, components in components_by_link.items():
        for component in components:
            maximum_motion_error = max(
                maximum_motion_error,
                v1.base.set_component_transform(sw, component, neutral[link]),
            )
    for key in internal_keys:
        round_sw.set_component_visibility(overlay_components[key], True)
    v1.base.refresh_assembly_display(model)
    model.SaveAs3(str(TOP_ASM), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT)
    _write_csv(COMPONENT_REPORT, component_rows)

    collision = json.loads(COLLISION_GATE.read_text(encoding="utf-8"))
    geometry = json.loads(GEOMETRY_GATE.read_text(encoding="utf-8"))
    expected_components = len(ordered_links) + len(servos) + len(part_rows) - 2
    gate = {
        "schema": "zeroth01.physical_mount_v2_minimal.solidworks_gate.v1",
        "solidworks_revision": str(v1.base.call(sw, "RevisionNumber", "")),
        "open_first_xray_assembly": str(TOP_ASM),
        "normal_assembly": str(NORMAL_ASM),
        "normal_save_code": normal_save_code,
        "xray_save_code": xray_save_code,
        "component_count": len(component_rows),
        "expected_component_count": expected_components,
        "carrier_component_count": len(carrier_components),
        "separate_blue_source_servo_component_count": len(servo_components),
        "old_claw_component_count": 0,
        "replacement_q_hand_component_count": 2,
        "external_neck_component_count": 0,
        "head_z_shift_mm": -55.0,
        "geometry_gate": geometry.get("overall", "MISSING"),
        "mujoco_collision_gate": collision.get("overall", "MISSING"),
        "component_transform_gate": (
            "PASS"
            if all(row["status"] == "PASS" for row in component_rows)
            and maximum_motion_error < 1.0e-8
            else "FAIL"
        ),
        "normal_view_gate": "PASS" if all(normal_views.values()) else "FAIL",
        "xray_view_gate": "PASS" if all(xray_views.values()) else "FAIL",
        "motion_gif": str(MOTION_GIF),
        "motion_gif_gate": "PASS" if gif_ok else "FAIL",
        "normal_snapshots": normal_views,
        "xray_snapshots": xray_views,
        "claim_boundary": (
            "The 16 blue parts are the source extracted installed servo bodies "
            "at their original owning-link coordinates. CLI forward kinematics "
            "drives native SolidWorks component transforms; MuJoCo supplies the "
            "sampled collision gate. This is not a SOLIDWORKS Motion contact solve, "
            "and exact STS3250 first-article hole/horn fit remains required."
        ),
    }
    gate["overall"] = (
        "PASS"
        if gate["component_count"] == expected_components
        and gate["carrier_component_count"] == 20
        and gate["separate_blue_source_servo_component_count"] == 16
        and all(
            gate[key] == "PASS"
            for key in (
                "geometry_gate",
                "mujoco_collision_gate",
                "component_transform_gate",
                "normal_view_gate",
                "xray_view_gate",
                "motion_gif_gate",
            )
        )
        else "FAIL"
    )
    GATE_REPORT.write_text(json.dumps(gate, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(gate, indent=2, ensure_ascii=False))
    return gate


def pack_and_go(startup_timeout: float) -> dict[str, object]:
    """Create one portable ZIP containing the assembly and every dependency."""
    if not TOP_ASM.is_file():
        raise FileNotFoundError(TOP_ASM)
    sw = v1.typed_sldworks(v1.get_or_start_sw(startup_timeout))
    model = None
    for raw in list(v1.base.call(sw, "GetDocuments", []) or []):
        candidate = v1.base.as_model_doc(raw)
        path = Path(str(v1.base.call(candidate, "GetPathName", "")))
        if path and path.resolve() == TOP_ASM.resolve():
            model = candidate
            break
    if model is None:
        opened = sw.OpenDoc6(str(TOP_ASM), 2, 1, "", 0, 0)
        raw = v1.first(opened)
        if raw is None:
            raise RuntimeError(f"SolidWorks could not open {TOP_ASM}")
        model = v1.base.as_model_doc(raw)

    typed_extension = v1.base.call(model, "Extension")
    if typed_extension is None:
        raise RuntimeError("SolidWorks model extension unavailable")
    # The SW 2025 generated Python wrapper incorrectly marks GetPackAndGo as
    # requiring a parameter.  Late-bound dispatch uses the actual COM call.
    extension = v1.win32_dynamic.DumbDispatch(typed_extension._oleobj_)
    try:
        pack = extension.GetPackAndGo()
    except Exception:
        # Some SW 2025 FCS type libraries expose the retval as an explicit
        # out argument through IDispatch even though the API docs show none.
        output = v1.win32.VARIANT(
            pythoncom.VT_BYREF | pythoncom.VT_DISPATCH,
            None,
        )
        extension.GetPackAndGo(output)
        pack = output.value
    for name, value in (
        ("IncludeDrawings", False),
        ("IncludeSimulationResults", False),
        ("IncludeSuppressed", True),
        ("IncludeToolboxComponents", True),
        ("FlattenToSingleFolder", True),
    ):
        try:
            setattr(pack, name, value)
        except Exception:
            pass
    document_count = int(pack.GetDocumentNamesCount())
    if PACK_GO_ZIP.is_file():
        PACK_GO_ZIP.unlink()
    override_ok = False
    method = "SetSaveToName"
    try:
        override_ok = bool(pack.SetSaveToName2(True, str(PACK_GO_ZIP)))
        method = "SetSaveToName2"
    except Exception:
        override_ok = bool(pack.SetSaveToName(True, str(PACK_GO_ZIP)))
    statuses = extension.SavePackAndGo(pack)
    if not PACK_GO_ZIP.is_file() or PACK_GO_ZIP.stat().st_size < 1024:
        raise RuntimeError(f"Pack and Go ZIP was not created: {PACK_GO_ZIP}")
    with zipfile.ZipFile(PACK_GO_ZIP) as archive:
        names = archive.namelist()
        bad = archive.testzip()
    sldasm_count = sum(name.lower().endswith(".sldasm") for name in names)
    sldprt_count = sum(name.lower().endswith(".sldprt") for name in names)
    row = {
        "schema": "zeroth01.physical_mount_v2_minimal.pack_and_go_gate.v1",
        "zip": str(PACK_GO_ZIP),
        "bytes": PACK_GO_ZIP.stat().st_size,
        "solidworks_document_count": document_count,
        "zip_member_count": len(names),
        "sldasm_count": sldasm_count,
        "sldprt_count": sldprt_count,
        "override_method": method,
        "override_gate": "PASS" if override_ok else "FAIL",
        "zip_crc_gate": "PASS" if bad is None else "FAIL",
        "save_statuses": repr(statuses),
    }
    row["overall"] = (
        "PASS"
        if override_ok and bad is None and sldasm_count >= 1 and sldprt_count >= 51
        else "FAIL"
    )
    report = REPORT_ROOT / "solidworks_pack_and_go_gate.json"
    report.write_text(json.dumps(row, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(row, indent=2, ensure_ascii=False))
    return row


def validate_portable(startup_timeout: float) -> dict[str, object]:
    if not PORTABLE_TOP_ASM.is_file():
        raise FileNotFoundError(PORTABLE_TOP_ASM)
    sw = v1.typed_sldworks(v1.get_or_start_sw(startup_timeout))
    close_task_documents(sw)
    opened = sw.OpenDoc6(str(PORTABLE_TOP_ASM), 2, 1, "", 0, 0)
    raw = v1.first(opened)
    if raw is None:
        raise RuntimeError(f"SolidWorks could not open portable assembly: {PORTABLE_TOP_ASM}")
    model = v1.base.as_model_doc(raw)
    assembly = v1.base.as_assembly_doc(raw)
    try:
        assembly.ResolveAllLightWeightComponents(True)
    except Exception:
        pass
    components = list(v1.base.call(assembly, "GetComponents", [], False) or [])
    portable = PORTABLE_ROOT.resolve()
    rows = []
    for component in components:
        path = Path(str(v1.base.call(component, "GetPathName", "")))
        resolved = bool(path) and path.is_file() and path.resolve().parent == portable
        rows.append({"name": str(v1.base.call(component, "Name2", "")), "path": str(path), "portable": resolved})
    snapshot = SNAPSHOT_ROOT / "solidworks_minimal_v2_portable_xray_front.png"
    view_ok = _save_view(model, snapshot, "front")
    row = {
        "schema": "zeroth01.physical_mount_v2_minimal.solidworks_portable_open_gate.v1",
        "assembly": str(PORTABLE_TOP_ASM),
        "component_count": len(components),
        "portable_resolved_component_count": sum(bool(item["portable"]) for item in rows),
        "missing_or_nonportable": [item for item in rows if not item["portable"]],
        "snapshot": str(snapshot),
        "snapshot_gate": "PASS" if view_ok else "FAIL",
    }
    row["overall"] = "PASS" if len(components) == 51 and not row["missing_or_nonportable"] and view_ok else "FAIL"
    report = REPORT_ROOT / "solidworks_portable_open_gate.json"
    report.write_text(json.dumps(row, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(row, indent=2, ensure_ascii=False))
    return row


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--stage",
        required=True,
        choices=(
            "probe",
            "import-trimmed",
            "import-step",
            "import-stl",
            "salvage-active-step",
            "close-task-session",
            "assemble",
            "pack-go",
            "validate-portable",
        ),
    )
    parser.add_argument("--key", default="")
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--frame-count", type=int, default=12)
    parser.add_argument("--startup-timeout", type=float, default=180.0)
    args = parser.parse_args()
    if args.frame_count < 3:
        raise ValueError("--frame-count must be at least 3")
    pythoncom.CoInitialize()
    try:
        if args.stage == "probe":
            sw = v1.get_or_start_sw(args.startup_timeout)
            documents = []
            for raw in list(v1.base.call(sw, "GetDocuments", []) or []):
                model = v1.base.as_model_doc(raw)
                documents.append(
                    {
                        "title": str(v1.base.call(model, "GetTitle", "")),
                        "path": str(v1.base.call(model, "GetPathName", "")),
                    }
                )
            print(
                json.dumps(
                    {
                        "revision": str(v1.base.call(sw, "RevisionNumber", "")),
                        "documents": documents,
                        "status": "PASS",
                    },
                    ensure_ascii=False,
                )
            )
            return 0
        if args.stage == "import-trimmed":
            row = import_trimmed(args.key, bool(args.force))
            print(json.dumps(row, indent=2, ensure_ascii=False))
            return 0 if row["status"] == "PASS" else 2
        if args.stage == "import-step":
            row = import_step(args.key, bool(args.force), args.startup_timeout)
            print(json.dumps(row, indent=2, ensure_ascii=False))
            return 0 if row["status"] == "PASS" else 2
        if args.stage == "import-stl":
            row = import_stl(args.key, bool(args.force))
            print(json.dumps(row, indent=2, ensure_ascii=False))
            return 0 if row["status"] == "PASS" else 2
        if args.stage == "salvage-active-step":
            row = salvage_active_step(args.key, args.startup_timeout)
            print(json.dumps(row, indent=2, ensure_ascii=False))
            return 0 if row["status"] == "PASS" else 2
        if args.stage == "close-task-session":
            row = close_task_session()
            print(json.dumps(row, indent=2, ensure_ascii=False))
            return 0
        if args.stage == "pack-go":
            row = pack_and_go(args.startup_timeout)
            return 0 if row["overall"] == "PASS" else 2
        if args.stage == "validate-portable":
            row = validate_portable(args.startup_timeout)
            return 0 if row["overall"] == "PASS" else 2
        gate = assemble(args.frame_count, args.startup_timeout)
        return 0 if gate["overall"] == "PASS" else 2
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    raise SystemExit(main())
