from __future__ import annotations

import argparse
import csv
import json
import math
import time
from pathlib import Path
import xml.etree.ElementTree as ET

import pythoncom
import win32com.client as win32
from PIL import Image


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_reference.urdf"
MESH_DIR = URDF.parent / "meshes"
PART_DIR = ROOT / "generated" / "solidworks" / "parts"
ASM_DIR = ROOT / "generated" / "solidworks"
SNAP_DIR = ROOT / "snapshots" / "solidworks"
FRAME_DIR = SNAP_DIR / "motion_frames"
REPORT_DIR = ROOT / "reports"
ASM_PATH = ASM_DIR / "OPEN_FIRST_ZEROTH01_16DOF_KINEMATIC_REVIEW.SLDASM"
MOTION_GIF = SNAP_DIR / "zeroth01_solidworks_kinematic_motion.gif"
PART_REPORT = REPORT_DIR / "solidworks_part_import.csv"
COMPONENT_REPORT = REPORT_DIR / "solidworks_component_manifest.csv"
SWEEP_REPORT = REPORT_DIR / "solidworks_kinematic_sweep.csv"
INTERFERENCE_REPORT = REPORT_DIR / "solidworks_surface_interference_probe.csv"
GATE_REPORT = REPORT_DIR / "solidworks_review_gate.csv"
TRACE_LOG = REPORT_DIR / "solidworks_trace.log"

PART_TEMPLATE = Path(
    r"C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2025\templates\gb_part.prtdot"
)
ASM_TEMPLATE = Path(
    r"C:\ProgramData\SOLIDWORKS\SOLIDWORKS 2025\templates\gb_assembly.asmdot"
)

SW_DOC_PART = 1
SW_SAVE_AS_CURRENT_VERSION = 0
SW_SAVE_AS_SILENT = 1
SW_OPEN_SILENT = 1
SW_IMPORT_STL_VRML_MODEL_TYPE = 208
SW_IMPORT_STL_VRML_UNITS = 210
SW_IMPORT_AS_SURFACE = 1
SW_LENGTH_UNIT_METER = 2

SW_MAIN_TYPELIB = "{83A33D31-27C5-11CE-BFD4-00400513BB57}"
IID_IMODELDOC2 = "{B90793FB-EF3D-4B80-A5C4-99959CDB6CEB}"
IID_IASSEMBLYDOC = "{83A33D35-27C5-11CE-BFD4-00400513BB57}"
IID_IPARTDOC = "{83A33D34-27C5-11CE-BFD4-00400513BB57}"

IDENTITY_R = [
    [1.0, 0.0, 0.0],
    [0.0, 1.0, 0.0],
    [0.0, 0.0, 1.0],
]

# The geometry-compatible 33b0553 link names refer to the same Drive STL
# payloads already imported under the later 43c5baa link vocabulary.
PART_PATH_ALIASES = {
    "shoulder_yaw_right": "right_shoulder_yaw_2",
    "shoulder_yaw_left": "left_shoulder_yaw_2",
    "Left_Hand": "right_hand",
    "hand_right": "left_hand",
    "knee_pitch_right": "knee_pitch_left",
    "knee_pitch_right_2": "knee_pitch_right",
    "ankle_pitch_right": "right_knee_pitch_motor",
    "ankle_pitch_left": "left_knee_pitch_motor",
}


def trace(message: str) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with TRACE_LOG.open("a", encoding="utf-8") as stream:
        stream.write(f"{stamp} {message}\n")
    print(message, flush=True)


def first(value):
    return value[0] if isinstance(value, tuple) else value


def call(obj, name: str, default=None, *args):
    try:
        value = getattr(obj, name)
        return value(*args) if callable(value) else value
    except Exception:
        return default


def safe_name(value: str) -> str:
    return "".join(character if character.isalnum() else "_" for character in value)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = rows or [{"status": "EMPTY"}]
    fields: list[str] = []
    for item in data:
        for key in item:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(data)


def parse_vec(text: str | None, default=(0.0, 0.0, 0.0)) -> list[float]:
    return [float(value) for value in text.split()] if text else list(default)


def mat_mul(a, b):
    return [
        [sum(a[row][index] * b[index][column] for index in range(3)) for column in range(3)]
        for row in range(3)
    ]


def mat_vec(matrix, vector):
    return [
        sum(matrix[row][column] * vector[column] for column in range(3))
        for row in range(3)
    ]


def vec_add(a, b):
    return [a[index] + b[index] for index in range(3)]


def tf_mul(a, b):
    ar, at = a
    br, bt = b
    return mat_mul(ar, br), vec_add(mat_vec(ar, bt), at)


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = [[1, 0, 0], [0, cr, -sr], [0, sr, cr]]
    ry = [[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]]
    rz = [[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]]
    return mat_mul(rz, mat_mul(ry, rx))


def axis_angle_matrix(axis, angle):
    norm = math.sqrt(sum(value * value for value in axis))
    if norm <= 1e-12:
        return [row[:] for row in IDENTITY_R]
    x, y, z = (value / norm for value in axis)
    cosine = math.cos(angle)
    sine = math.sin(angle)
    one_minus = 1.0 - cosine
    return [
        [
            cosine + x * x * one_minus,
            x * y * one_minus - z * sine,
            x * z * one_minus + y * sine,
        ],
        [
            y * x * one_minus + z * sine,
            cosine + y * y * one_minus,
            y * z * one_minus - x * sine,
        ],
        [
            z * x * one_minus - y * sine,
            z * y * one_minus + x * sine,
            cosine + z * z * one_minus,
        ],
    ]


def rotation_distance(a, b) -> float:
    relative = mat_mul(
        [[a[column][row] for column in range(3)] for row in range(3)],
        b,
    )
    cosine = max(-1.0, min(1.0, (sum(relative[i][i] for i in range(3)) - 1.0) / 2.0))
    return math.acos(cosine)


def load_model_data():
    root = ET.parse(URDF).getroot()
    link_meshes: dict[str, str] = {}
    for link in root.findall("link"):
        mesh = link.find("./visual/geometry/mesh")
        if mesh is not None:
            link_meshes[link.get("name", "")] = Path(
                mesh.get("filename", "").replace("\\", "/")
            ).name
    joints: list[dict[str, object]] = []
    for joint in root.findall("joint"):
        origin = joint.find("origin")
        axis = joint.find("axis")
        limit = joint.find("limit")
        joints.append(
            {
                "name": joint.get("name", ""),
                "type": joint.get("type", ""),
                "parent": joint.find("parent").get("link", ""),
                "child": joint.find("child").get("link", ""),
                "origin": (
                    rpy_matrix(
                        parse_vec(
                            origin.get("rpy") if origin is not None else None
                        )
                    ),
                    parse_vec(origin.get("xyz") if origin is not None else None),
                ),
                "axis": parse_vec(
                    axis.get("xyz") if axis is not None else None, (0, 0, 1)
                ),
                "lower": (
                    float(limit.get("lower"))
                    if limit is not None and limit.get("lower") is not None
                    else 0.0
                ),
                "upper": (
                    float(limit.get("upper"))
                    if limit is not None and limit.get("upper") is not None
                    else 0.0
                ),
            }
        )
    return link_meshes, joints


def forward_kinematics(joints, q: dict[str, float]):
    transforms = {"base": ([row[:] for row in IDENTITY_R], [0.0, 0.0, 0.0])}
    pending = list(joints)
    while pending:
        changed = False
        for joint in pending[:]:
            parent = str(joint["parent"])
            if parent not in transforms:
                continue
            angle = float(q.get(str(joint["name"]), 0.0))
            motion = (
                axis_angle_matrix(joint["axis"], angle)
                if joint["type"] in {"revolute", "continuous"}
                else [row[:] for row in IDENTITY_R],
                [0.0, 0.0, 0.0],
            )
            transforms[str(joint["child"])] = tf_mul(
                transforms[parent], tf_mul(joint["origin"], motion)
            )
            pending.remove(joint)
            changed = True
        if not changed:
            raise RuntimeError(
                f"URDF tree did not resolve: {[joint['name'] for joint in pending]}"
            )
    return transforms


def transform_array(transform) -> list[float]:
    rotation, translation_m = transform
    # URDF FK is represented with column vectors. SolidWorks IMathTransform
    # stores the 3x3 block for its row-vector convention, so serialize R^T.
    return [
        rotation[0][0],
        rotation[1][0],
        rotation[2][0],
        rotation[0][1],
        rotation[1][1],
        rotation[2][1],
        rotation[0][2],
        rotation[1][2],
        rotation[2][2],
        translation_m[0],
        translation_m[1],
        translation_m[2],
        1.0,
        0.0,
        0.0,
        0.0,
    ]


def get_sw_modules():
    win32.gencache.EnsureModule(SW_MAIN_TYPELIB, 0, 33, 0)
    return (
        win32.gencache.GetModuleForCLSID(IID_IMODELDOC2),
        win32.gencache.GetModuleForCLSID(IID_IASSEMBLYDOC),
        win32.gencache.GetModuleForCLSID(IID_IPARTDOC),
    )


MODELDOC_MODULE, ASSEMBLY_MODULE, PART_MODULE = get_sw_modules()


def as_model_doc(obj):
    return MODELDOC_MODULE.IModelDoc2(obj._oleobj_)


def as_assembly_doc(obj):
    return ASSEMBLY_MODULE.IAssemblyDoc(obj._oleobj_)


def as_part_doc(obj):
    return PART_MODULE.IPartDoc(obj._oleobj_)


def get_or_start_sw():
    try:
        sw = win32.GetActiveObject("SldWorks.Application")
    except Exception:
        sw = win32.Dispatch("SldWorks.Application.33")
    sw.Visible = True
    sw.CommandInProgress = False
    return sw


def close_generated_target_if_open(sw) -> None:
    """Close only this script's generated assembly before replacing it."""
    try:
        sw.CloseDoc(ASM_PATH.name)
        trace(f"closed generated target if open: {ASM_PATH.name}")
    except Exception as error:
        trace(
            "generated target close probe was non-fatal: "
            f"{type(error).__name__}:{error}"
        )


def close_document(sw, model) -> None:
    title = call(model, "GetTitle", "")
    if title:
        try:
            sw.CloseDoc(str(title))
        except Exception:
            pass


def part_path(link: str) -> Path:
    stored_name = PART_PATH_ALIASES.get(link, link)
    return PART_DIR / f"ZEROTH01_LINK_{safe_name(stored_name).upper()}.SLDPRT"


def part_box(sw, path: Path) -> list[float]:
    result = sw.OpenDoc6(str(path), SW_DOC_PART, SW_OPEN_SILENT, "", 0, 0)
    model = first(result)
    if model is None:
        raise RuntimeError(f"cannot open generated part {path}")
    try:
        return [float(value) for value in as_part_doc(model).GetPartBox(True)]
    finally:
        close_document(sw, model)


def import_surface_parts(sw, link_meshes: dict[str, str]) -> list[dict[str, object]]:
    PART_DIR.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, object]] = []
    previous_model_type = int(
        sw.GetUserPreferenceIntegerValue(SW_IMPORT_STL_VRML_MODEL_TYPE)
    )
    previous_units = int(sw.GetUserPreferenceIntegerValue(SW_IMPORT_STL_VRML_UNITS))
    sw.SetUserPreferenceIntegerValue(
        SW_IMPORT_STL_VRML_MODEL_TYPE, SW_IMPORT_AS_SURFACE
    )
    sw.SetUserPreferenceIntegerValue(SW_IMPORT_STL_VRML_UNITS, SW_LENGTH_UNIT_METER)
    try:
        ordered_links = sorted(
            link_meshes,
            key=lambda name: (0 if name == "Torso" else 1, name.lower()),
        )
        for index, link in enumerate(ordered_links, start=1):
            source = MESH_DIR / link_meshes[link]
            target = part_path(link)
            start = time.time()
            import_method = "reuse_existing_surface_part"
            if not target.is_file() or target.stat().st_size < 1024:
                trace(
                    f"surface import {index}/{len(ordered_links)} "
                    f"link={link} mesh={source.name}"
                )
                result = sw.LoadFile4(str(source), "", None, 0)
                model = first(result)
                error = (
                    int(result[1])
                    if isinstance(result, tuple) and len(result) > 1
                    else 0
                )
                if model is None:
                    raise RuntimeError(f"SolidWorks LoadFile4 failed: {source}")
                import_method = "LoadFile4_STL_surface_meter"
                model.SaveAs3(
                    str(target),
                    SW_SAVE_AS_CURRENT_VERSION,
                    SW_SAVE_AS_SILENT,
                )
                close_document(sw, model)
                if not target.is_file() or target.stat().st_size < 1024:
                    raise RuntimeError(f"SolidWorks SaveAs3 failed: {target}")
            else:
                error = 0
            bounds = part_box(sw, target)
            extents = [
                bounds[3] - bounds[0],
                bounds[4] - bounds[1],
                bounds[5] - bounds[2],
            ]
            rows.append(
                {
                    "link": link,
                    "mesh": str(source),
                    "part": str(target),
                    "status": "OK",
                    "method": import_method,
                    "load_error": error,
                    "bytes": target.stat().st_size,
                    "extent_x_m": f"{extents[0]:.9f}",
                    "extent_y_m": f"{extents[1]:.9f}",
                    "extent_z_m": f"{extents[2]:.9f}",
                    "seconds": f"{time.time() - start:.3f}",
                }
            )
    finally:
        sw.SetUserPreferenceIntegerValue(
            SW_IMPORT_STL_VRML_MODEL_TYPE, previous_model_type
        )
        sw.SetUserPreferenceIntegerValue(SW_IMPORT_STL_VRML_UNITS, previous_units)
    return rows


def create_math_transform(sw, values: list[float]):
    utility = sw.GetMathUtility()
    module = win32.gencache.EnsureModule(
        SW_MAIN_TYPELIB,
        0,
        33,
        0,
    )
    typed_utility = module.IMathUtility(utility._oleobj_)
    payload = win32.VARIANT(
        pythoncom.VT_ARRAY | pythoncom.VT_R8,
        [float(value) for value in values],
    )
    return typed_utility.CreateTransform(payload)


def set_component_transform(sw, component, transform) -> float:
    expected = transform_array(transform)
    math_transform = create_math_transform(sw, expected)
    component.Transform2 = math_transform
    try:
        component.SetTransformAndSolve2(math_transform)
    except Exception:
        pass
    actual = [float(value) for value in list(component.Transform2.ArrayData)]
    return max(abs(actual[index] - expected[index]) for index in range(12))


def refresh_assembly_display(model) -> None:
    # Transform2 changes the underlying geometry immediately, but SOLIDWORKS
    # can keep a stale graphics bounding box until UpdateBox/rebuild/redraw.
    # That stale state made a connected assembly look visually exploded even
    # though IComponent2.GetBox already reported the correct URDF positions.
    try:
        model.UpdateBox()
    except Exception:
        pass
    try:
        model.EditRebuild3()
    except Exception:
        pass
    try:
        model.ForceRebuild3(False)
    except Exception:
        pass
    try:
        model.GraphicsRedraw2()
    except Exception:
        pass


def activate_document(sw, model) -> None:
    title = str(call(model, "GetTitle", ""))
    if not title:
        return
    try:
        sw.ActivateDoc3(title, False, 0, 0)
    except Exception:
        try:
            sw.ActivateDoc2(title, False, 0)
        except Exception:
            pass


def add_component(sw, model, asm, link: str, path: Path, transform):
    result = sw.OpenDoc6(str(path), SW_DOC_PART, SW_OPEN_SILENT, "", 0, 0)
    part_model = first(result)
    if part_model is None:
        raise RuntimeError(f"cannot preload {path}")
    activate_document(sw, model)
    component = asm.AddComponent5(str(path), 0, "", False, "", 0.0, 0.0, 0.0)
    if component is None:
        close_document(sw, part_model)
        raise RuntimeError(f"AddComponent5 failed: {path}")
    try:
        component.Name2 = f"ZEROTH01_{safe_name(link)}"
    except Exception:
        pass
    error = set_component_transform(sw, component, transform)
    close_document(sw, part_model)
    activate_document(sw, model)
    return component, error


def save_view(model, view_id: int, path: Path) -> bool:
    try:
        refresh_assembly_display(model)
        model.ShowNamedView2("", view_id)
        model.ViewZoomtofit2()
        model.GraphicsRedraw2()
    except Exception:
        pass
    model.SaveAs3(str(path), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT)
    return path.is_file() and path.stat().st_size > 0


def component_pair(interference) -> tuple[str, str]:
    for getter in (
        lambda: interference.Components,
        lambda: interference.GetComponents(),
    ):
        try:
            components = getter()
            if components:
                names = [str(call(component, "Name2", "")) for component in components]
                if len(names) >= 2:
                    return names[0], names[1]
        except Exception:
            pass
    return "", ""


def interference_probe(asm, pose: str) -> tuple[str, list[dict[str, object]]]:
    try:
        manager_module = win32.gencache.GetModuleForCLSID(
            "{EAE282BD-588A-4C1B-AD99-5FE6081C4585}"
        )
        manager = manager_module.IInterferenceDetectionMgr(
            asm.InterferenceDetectionManager._oleobj_
        )
        manager.TreatCoincidenceAsInterference = False
        manager.IncludeMultibodyPartInterferences = False
        manager.TreatSubAssembliesAsComponents = False
        asm.ToolsCheckInterference2(0, None, False)
        raw_items = list(manager.GetInterferences() or [])
        item_module = win32.gencache.GetModuleForCLSID(
            "{F04EC279-EDF3-4A8E-BF87-E3237CBCCD8C}"
        )
        rows = []
        for index, raw_item in enumerate(raw_items, start=1):
            item = item_module.IInterference(raw_item._oleobj_)
            first_name, second_name = component_pair(item)
            rows.append(
                {
                    "pose": pose,
                    "index": index,
                    "component_a": first_name,
                    "component_b": second_name,
                    "volume_mm3": f"{float(item.Volume) * 1_000_000_000.0:.6f}",
                    "status": "SURFACE_MESH_PROBE_ONLY",
                }
            )
        return "AVAILABLE", rows
    except Exception as error:
        return f"UNAVAILABLE:{type(error).__name__}:{error}", []


def set_pose(sw, model, components, transforms) -> float:
    maximum_error = 0.0
    for link, component in components.items():
        maximum_error = max(
            maximum_error,
            set_component_transform(sw, component, transforms[link]),
        )
    refresh_assembly_display(model)
    return maximum_error


def create_motion_gif(frame_paths: list[Path]) -> bool:
    images = [Image.open(path).convert("P", palette=Image.Palette.ADAPTIVE) for path in frame_paths]
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


def create_assembly_and_review(
    sw,
    link_meshes,
    joints,
    frame_count: int,
    neutral_only: bool = False,
) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    for folder in (ASM_DIR, SNAP_DIR, FRAME_DIR, REPORT_DIR):
        folder.mkdir(parents=True, exist_ok=True)
    raw = first(sw.NewDocument(str(ASM_TEMPLATE), 0, 0, 0))
    if raw is None:
        raise RuntimeError(f"SolidWorks NewDocument failed: {ASM_TEMPLATE}")
    model = as_model_doc(raw)
    asm = as_assembly_doc(raw)
    neutral = forward_kinematics(joints, {})
    components = {}
    component_rows: list[dict[str, object]] = []
    ordered_links = sorted(
        link_meshes,
        key=lambda name: (0 if name == "Torso" else 1, name.lower()),
    )
    for index, link in enumerate(ordered_links, start=1):
        trace(f"assembly component {index}/{len(ordered_links)} link={link}")
        component, error = add_component(
            sw, model, asm, link, part_path(link), neutral[link]
        )
        components[link] = component
        rotation, translation = neutral[link]
        component_rows.append(
            {
                "link": link,
                "component": str(call(component, "Name2", "")),
                "part": str(part_path(link)),
                "x_m": f"{translation[0]:.9f}",
                "y_m": f"{translation[1]:.9f}",
                "z_m": f"{translation[2]:.9f}",
                "transform_readback_max_abs_error": f"{error:.3e}",
                "status": "OK" if error < 1e-8 else "FAIL",
            }
        )

    refresh_assembly_display(model)
    model.ViewZoomtofit2()
    model.SaveAs3(str(ASM_PATH), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT)
    if not ASM_PATH.is_file() or ASM_PATH.stat().st_size < 1024:
        raise RuntimeError(f"SolidWorks assembly save failed: {ASM_PATH}")

    for label, view_id in (
        ("isometric", 7),
        ("solidworks_front", 1),
        ("robot_side", 4),
        ("robot_front", 6),
    ):
        save_view(model, view_id, SNAP_DIR / f"zeroth01_neutral_{label}.png")

    moving = [joint for joint in joints if joint["type"] == "revolute"]
    sweep_rows: list[dict[str, object]] = []
    interference_rows: list[dict[str, object]] = []
    if neutral_only:
        save_view(model, 7, SNAP_DIR / "zeroth01_neutral_final_isometric.png")
        save_view(model, 6, SNAP_DIR / "zeroth01_neutral_final_robot_front.png")
        activate_document(sw, model)
        return (
            component_rows,
            sweep_rows,
            interference_rows,
            [
                {
                    "check": "neutral_only_geometry_review",
                    "status": "INFO",
                    "detail": (
                        "17 geometry-compatible components assembled at URDF "
                        "neutral transforms; full sweep intentionally skipped"
                    ),
                },
                {
                    "check": "assembly_saved_and_left_open",
                    "status": "OK",
                    "detail": str(ASM_PATH),
                },
            ],
        )
    interference_status = "NOT_RUN"
    for joint_index, joint in enumerate(moving, start=1):
        orientations = []
        for sample_name, angle in (
            ("lower", float(joint["lower"])),
            ("zero", 0.0),
            ("upper", float(joint["upper"])),
        ):
            transforms = forward_kinematics(
                joints, {str(joint["name"]): angle}
            )
            maximum_error = set_pose(sw, model, components, transforms)
            child_rotation, child_translation = transforms[str(joint["child"])]
            orientations.append(child_rotation)
            pose_name = f"{joint['name']}:{sample_name}"
            if interference_status == "NOT_RUN" or interference_status == "AVAILABLE":
                status, rows = interference_probe(asm, pose_name)
                interference_status = status
                interference_rows.extend(rows)
            sweep_rows.append(
                {
                    "joint": joint["name"],
                    "sample": sample_name,
                    "angle_rad": f"{angle:.9f}",
                    "angle_deg": f"{math.degrees(angle):.6f}",
                    "child_link": joint["child"],
                    "child_x_m": f"{child_translation[0]:.9f}",
                    "child_y_m": f"{child_translation[1]:.9f}",
                    "child_z_m": f"{child_translation[2]:.9f}",
                    "transform_readback_max_abs_error": f"{maximum_error:.3e}",
                    "solidworks_interference_probe": interference_status,
                    "status": "OK" if maximum_error < 1e-8 else "FAIL",
                }
            )
        observed_rotation = rotation_distance(orientations[0], orientations[-1])
        for row in sweep_rows[-3:]:
            row["lower_to_upper_child_rotation_deg"] = (
                f"{math.degrees(observed_rotation):.6f}"
            )
        trace(
            f"kinematic sweep {joint_index}/{len(moving)} "
            f"joint={joint['name']} rotation_deg={math.degrees(observed_rotation):.3f}"
        )

    # Do not leave frames from an earlier run with a larger frame count; those
    # stale PNGs can be mistaken for current evidence even though the GIF is
    # built only from this run's frame_paths.
    for stale_frame in FRAME_DIR.glob("zeroth01_motion_*.png"):
        stale_frame.unlink()
    frame_paths: list[Path] = []
    safe_summary_path = REPORT_DIR / "mujoco_motion_summary.json"
    safe_limits = {}
    if safe_summary_path.is_file():
        safe_limits = json.loads(safe_summary_path.read_text(encoding="utf-8")).get(
            "single_axis_sampled_safe_limits", {}
        )
    for frame_index in range(frame_count):
        phase = 2.0 * math.pi * frame_index / max(1, frame_count - 1)
        q = {}
        for joint_index, joint in enumerate(moving):
            entry = safe_limits.get(str(joint["name"]), {})
            lower = float(
                entry.get("single_axis_sampled_safe_lower_rad", joint["lower"])
            )
            upper = float(
                entry.get("single_axis_sampled_safe_upper_rad", joint["upper"])
            )
            negative_room = max(0.0, -lower)
            positive_room = max(0.0, upper)
            amplitude = min(
                math.radians(12.0),
                0.65 * negative_room if negative_room else math.radians(4.0),
                0.65 * positive_room if positive_room else math.radians(4.0),
            )
            q[str(joint["name"])] = amplitude * math.sin(
                phase + (joint_index % 4) * math.pi / 2.0
            )
        set_pose(sw, model, components, forward_kinematics(joints, q))
        frame_path = FRAME_DIR / f"zeroth01_motion_{frame_index:03d}.png"
        save_view(model, 6, frame_path)
        frame_paths.append(frame_path)
    gif_ok = create_motion_gif(frame_paths)

    set_pose(sw, model, components, neutral)
    model.SaveAs3(str(ASM_PATH), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT)
    save_view(model, 7, SNAP_DIR / "zeroth01_neutral_final_isometric.png")
    save_view(model, 6, SNAP_DIR / "zeroth01_neutral_final_robot_front.png")
    activate_document(sw, model)

    gate_rows = [
        {
            "check": "solidworks_revision",
            "status": "OK",
            "detail": str(call(sw, "RevisionNumber", "")),
        },
        {
            "check": "surface_part_count",
            "status": "OK" if len(link_meshes) == 17 else "FAIL",
            "detail": f"count={len(link_meshes)}; metre STL imported as SolidWorks surface bodies",
        },
        {
            "check": "assembly_component_count",
            "status": "OK" if len(components) == 17 else "FAIL",
            "detail": f"count={len(components)}",
        },
        {
            "check": "urdf_joint_transform_sweep",
            "status": (
                "OK"
                if len(sweep_rows) == 48
                and all(row["status"] == "OK" for row in sweep_rows)
                else "FAIL"
            ),
            "detail": f"16 joints x lower/zero/upper; rows={len(sweep_rows)}",
        },
        {
            "check": "solidworks_surface_interference_probe",
            "status": "INFO",
            "detail": (
                f"{interference_status}; STL sources are open surface meshes, "
                "so this probe is not used as the RL collision authority"
            ),
        },
        {
            "check": "motion_gif",
            "status": "OK" if gif_ok else "FAIL",
            "detail": str(MOTION_GIF),
        },
        {
            "check": "native_mate_motion_study",
            "status": "NOT_CLAIMED",
            "detail": (
                "The authoritative source is open STL mesh geometry without "
                "stable cylindrical B-Rep mate faces. This deliverable uses "
                "SolidWorks COM forward-kinematic transform sweeps; it does not "
                "claim native motor/mate Motion Study evidence."
            ),
        },
        {
            "check": "assembly_saved_and_left_open",
            "status": (
                "OK"
                if ASM_PATH.is_file() and ASM_PATH.stat().st_size > 1024
                else "FAIL"
            ),
            "detail": str(ASM_PATH),
        },
    ]
    return component_rows, sweep_rows, interference_rows, gate_rows


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Import Zeroth-01 STL links into SolidWorks and run a URDF FK review."
    )
    parser.add_argument("--frame-count", type=int, default=25)
    parser.add_argument(
        "--neutral-only",
        action="store_true",
        help="Build and snapshot only the neutral assembly; skip sweeps/GIF.",
    )
    args = parser.parse_args()
    if args.frame_count < 3:
        raise ValueError("--frame-count must be >= 3")
    for required in (URDF, PART_TEMPLATE, ASM_TEMPLATE):
        if not required.is_file():
            raise FileNotFoundError(required)
    if ROOT.drive.upper() != "E:":
        raise RuntimeError(f"Zeroth-01 work must remain on E:, got {ROOT}")

    TRACE_LOG.unlink(missing_ok=True)
    pythoncom.CoInitialize()
    sw = get_or_start_sw()
    close_generated_target_if_open(sw)
    link_meshes, joints = load_model_data()
    part_rows = import_surface_parts(sw, link_meshes)
    component_rows, sweep_rows, interference_rows, gate_rows = (
        create_assembly_and_review(sw, link_meshes, joints, args.frame_count)
        if not args.neutral_only
        else create_assembly_and_review(
            sw,
            link_meshes,
            joints,
            args.frame_count,
            neutral_only=True,
        )
    )
    write_csv(PART_REPORT, part_rows)
    write_csv(COMPONENT_REPORT, component_rows)
    write_csv(SWEEP_REPORT, sweep_rows)
    write_csv(INTERFERENCE_REPORT, interference_rows)
    write_csv(GATE_REPORT, gate_rows)
    print(f"ASSEMBLY={ASM_PATH}")
    print(f"MOTION_GIF={MOTION_GIF}")
    print(f"PARTS={len(part_rows)}")
    print(f"COMPONENTS={len(component_rows)}")
    print(f"SWEEP_ROWS={len(sweep_rows)}")
    print(f"GATE_REPORT={GATE_REPORT}")


if __name__ == "__main__":
    main()
