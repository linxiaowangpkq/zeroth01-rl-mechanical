from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import time

import pythoncom
import win32com.client as win32
from PIL import Image


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
BASE_SCRIPT = THIS_FILE.with_name("create_solidworks_kinematic_review.py")
ROUND_URDF = ROOT / "generated" / "urdf" / "zeroth01_rl_round_v1.urdf"
PHASE_CONFIG = (
    ROOT
    / "generated"
    / "config"
    / "zeroth01_sts3250_mount_phase.json"
)
SERVO_AXIS_REPORT = ROOT / "reports" / "round_v1_servo_axis_alignment.csv"
MUJOCO_GATE = ROOT / "reports" / "mujoco_round_v1_gate.json"
STEP_PART_DIR = ROOT / "generated" / "cad" / "round_v1" / "parts"
SERVO_STEP = (
    ROOT
    / "source_assets"
    / "vendor"
    / "sts3250"
    / "FEETECH_STS3250.step"
)

SW_ROOT = ROOT / "generated" / "solidworks" / "round_v1"
SW_PART_DIR = SW_ROOT / "parts"
ASM_PATH = SW_ROOT / "OPEN_FIRST_ZEROTH01_ROUND_V1_WITH_STS3250.SLDASM"
SNAP_DIR = ROOT / "snapshots" / "solidworks" / "round_v1"
FRAME_DIR = SNAP_DIR / "motion_frames"
MOTION_GIF = SNAP_DIR / "zeroth01_round_v1_solidworks_motion.gif"
REPORT_DIR = ROOT / "reports"
PART_REPORT = REPORT_DIR / "solidworks_round_v1_part_import.csv"
COMPONENT_REPORT = REPORT_DIR / "solidworks_round_v1_component_manifest.csv"
SWEEP_REPORT = REPORT_DIR / "solidworks_round_v1_kinematic_sweep.csv"
GATE_REPORT = REPORT_DIR / "solidworks_round_v1_gate.json"
TRACE_LOG = REPORT_DIR / "solidworks_round_v1_trace.log"

SW_DOC_PART = 1
SW_SAVE_AS_CURRENT_VERSION = 0
SW_SAVE_AS_SILENT = 1
SW_OPEN_SILENT = 1
SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING = 579
SW_IMPORT_NEUTRAL_AS_MULTIBODY_PART = 2

CREAM = (0.909804, 0.823529, 0.701961)
TAN = (0.717647, 0.529412, 0.368627)
DARK = (0.164706, 0.176471, 0.196078)

OVERLAYS = [
    (
        "ROUND_CHEST_FRONT",
        "ZEROTH01_ROUND_V1_CHEST_FRONT.step",
        "ZEROTH01_ROUND_V1_CHEST_FRONT.SLDPRT",
        "Torso",
        CREAM,
    ),
    (
        "ROUND_CHEST_BACK",
        "ZEROTH01_ROUND_V1_CHEST_BACK.step",
        "ZEROTH01_ROUND_V1_CHEST_BACK.SLDPRT",
        "Torso",
        CREAM,
    ),
    (
        "ROUND_HEAD_FRONT",
        "ZEROTH01_ROUND_V1_HEAD_FRONT.step",
        "ZEROTH01_ROUND_V1_HEAD_FRONT.SLDPRT",
        "Torso",
        CREAM,
    ),
    (
        "ROUND_HEAD_BACK",
        "ZEROTH01_ROUND_V1_HEAD_BACK.step",
        "ZEROTH01_ROUND_V1_HEAD_BACK.SLDPRT",
        "Torso",
        CREAM,
    ),
    (
        "ROUND_PELVIS_FRONT",
        "ZEROTH01_ROUND_V1_PELVIS_FRONT.step",
        "ZEROTH01_ROUND_V1_PELVIS_FRONT.SLDPRT",
        "Torso",
        CREAM,
    ),
    (
        "ROUND_PELVIS_BACK",
        "ZEROTH01_ROUND_V1_PELVIS_BACK.step",
        "ZEROTH01_ROUND_V1_PELVIS_BACK.SLDPRT",
        "Torso",
        CREAM,
    ),
    (
        "ROUND_MUZZLE",
        "ZEROTH01_ROUND_V1_MUZZLE_BADGE.step",
        "ZEROTH01_ROUND_V1_MUZZLE_BADGE.SLDPRT",
        "Torso",
        TAN,
    ),
    (
        "ROUND_VISOR",
        "ZEROTH01_ROUND_V1_VISOR_BADGE.step",
        "ZEROTH01_ROUND_V1_VISOR_BADGE.SLDPRT",
        "Torso",
        DARK,
    ),
    (
        "ROUND_LEFT_SOLE",
        "ZEROTH01_ROUND_V1_LEFT_SOLE.step",
        "ZEROTH01_ROUND_V1_LEFT_SOLE.SLDPRT",
        "foot_left",
        DARK,
    ),
    (
        "ROUND_RIGHT_SOLE",
        "ZEROTH01_ROUND_V1_RIGHT_SOLE.step",
        "ZEROTH01_ROUND_V1_RIGHT_SOLE.SLDPRT",
        "foot_right",
        DARK,
    ),
]


def load_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import helper: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


base = load_module(BASE_SCRIPT, "zeroth01_solidworks_base")
base.URDF = ROUND_URDF


def trace(message: str) -> None:
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with TRACE_LOG.open("a", encoding="utf-8") as stream:
        stream.write(f"{stamp} {message}\n")
    print(message, flush=True)


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    data = rows or [{"status": "EMPTY"}]
    fields: list[str] = []
    for row in data:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(data)


def close_document(sw, model) -> None:
    title = str(base.call(model, "GetTitle", ""))
    if title:
        try:
            sw.CloseDoc(title)
        except Exception:
            pass


def close_target_if_open(sw, path: Path) -> None:
    try:
        sw.CloseDoc(path.name)
    except Exception:
        pass


def import_step_part(
    sw,
    source: Path,
    target: Path,
    *,
    force: bool,
) -> dict[str, object]:
    target.parent.mkdir(parents=True, exist_ok=True)
    reuse = (
        target.is_file()
        and target.stat().st_size > 1024
        and target.stat().st_mtime >= source.stat().st_mtime
        and not force
    )
    start = time.time()
    error = 0
    if not reuse:
        close_target_if_open(sw, target)
        # STEP imports require IImportStepData when 3D Interconnect is
        # enabled. Passing None works for the legacy STL path but raises
        # DISP_E_TYPEMISMATCH for STEP in SOLIDWORKS 2025.
        import_data = sw.GetImportFileData(str(source))
        if import_data is None:
            raise RuntimeError(
                f"SolidWorks GetImportFileData failed: {source}"
            )
        result = sw.LoadFile4(str(source), "r", import_data, 0)
        raw_model = base.first(result)
        if isinstance(result, tuple) and len(result) > 1:
            try:
                error = int(result[1])
            except Exception:
                error = 0
        if raw_model is None:
            raise RuntimeError(f"SolidWorks STEP import failed: {source}")
        model = base.as_model_doc(raw_model)
        document_type = int(base.call(model, "GetType", 0))
        if document_type != SW_DOC_PART:
            close_document(sw, model)
            raise RuntimeError(
                f"STEP did not import as a part ({document_type}): {source}"
            )
        save_code = int(
            model.SaveAs3(
                str(target), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT
            )
        )
        close_document(sw, model)
        if not target.is_file() or target.stat().st_size < 1024:
            raise RuntimeError(f"SolidWorks SLDPRT save failed: {target}")
        method = "LoadFile4_STEP_to_native_SLDPRT"
    else:
        method = "reuse_current_native_SLDPRT"
        save_code = 0

    bounds = base.part_box(sw, target)
    extents_mm = [
        (bounds[3] - bounds[0]) * 1000.0,
        (bounds[4] - bounds[1]) * 1000.0,
        (bounds[5] - bounds[2]) * 1000.0,
    ]
    return {
        "source_step": str(source),
        "native_part": str(target),
        "method": method,
        "load_error": error,
        "save_code": save_code,
        "bytes": target.stat().st_size,
        "extent_x_mm": f"{extents_mm[0]:.6f}",
        "extent_y_mm": f"{extents_mm[1]:.6f}",
        "extent_z_mm": f"{extents_mm[2]:.6f}",
        "seconds": f"{time.time() - start:.3f}",
        "status": "PASS",
    }


def rotation_z(angle_rad: float) -> list[list[float]]:
    cosine = math.cos(angle_rad)
    sine = math.sin(angle_rad)
    return [
        [cosine, -sine, 0.0],
        [sine, cosine, 0.0],
        [0.0, 0.0, 1.0],
    ]


def servo_transforms(
    joints: list[dict[str, object]],
    link_transforms: dict[str, tuple[list[list[float]], list[float]]],
) -> dict[str, tuple[list[list[float]], list[float]]]:
    phase_payload = json.loads(PHASE_CONFIG.read_text(encoding="utf-8"))
    phases = phase_payload.get("joint_mount_phase", {})
    flip_x = [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]]
    result = {}
    for joint in joints:
        if joint["type"] not in {"revolute", "continuous"}:
            continue
        name = str(joint["name"])
        child = str(joint["child"])
        rotation, translation = link_transforms[child]
        axis_local = [float(value) for value in joint["axis"]]
        positive = (
            base.mat_mul(rotation, flip_x)
            if axis_local[2] < 0.0
            else rotation
        )
        entry = phases.get(name, {})
        sign = int(entry.get("output_axis_sign", 1))
        phase_deg = float(entry.get("phase_deg", 0.0))
        oriented = positive if sign == 1 else base.mat_mul(positive, flip_x)
        oriented = base.mat_mul(
            oriented, rotation_z(math.radians(phase_deg))
        )
        result[name] = (oriented, translation)
    return result


def set_material(component, color: tuple[float, float, float]) -> bool:
    payload = win32.VARIANT(
        pythoncom.VT_ARRAY | pythoncom.VT_R8,
        [
            float(color[0]),
            float(color[1]),
            float(color[2]),
            0.35,
            0.75,
            0.25,
            0.35,
            0.0,
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


def try_shaded(model) -> str:
    for method in ("ViewDisplayShaded", "ViewDisplayShaded2"):
        try:
            getattr(model, method)()
            return method
        except Exception:
            continue
    return "UNAVAILABLE"


def save_view(model, view_id: int, path: Path) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    try_shaded(model)
    return base.save_view(model, view_id, path)


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
            duration=100,
            loop=0,
            optimize=False,
        )
    finally:
        for image in images:
            image.close()
    return MOTION_GIF.is_file() and MOTION_GIF.stat().st_size > 0


def set_round_pose(
    sw,
    model,
    joints: list[dict[str, object]],
    link_components: dict[str, object],
    overlay_components: dict[str, object],
    servo_components: dict[str, object],
    q: dict[str, float],
) -> float:
    transforms = base.forward_kinematics(joints, q)
    maximum_error = 0.0
    for link, component in link_components.items():
        maximum_error = max(
            maximum_error,
            base.set_component_transform(sw, component, transforms[link]),
        )
    for label, _, _, attachment, _ in OVERLAYS:
        transform = (
            transforms[attachment]
            if attachment in {"foot_left", "foot_right"}
            else (base.IDENTITY_R, [0.0, 0.0, 0.0])
        )
        maximum_error = max(
            maximum_error,
            base.set_component_transform(
                sw, overlay_components[label], transform
            ),
        )
    for name, transform in servo_transforms(joints, transforms).items():
        maximum_error = max(
            maximum_error,
            base.set_component_transform(
                sw, servo_components[name], transform
            ),
        )
    base.refresh_assembly_display(model)
    return maximum_error


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Create the native SolidWorks round-v1 review assembly with "
            "explicit FEETECH STS3250 reference components."
        )
    )
    parser.add_argument("--frame-count", type=int, default=12)
    parser.add_argument("--force-part-import", action="store_true")
    parser.add_argument(
        "--refresh-only",
        action="store_true",
        help=(
            "Rebuild neutral assembly/screenshots after geometry-only changes "
            "and reuse the existing 48-row transform sweep and motion GIF."
        ),
    )
    args = parser.parse_args()
    if args.frame_count < 3:
        raise ValueError("--frame-count must be at least 3")

    pythoncom.CoInitialize()
    sw = base.get_or_start_sw()
    SW_ROOT.mkdir(parents=True, exist_ok=True)
    SW_PART_DIR.mkdir(parents=True, exist_ok=True)
    SNAP_DIR.mkdir(parents=True, exist_ok=True)
    FRAME_DIR.mkdir(parents=True, exist_ok=True)

    part_rows: list[dict[str, object]] = []
    previous_structure_mapping = int(
        sw.GetUserPreferenceIntegerValue(
            SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING
        )
    )
    sw.SetUserPreferenceIntegerValue(
        SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING,
        SW_IMPORT_NEUTRAL_AS_MULTIBODY_PART,
    )
    try:
        for index, (_, step_name, part_name, _, _) in enumerate(
            OVERLAYS, start=1
        ):
            trace(f"STEP import {index}/{len(OVERLAYS) + 1}: {step_name}")
            row = import_step_part(
                sw,
                STEP_PART_DIR / step_name,
                SW_PART_DIR / part_name,
                force=bool(args.force_part_import),
            )
            row["role"] = "round_v1_printed_overlay"
            part_rows.append(row)
        trace(
            f"STEP import {len(OVERLAYS) + 1}/{len(OVERLAYS) + 1}: "
            "STS3250"
        )
        servo_part = SW_PART_DIR / "FEETECH_STS3250.SLDPRT"
        row = import_step_part(
            sw,
            SERVO_STEP,
            servo_part,
            force=bool(args.force_part_import),
        )
        row["role"] = "vendor_reference_not_extra_inertial_mass"
        part_rows.append(row)
    finally:
        sw.SetUserPreferenceIntegerValue(
            SW_IMPORT_NEUTRAL_ASSEMBLY_STRUCTURE_MAPPING,
            previous_structure_mapping,
        )
    write_csv(PART_REPORT, part_rows)

    close_target_if_open(sw, ASM_PATH)
    raw = base.first(sw.NewDocument(str(base.ASM_TEMPLATE), 0, 0, 0))
    if raw is None:
        raise RuntimeError(
            f"SolidWorks NewDocument failed: {base.ASM_TEMPLATE}"
        )
    model = base.as_model_doc(raw)
    asm = base.as_assembly_doc(raw)
    link_meshes, joints = base.load_model_data()
    neutral = base.forward_kinematics(joints, {})

    component_rows: list[dict[str, object]] = []
    link_components: dict[str, object] = {}
    ordered_links = sorted(
        link_meshes,
        key=lambda name: (0 if name == "Torso" else 1, name.lower()),
    )
    for index, link in enumerate(ordered_links, start=1):
        trace(f"link component {index}/{len(ordered_links)}: {link}")
        component, error = base.add_component(
            sw, model, asm, link, base.part_path(link), neutral[link]
        )
        link_components[link] = component
        material_set = set_material(component, CREAM)
        component_rows.append(
            {
                "role": "source_link_surface",
                "name": link,
                "component": str(base.call(component, "Name2", "")),
                "part": str(base.part_path(link)),
                "attachment": link,
                "material_set": material_set,
                "transform_readback_max_abs_error": f"{error:.3e}",
                "status": "PASS" if error < 1e-8 else "FAIL",
            }
        )

    overlay_components: dict[str, object] = {}
    for index, (label, _, part_name, attachment, color) in enumerate(
        OVERLAYS, start=1
    ):
        trace(f"round overlay {index}/{len(OVERLAYS)}: {label}")
        transform = (
            neutral[attachment]
            if attachment in {"foot_left", "foot_right"}
            else (base.IDENTITY_R, [0.0, 0.0, 0.0])
        )
        component, error = base.add_component(
            sw,
            model,
            asm,
            label,
            SW_PART_DIR / part_name,
            transform,
        )
        overlay_components[label] = component
        material_set = set_material(component, color)
        component_rows.append(
            {
                "role": "round_v1_printed_overlay",
                "name": label,
                "component": str(base.call(component, "Name2", "")),
                "part": str(SW_PART_DIR / part_name),
                "attachment": attachment,
                "material_set": material_set,
                "transform_readback_max_abs_error": f"{error:.3e}",
                "status": "PASS" if error < 1e-8 else "FAIL",
            }
        )

    servo_components: dict[str, object] = {}
    neutral_servos = servo_transforms(joints, neutral)
    for index, (name, transform) in enumerate(
        neutral_servos.items(), start=1
    ):
        trace(f"STS3250 component {index}/{len(neutral_servos)}: {name}")
        component, error = base.add_component(
            sw, model, asm, f"STS3250_{name}", servo_part, transform
        )
        servo_components[name] = component
        material_set = set_material(component, DARK)
        component_rows.append(
            {
                "role": "vendor_servo_reference",
                "name": name,
                "component": str(base.call(component, "Name2", "")),
                "part": str(servo_part),
                "attachment": name,
                "material_set": material_set,
                "transform_readback_max_abs_error": f"{error:.3e}",
                "status": "PASS" if error < 1e-8 else "FAIL",
            }
        )

    display_method = try_shaded(model)
    base.refresh_assembly_display(model)
    model.ViewZoomtofit2()
    save_code = int(
        model.SaveAs3(
            str(ASM_PATH), SW_SAVE_AS_CURRENT_VERSION, SW_SAVE_AS_SILENT
        )
    )
    if not ASM_PATH.is_file() or ASM_PATH.stat().st_size < 1024:
        raise RuntimeError(f"SolidWorks assembly save failed: {ASM_PATH}")
    write_csv(COMPONENT_REPORT, component_rows)

    snapshot_results = {
        "isometric": save_view(
            model, 7, SNAP_DIR / "zeroth01_round_v1_isometric.png"
        ),
        "robot_front": save_view(
            model, 6, SNAP_DIR / "zeroth01_round_v1_robot_front.png"
        ),
        "robot_side": save_view(
            model, 4, SNAP_DIR / "zeroth01_round_v1_robot_side.png"
        ),
    }

    moving = [
        joint
        for joint in joints
        if joint["type"] in {"revolute", "continuous"}
    ]
    sweep_rows: list[dict[str, object]] = []
    if args.refresh_only:
        if not SWEEP_REPORT.is_file():
            raise FileNotFoundError(
                "--refresh-only requires an existing transform sweep: "
                f"{SWEEP_REPORT}"
            )
        with SWEEP_REPORT.open(
            "r", encoding="utf-8-sig", newline=""
        ) as stream:
            sweep_rows = list(csv.DictReader(stream))
        if len(sweep_rows) != 48 or not all(
            row.get("status") == "PASS" for row in sweep_rows
        ):
            raise RuntimeError(
                "--refresh-only found invalid prior transform evidence"
            )
        gif_ok = MOTION_GIF.is_file() and MOTION_GIF.stat().st_size > 0
        trace(
            "refresh-only: reused 48-row transform sweep and motion GIF; "
            "attachment transforms/topology are unchanged"
        )
    else:
        for joint_index, joint in enumerate(moving, start=1):
            rotations = []
            for sample, angle in (
                ("lower", float(joint["lower"])),
                ("zero", 0.0),
                ("upper", float(joint["upper"])),
            ):
                q = {str(joint["name"]): angle}
                transforms = base.forward_kinematics(joints, q)
                maximum_error = set_round_pose(
                    sw,
                    model,
                    joints,
                    link_components,
                    overlay_components,
                    servo_components,
                    q,
                )
                rotation, translation = transforms[str(joint["child"])]
                rotations.append(rotation)
                sweep_rows.append(
                    {
                        "joint": joint["name"],
                        "sample": sample,
                        "angle_rad": f"{angle:.9f}",
                        "angle_deg": f"{math.degrees(angle):.6f}",
                        "child_link": joint["child"],
                        "child_x_m": f"{translation[0]:.9f}",
                        "child_y_m": f"{translation[1]:.9f}",
                        "child_z_m": f"{translation[2]:.9f}",
                        "transform_readback_max_abs_error": (
                            f"{maximum_error:.3e}"
                        ),
                        "collision_evidence": str(MUJOCO_GATE),
                        "status": (
                            "PASS" if maximum_error < 1e-8 else "FAIL"
                        ),
                    }
                )
            observed = base.rotation_distance(rotations[0], rotations[-1])
            for row in sweep_rows[-3:]:
                row["lower_to_upper_child_rotation_deg"] = (
                    f"{math.degrees(observed):.6f}"
                )
            trace(
                f"kinematic sweep {joint_index}/{len(moving)}: "
                f"{joint['name']} {math.degrees(observed):.3f} deg"
            )
        write_csv(SWEEP_REPORT, sweep_rows)

        for path in FRAME_DIR.glob("zeroth01_round_v1_motion_*.png"):
            path.unlink()
        frames: list[Path] = []
        for frame_index in range(args.frame_count):
            phase = (
                2.0
                * math.pi
                * frame_index
                / max(1, args.frame_count - 1)
            )
            q: dict[str, float] = {}
            for joint_index, joint in enumerate(moving):
                lower = float(joint["lower"])
                upper = float(joint["upper"])
                amplitude = min(
                    math.radians(12.0),
                    0.65 * max(0.0, -lower)
                    if lower < 0.0
                    else math.radians(4.0),
                    0.65 * max(0.0, upper)
                    if upper > 0.0
                    else math.radians(4.0),
                )
                q[str(joint["name"])] = amplitude * math.sin(
                    phase + (joint_index % 4) * math.pi / 2.0
                )
            set_round_pose(
                sw,
                model,
                joints,
                link_components,
                overlay_components,
                servo_components,
                q,
            )
            frame = (
                FRAME_DIR
                / f"zeroth01_round_v1_motion_{frame_index:03d}.png"
            )
            save_view(model, 6, frame)
            frames.append(frame)
            trace(f"motion frame {frame_index + 1}/{args.frame_count}")
        gif_ok = create_motion_gif(frames)

    final_error = set_round_pose(
        sw,
        model,
        joints,
        link_components,
        overlay_components,
        servo_components,
        {},
    )
    try_shaded(model)
    model.ShowNamedView2("", 7)
    model.ViewZoomtofit2()
    model.GraphicsRedraw2()
    model.SaveAs3(
        str(ASM_PATH),
        SW_SAVE_AS_CURRENT_VERSION,
        SW_SAVE_AS_SILENT,
    )

    axis_rows = []
    if SERVO_AXIS_REPORT.is_file():
        with SERVO_AXIS_REPORT.open(
            "r", encoding="utf-8-sig", newline=""
        ) as stream:
            axis_rows = list(csv.DictReader(stream))
    mujoco_gate = (
        json.loads(MUJOCO_GATE.read_text(encoding="utf-8"))
        if MUJOCO_GATE.is_file()
        else {}
    )
    expected_component_count = 17 + len(OVERLAYS) + len(moving)
    gate = {
        "schema": "zeroth01.solidworks.round_v1.gate.v1",
        "solidworks_revision": str(base.call(sw, "RevisionNumber", "")),
        "assembly": str(ASM_PATH),
        "assembly_bytes": ASM_PATH.stat().st_size,
        "assembly_initial_save_code": save_code,
        "display_mode_command": display_method,
        "native_brep_part_count": len(part_rows),
        "component_count": len(component_rows),
        "expected_component_count": expected_component_count,
        "source_link_component_count": len(link_components),
        "round_overlay_component_count": len(overlay_components),
        "explicit_sts3250_component_count": len(servo_components),
        "transform_gate": (
            "PASS"
            if all(row["status"] == "PASS" for row in component_rows)
            and all(row["status"] == "PASS" for row in sweep_rows)
            and final_error < 1e-8
            else "FAIL"
        ),
        "servo_axis_gate": (
            "PASS"
            if len(axis_rows) == 16
            and all(row.get("gate") == "PASS" for row in axis_rows)
            else "FAIL"
        ),
        "mujoco_collision_motion_gate": mujoco_gate.get(
            "overall", "MISSING"
        ),
        "snapshots": {
            key: {
                "path": str(
                    SNAP_DIR
                    / {
                        "isometric": "zeroth01_round_v1_isometric.png",
                        "robot_front": "zeroth01_round_v1_robot_front.png",
                        "robot_side": "zeroth01_round_v1_robot_side.png",
                    }[key]
                ),
                "gate": "PASS" if value else "FAIL",
            }
            for key, value in snapshot_results.items()
        },
        "motion_gif": str(MOTION_GIF),
        "motion_gif_gate": "PASS" if gif_ok else "FAIL",
        "motion_evidence_reused_after_geometry_only_refresh": bool(
            args.refresh_only
        ),
        "native_mate_motion_study": (
            "NOT_CLAIMED: the 17 upstream link meshes are imported surface "
            "bodies without stable mate faces; CLI FK drives Transform2 while "
            "the new shells and STS3250 are native B-Rep components"
        ),
        "hardware_interference_signoff": (
            "BLOCKED_PENDING_PHYSICAL_SERVO_CLOCKING_CABLE_FASTENER_AND_"
            "TOLERANCE_VALIDATION"
        ),
    }
    gate["overall_review_gate"] = (
        "PASS_WITH_HARDWARE_LIMITATIONS"
        if (
            gate["component_count"] == expected_component_count
            and gate["native_brep_part_count"] == len(OVERLAYS) + 1
            and gate["transform_gate"] == "PASS"
            and gate["servo_axis_gate"] == "PASS"
            and gate["mujoco_collision_motion_gate"] == "PASS"
            and gate["motion_gif_gate"] == "PASS"
            and all(
                value["gate"] == "PASS"
                for value in gate["snapshots"].values()
            )
        )
        else "FAIL"
    )
    GATE_REPORT.write_text(
        json.dumps(gate, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(gate, ensure_ascii=False, indent=2))
    if gate["overall_review_gate"] == "FAIL":
        raise SystemExit(2)


if __name__ == "__main__":
    main()
