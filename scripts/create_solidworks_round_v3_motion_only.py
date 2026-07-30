from __future__ import annotations

import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import time

import pythoncom


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
REVIEW_SCRIPT = THIS_FILE.with_name("create_solidworks_round_v1_review.py")
MANIFEST = ROOT / "reports" / "solidworks_round_v1_component_manifest.csv"
SWEEP_REPORT = ROOT / "reports" / "solidworks_round_v1_kinematic_sweep.csv"
TRANSMISSION_REPORT = (
    ROOT / "reports" / "solidworks_round_v1_transmission_semantics.csv"
)
GATE_REPORT = ROOT / "reports" / "solidworks_round_v3_motion_only_gate.json"
TRACE_LOG = ROOT / "reports" / "solidworks_round_v3_motion_only_trace.log"
SW_DOC_ASSEMBLY = 2
SW_OPEN_SILENT = 1


def load_review_module():
    spec = importlib.util.spec_from_file_location(
        "zeroth01_round_v3_review", REVIEW_SCRIPT
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {REVIEW_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def trace(message: str) -> None:
    timestamp = time.strftime("%Y-%m-%d %H:%M:%S")
    line = f"{timestamp} {message}"
    print(line, flush=True)
    with TRACE_LOG.open("a", encoding="utf-8") as stream:
        stream.write(line + "\n")


def first(value):
    if isinstance(value, tuple):
        return value[0] if value else None
    return value


def validate_reused_evidence(path: Path, expected_rows: int) -> None:
    if not path.is_file():
        raise FileNotFoundError(path)
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))
    if len(rows) != expected_rows or any(
        row.get("status") != "PASS" for row in rows
    ):
        raise RuntimeError(
            f"cannot reuse invalid evidence: {path} "
            f"rows={len(rows)} failures="
            f"{sum(row.get('status') != 'PASS' for row in rows)}"
        )


def component_name(component) -> str:
    for accessor in (
        lambda: str(component.Name2),
        lambda: str(component.GetSelectByIDString()),
    ):
        try:
            value = accessor()
            if value:
                return value.split("/")[-1]
        except Exception:
            continue
    return ""


def main() -> None:
    parser = argparse.ArgumentParser(
        description=(
            "Open the saved 57-component Zeroth-01 round-v3 SOLIDWORKS "
            "assembly and render a bounded motion GIF while reusing the "
            "unchanged-mechanism 48-row transform/transmission evidence."
        )
    )
    parser.add_argument("--frame-count", type=int, default=8)
    args = parser.parse_args()
    if args.frame_count < 4 or args.frame_count > 16:
        raise ValueError("--frame-count must be between 4 and 16")

    review = load_review_module()
    validate_reused_evidence(SWEEP_REPORT, 48)
    validate_reused_evidence(TRANSMISSION_REPORT, 48)
    collision_gate = json.loads(
        review.MUJOCO_GATE.read_text(encoding="utf-8")
    )
    if collision_gate.get("overall") != "PASS":
        raise RuntimeError(
            f"cannot render motion against a failed collision gate: "
            f"{review.MUJOCO_GATE}"
        )
    if not MANIFEST.is_file():
        raise FileNotFoundError(MANIFEST)

    TRACE_LOG.parent.mkdir(parents=True, exist_ok=True)
    TRACE_LOG.write_text("", encoding="utf-8")
    started = time.perf_counter()
    pythoncom.CoInitialize()
    sw = review.base.get_or_start_sw()
    try:
        # Close only the task-owned target so the file is reopened in its
        # saved neutral pose.  No unrelated user document is touched.
        try:
            sw.CloseDoc(review.ASM_PATH.name)
        except Exception:
            pass
        opened = sw.OpenDoc6(
            str(review.ASM_PATH),
            SW_DOC_ASSEMBLY,
            SW_OPEN_SILENT,
            "",
            0,
            0,
        )
        raw_model = first(opened)
        if raw_model is None:
            raise RuntimeError(f"cannot open {review.ASM_PATH}")
        model = review.base.as_model_doc(raw_model)
        assembly = review.base.as_assembly_doc(raw_model)
        try:
            sw.ActivateDoc3(review.ASM_PATH.name, True, 0, 0)
        except Exception:
            pass

        components = list(assembly.GetComponents(False) or [])
        by_name = {
            component_name(component): component
            for component in components
            if component_name(component)
        }
        with MANIFEST.open(
            "r", encoding="utf-8-sig", newline=""
        ) as stream:
            rows = list(csv.DictReader(stream))

        link_components = {}
        overlay_components = {}
        servo_components = {}
        missing = []
        for row in rows:
            expected_name = row["component"]
            component = by_name.get(expected_name)
            if component is None:
                missing.append(expected_name)
                continue
            role = row["role"]
            if role == "source_link_surface":
                link_components[row["name"]] = component
            elif role.startswith("diagnostic_blue_"):
                servo_components[row["name"]] = component
            else:
                overlay_components[row["name"]] = component
        if missing:
            raise RuntimeError(
                "missing saved assembly components: " + ", ".join(missing)
            )
        if (
            len(link_components) != 17
            or len(overlay_components) != 24
            or len(servo_components) != 16
        ):
            raise RuntimeError(
                "unexpected component mapping "
                f"links={len(link_components)} "
                f"overlays={len(overlay_components)} "
                f"servos={len(servo_components)}"
            )
        trace(
            "opened saved assembly: "
            f"{len(link_components)} links + {len(overlay_components)} "
            f"overlays + {len(servo_components)} blue servo instances"
        )

        _, joints = review.base.load_model_data()
        moving = [
            joint
            for joint in joints
            if joint["type"] in {"revolute", "continuous"}
        ]
        review.try_shaded(model)
        # The source Zeroth coordinate convention is Y-up.  SOLIDWORKS'
        # stock isometric view assumes Z-up and makes the standing robot look
        # as if it were lying down, so use the validated upright front view.
        model.ShowNamedView2("", 6)
        model.ViewZoomtofit2()

        review.FRAME_DIR.mkdir(parents=True, exist_ok=True)
        for old_frame in review.FRAME_DIR.glob(
            "zeroth01_round_v3_motion_*.png"
        ):
            old_frame.unlink()
        frames = []
        maximum_error = 0.0
        frame_timings = []
        for frame_index in range(args.frame_count):
            frame_started = time.perf_counter()
            phase = (
                2.0
                * math.pi
                * frame_index
                / max(1, args.frame_count - 1)
            )
            q = {}
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
            maximum_error = max(
                maximum_error,
                review.set_round_pose(
                    sw,
                    model,
                    joints,
                    link_components,
                    overlay_components,
                    servo_components,
                    q,
                    fast_display_refresh=True,
                ),
            )
            frame = (
                review.FRAME_DIR
                / f"zeroth01_round_v3_motion_{frame_index:03d}.png"
            )
            if not review.save_view(model, 6, frame):
                raise RuntimeError(f"failed to render {frame}")
            frames.append(frame)
            elapsed = time.perf_counter() - frame_started
            frame_timings.append(elapsed)
            trace(
                f"motion frame {frame_index + 1}/{args.frame_count} "
                f"completed in {elapsed:.1f}s"
            )

        if not review.create_motion_gif(frames):
            raise RuntimeError("motion GIF creation failed")

        # Return the live review document to neutral, but do not overwrite the
        # already-saved neutral assembly.
        maximum_error = max(
            maximum_error,
            review.set_round_pose(
                sw,
                model,
                joints,
                link_components,
                overlay_components,
                servo_components,
                {},
                fast_display_refresh=True,
            ),
        )
        elapsed_total = time.perf_counter() - started
        gate = {
            "schema": "zeroth01_solidworks_round_v3_motion_only_gate_v1",
            "status": (
                "PASS"
                if review.MOTION_GIF.is_file()
                and review.MOTION_GIF.stat().st_size > 0
                and maximum_error < 1e-8
                else "FAIL"
            ),
            "assembly": str(review.ASM_PATH),
            "motion_gif": str(review.MOTION_GIF),
            "frame_count": len(frames),
            "frame_seconds": frame_timings,
            "elapsed_seconds": elapsed_total,
            "component_count": len(components),
            "source_link_component_count": len(link_components),
            "round_overlay_component_count": len(overlay_components),
            "diagnostic_blue_servo_component_count": len(
                servo_components
            ),
            "transform_readback_max_abs_error": maximum_error,
            "reused_unchanged_mechanism_sweep": str(SWEEP_REPORT),
            "reused_unchanged_transmission_sweep": str(
                TRANSMISSION_REPORT
            ),
            "collision_evidence": str(review.MUJOCO_GATE),
            "native_motion_study": False,
            "motion_method": (
                "SOLIDWORKS IComponent2.Transform2 FK visualization; "
                "collision truth remains the validated URDF/MuJoCo chain"
            ),
        }
        GATE_REPORT.write_text(
            json.dumps(gate, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        trace(
            f"motion-only gate {gate['status']} in {elapsed_total:.1f}s: "
            f"{review.MOTION_GIF}"
        )
        if gate["status"] != "PASS":
            raise RuntimeError(json.dumps(gate, ensure_ascii=False))
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
