from __future__ import annotations

import argparse
import importlib.util
from pathlib import Path
import shutil
import sys


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
ROUND_SCRIPT = THIS_FILE.with_name("create_solidworks_round_v1_review.py")
SOURCE_BASE_PARTS = ROOT / "generated" / "solidworks" / "parts"
SOURCE_ROUND_PARTS = ROOT / "generated" / "solidworks" / "round_v1" / "parts"
SOURCE_SWEEP = ROOT / "reports" / "solidworks_round_v1_kinematic_sweep.csv"
SOURCE_MOTION = (
    ROOT
    / "snapshots"
    / "solidworks"
    / "round_v1"
    / "zeroth01_round_v1_solidworks_motion.gif"
)
DEFAULT_OUTPUT = (
    ROOT / "generated" / "solidworks" / "portable_flat_round_v1"
)


def load_round_module():
    spec = importlib.util.spec_from_file_location(
        "zeroth01_solidworks_round_portable", ROUND_SCRIPT
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import SolidWorks generator: {ROUND_SCRIPT}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def copy_flat_parts(output: Path, expected_count: int) -> int:
    output.mkdir(parents=True, exist_ok=True)
    sources = [
        path
        for path in (
            sorted(SOURCE_BASE_PARTS.glob("*.SLDPRT"))
            + sorted(SOURCE_ROUND_PARTS.glob("*.SLDPRT"))
        )
        if not path.name.startswith("~$")
    ]
    folded: dict[str, Path] = {}
    for source in sources:
        key = source.name.casefold()
        previous = folded.get(key)
        if previous is not None and previous.name != source.name:
            raise RuntimeError(
                f"case-insensitive filename collision: {previous.name}, {source.name}"
            )
        folded[key] = source
        shutil.copy2(source, output / source.name)
    if len(sources) != expected_count:
        raise RuntimeError(
            f"expected {expected_count} source part files, found {len(sources)}"
        )
    return len(sources)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Rebuild the round-v1 SolidWorks assembly with every referenced "
            "SLDPRT in the same portable folder."
        )
    )
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    output = args.output_dir.resolve()

    round_module = load_round_module()
    expected_count = 17 + len(round_module.OVERLAYS) + 3
    copied = copy_flat_parts(output, expected_count)
    previews = output / "previews"
    previews.mkdir(parents=True, exist_ok=True)
    portable_motion = previews / SOURCE_MOTION.name
    shutil.copy2(SOURCE_MOTION, portable_motion)

    assembly = output / "OPEN_FIRST_ZEROTH01_ROUND_V1_WITH_STS3250.SLDASM"
    round_module.SW_ROOT = output
    round_module.SW_PART_DIR = output
    round_module.ASM_PATH = assembly
    round_module.SNAP_DIR = previews
    round_module.FRAME_DIR = previews / "motion_frames"
    round_module.MOTION_GIF = portable_motion
    round_module.REPORT_DIR = output
    round_module.PART_REPORT = output / "solidworks_portable_part_import.csv"
    round_module.COMPONENT_REPORT = (
        output / "solidworks_portable_component_manifest.csv"
    )
    # Geometry, attachments, and transforms are identical to the already
    # completed 48-pose sweep; refresh-only rebuilds references and images.
    round_module.SWEEP_REPORT = SOURCE_SWEEP
    round_module.GATE_REPORT = output / "solidworks_portable_gate.json"
    round_module.TRACE_LOG = output / "solidworks_portable_trace.log"
    round_module.base.PART_DIR = output
    round_module.base.ASM_DIR = output

    previous_argv = sys.argv
    try:
        sys.argv = [str(ROUND_SCRIPT), "--refresh-only"]
        round_module.main()
    finally:
        sys.argv = previous_argv

    if not assembly.is_file() or assembly.stat().st_size < 1024:
        raise RuntimeError(f"portable assembly was not created: {assembly}")
    print(f"PORTABLE_PART_COUNT={copied}")
    print(f"PORTABLE_ASSEMBLY={assembly}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
