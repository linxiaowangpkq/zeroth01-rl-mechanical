from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Writer
from OCP.StlAPI import StlAPI_Reader
from OCP.TopoDS import TopoDS_Shape
from OCP.gp import gp_Pnt, gp_Trsf


ROOT = Path(__file__).resolve().parents[2]
CAD_ROOT = ROOT / "generated" / "cad" / "physical_mount_v1"
STL_ROOTS = {
    "skeleton": CAD_ROOT / "skeleton",
    "servos": CAD_ROOT / "servos",
}
STEP_ROOT = CAD_ROOT / "step"
REPORT = (
    ROOT
    / "reports"
    / "physical_mount_v1"
    / "stl_to_step_conversion.json"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def convert(source: Path, target: Path, *, force: bool) -> dict[str, object]:
    target.parent.mkdir(parents=True, exist_ok=True)
    if (
        not force
        and target.is_file()
        and target.stat().st_size > 1024
        and target.stat().st_mtime >= source.stat().st_mtime
    ):
        return {
            "source": str(source),
            "target": str(target),
            "method": "reuse_current_step",
            "bytes": target.stat().st_size,
            "sha256": sha256(target),
            "status": "PASS",
        }

    started = time.monotonic()
    shape = TopoDS_Shape()
    reader = StlAPI_Reader()
    if not reader.Read(shape, str(source)):
        raise RuntimeError(f"OCC STL read failed: {source}")
    if shape.IsNull():
        raise RuntimeError(f"OCC STL read produced null shape: {source}")

    # The Zeroth source STL coordinates are metres because they are consumed
    # directly by URDF. STEP files are authored in millimetres so SolidWorks
    # imports them at the same physical size.
    scale = gp_Trsf()
    scale.SetScale(gp_Pnt(0.0, 0.0, 0.0), 1000.0)
    scaled = BRepBuilderAPI_Transform(shape, scale, True).Shape()
    if scaled.IsNull():
        raise RuntimeError(f"OCC scale produced null shape: {source}")

    Interface_Static.SetCVal_s("write.step.schema", "AP214")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    writer = STEPControl_Writer()
    transfer = writer.Transfer(scaled, STEPControl_AsIs)
    if transfer != IFSelect_RetDone:
        raise RuntimeError(
            f"OCC STEP transfer failed: {source}; status={transfer}"
        )
    temporary = target.with_suffix(".tmp.step")
    temporary.unlink(missing_ok=True)
    written = writer.Write(str(temporary))
    if written != IFSelect_RetDone:
        temporary.unlink(missing_ok=True)
        raise RuntimeError(
            f"OCC STEP write failed: {target}; status={written}"
        )
    if not temporary.is_file() or temporary.stat().st_size < 1024:
        temporary.unlink(missing_ok=True)
        raise RuntimeError(f"OCC STEP file missing or empty: {target}")
    temporary.replace(target)
    return {
        "source": str(source),
        "target": str(target),
        "method": "OCP_StlAPI_triangular_faces_scale_m_to_mm_STEP_AP214",
        "bytes": target.stat().st_size,
        "sha256": sha256(target),
        "seconds": time.monotonic() - started,
        "status": "PASS",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--force", action="store_true")
    parser.add_argument("--limit", type=int, default=0)
    args = parser.parse_args()

    inputs: list[tuple[str, Path]] = []
    for kind, directory in STL_ROOTS.items():
        inputs.extend((kind, path) for path in sorted(directory.glob("*.stl")))
    if args.limit > 0:
        inputs = inputs[: args.limit]
    if not inputs:
        raise RuntimeError("No physical-mount STL files found")

    rows: list[dict[str, object]] = []
    for index, (kind, source) in enumerate(inputs, start=1):
        target = STEP_ROOT / kind / f"{source.stem}.step"
        print(
            f"STEP conversion {index}/{len(inputs)}: {kind}/{source.name}",
            flush=True,
        )
        row = convert(source, target, force=args.force)
        row["kind"] = kind
        rows.append(row)

    report = {
        "schema": "zeroth01.physical_mount_v1.stl_to_step.v1",
        "input_units": "metre",
        "step_units": "millimetre",
        "scale": 1000.0,
        "file_count": len(rows),
        "overall": (
            "PASS"
            if len(rows) == (36 if args.limit == 0 else len(inputs))
            and all(row["status"] == "PASS" for row in rows)
            else "FAIL"
        ),
        "files": rows,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(
        json.dumps(report, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(f"overall={report['overall']} report={REPORT}", flush=True)
    return 0 if report["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
