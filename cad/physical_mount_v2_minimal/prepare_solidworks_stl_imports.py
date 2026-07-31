"""Prepare metre-authored STL copies for deterministic SolidWorks import."""

from __future__ import annotations

import hashlib
import json
from pathlib import Path

from vtkmodules.vtkCommonTransforms import vtkTransform
from vtkmodules.vtkFiltersGeneral import vtkTransformPolyDataFilter
from vtkmodules.vtkIOGeometry import vtkSTLReader, vtkSTLWriter


ROOT = Path(__file__).resolve().parents[2]
SOURCE_ROOT = ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "parts"
OUTPUT_ROOT = (
    ROOT / "generated" / "cad" / "physical_mount_v2_minimal" / "solidworks_import_m"
)
MANIFEST = ROOT / "reports" / "physical_mount_v2_minimal" / "component_manifest.json"
REPORT = ROOT / "reports" / "physical_mount_v2_minimal" / "solidworks_stl_scale_gate.json"


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    rows = []
    parts = json.loads(MANIFEST.read_text(encoding="utf-8"))["parts"]
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    for index, row in enumerate(parts, start=1):
        key = str(row["key"])
        source = SOURCE_ROOT / f"{key}.stl"
        target = OUTPUT_ROOT / f"{key}.stl"
        reader = vtkSTLReader()
        reader.SetFileName(str(source))
        reader.Update()
        transform = vtkTransform()
        transform.Scale(0.001, 0.001, 0.001)
        apply = vtkTransformPolyDataFilter()
        apply.SetInputConnection(reader.GetOutputPort())
        apply.SetTransform(transform)
        apply.Update()
        writer = vtkSTLWriter()
        writer.SetFileName(str(target))
        writer.SetFileTypeToBinary()
        writer.SetInputConnection(apply.GetOutputPort())
        writer.Write()
        bounds = apply.GetOutput().GetBounds()
        extent_m = max(bounds[1] - bounds[0], bounds[3] - bounds[2], bounds[5] - bounds[4])
        gate = target.is_file() and target.stat().st_size > 84 and 0.005 <= extent_m <= 0.25
        print(f"[{index:02d}/{len(parts):02d}] {key} extent_m={extent_m:.6f}", flush=True)
        rows.append(
            {
                "key": key,
                "source": source.relative_to(ROOT).as_posix(),
                "target": target.relative_to(ROOT).as_posix(),
                "source_units": "millimetre",
                "target_units": "metre",
                "scale": 0.001,
                "maximum_extent_m": extent_m,
                "bytes": target.stat().st_size,
                "sha256": _sha256(target),
                "gate": "PASS" if gate else "FAIL",
            }
        )
    payload = {
        "schema": "zeroth01.physical_mount_v2_minimal.solidworks_stl_scale_gate.v1",
        "rows": rows,
        "overall": "PASS" if all(row["gate"] == "PASS" for row in rows) else "FAIL",
    }
    REPORT.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")
    print(REPORT)
    return 0 if payload["overall"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())
