from __future__ import annotations

import argparse
import json
from pathlib import Path, PurePosixPath
import sys
import xml.etree.ElementTree as ET


def exact_child(parent: Path, name: str) -> Path | None:
    if not parent.is_dir():
        return None
    for child in parent.iterdir():
        if child.name == name:
            return child
    return None


def resolve_case_exact(base: Path, relative: str) -> tuple[Path | None, str | None]:
    current = base
    for component in PurePosixPath(relative.replace("\\", "/")).parts:
        if component in {"", "."}:
            continue
        if component == "..":
            current = current.parent
            continue
        child = exact_child(current, component)
        if child is None:
            return None, component
        current = child
    return current, None


def urdf_meshes(path: Path) -> list[str]:
    root = ET.parse(path).getroot()
    return sorted(
        {
            mesh.get("filename", "")
            for mesh in root.findall(".//mesh")
            if mesh.get("filename")
        }
    )


def mjcf_meshes(path: Path) -> tuple[Path, list[str]]:
    root = ET.parse(path).getroot()
    compiler = root.find("compiler")
    meshdir = compiler.get("meshdir", ".") if compiler is not None else "."
    base = (path.parent / meshdir).resolve()
    refs = sorted(
        {
            mesh.get("file", "")
            for mesh in root.findall("./asset/mesh")
            if mesh.get("file")
        }
    )
    return base, refs


def check_refs(kind: str, source: Path, base: Path, refs: list[str]) -> list[dict[str, str]]:
    failures: list[dict[str, str]] = []
    for ref in refs:
        resolved, missing_component = resolve_case_exact(base, ref)
        if resolved is None or not resolved.is_file():
            failures.append(
                {
                    "kind": kind,
                    "source": str(source),
                    "reference": ref,
                    "missing_or_case_mismatched_component": missing_component or "",
                }
            )
    return failures


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate case-exact, Linux-portable mesh paths in a URDF/MJCF pair."
    )
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--mjcf", type=Path, required=True)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()

    urdf = args.urdf.resolve()
    mjcf = args.mjcf.resolve()
    package_root = urdf.parents[2]

    def display(path: Path) -> str:
        try:
            return path.relative_to(package_root).as_posix()
        except ValueError:
            return str(path)

    failures = check_refs("urdf", urdf, urdf.parent, urdf_meshes(urdf))
    mjcf_base, mjcf_refs = mjcf_meshes(mjcf)
    failures.extend(check_refs("mjcf", mjcf, mjcf_base, mjcf_refs))

    payload = {
        "schema": "zeroth01.rl_package_portability.v1",
        "urdf": display(urdf),
        "mjcf": display(mjcf),
        "urdf_mesh_reference_count": len(urdf_meshes(urdf)),
        "mjcf_mesh_reference_count": len(mjcf_refs),
        "case_exact_mesh_paths": not failures,
        "failures": failures,
        "overall": "PASS" if not failures else "FAIL",
    }
    rendered = json.dumps(payload, indent=2, ensure_ascii=False) + "\n"
    if args.report:
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(rendered, encoding="utf-8", newline="\n")
    sys.stdout.write(rendered)
    return 0 if not failures else 1


if __name__ == "__main__":
    raise SystemExit(main())
