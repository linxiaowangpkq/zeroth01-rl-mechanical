from __future__ import annotations

import argparse
import csv
import hashlib
import json
import shutil
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
SOURCE_REPO = ROOT / "upstream" / "zeroth-sim"
SOURCE_URDF_REVISION = "33b0553bd085ff6360495497a8e86afaa801785d"
SOURCE_URDF_REPO_PATH = "sim/resources/stompymicro/robot.urdf"
SOURCE_MESH_DIR = ROOT / "source_assets" / "stompymicro" / "meshes"
MAPPING_PATH = ROOT / "config" / "mesh_name_map.json"
DEFAULT_OUTPUT = ROOT / "generated" / "urdf" / "zeroth01_rl_reference.urdf"
DEFAULT_MESH_DIR = DEFAULT_OUTPUT.parent / "meshes"
DEFAULT_MANIFEST = ROOT / "reports" / "generated_urdf_manifest.csv"
ROBOT_NAME = "zeroth01_rl_reference_16dof_geometry_matched"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_mapping() -> dict[str, str]:
    payload = json.loads(MAPPING_PATH.read_text(encoding="utf-8"))
    mapping = payload["target_to_downloaded"]
    if len(mapping) != 17:
        raise ValueError(f"expected 17 mesh mappings, got {len(mapping)}")
    return {str(target): str(source) for target, source in mapping.items()}


def load_source_urdf() -> ET.Element:
    result = subprocess.run(
        [
            "git",
            "-c",
            f"safe.directory={SOURCE_REPO.as_posix()}",
            "-C",
            str(SOURCE_REPO),
            "show",
            f"{SOURCE_URDF_REVISION}:{SOURCE_URDF_REPO_PATH}",
        ],
        check=True,
        capture_output=True,
    )
    return ET.fromstring(result.stdout)


def _validate_tree(root: ET.Element) -> None:
    links = [item.get("name", "") for item in root.findall("link")]
    joints = [item.get("name", "") for item in root.findall("joint")]
    if len(links) != len(set(links)):
        raise ValueError("duplicate link names")
    if len(joints) != len(set(joints)):
        raise ValueError("duplicate joint names")
    link_set = set(links)
    child_links: set[str] = set()
    revolute_count = 0
    for joint in root.findall("joint"):
        parent = joint.find("parent")
        child = joint.find("child")
        if parent is None or child is None:
            raise ValueError(f"joint {joint.get('name')} lacks parent/child")
        parent_name = parent.get("link", "")
        child_name = child.get("link", "")
        if parent_name not in link_set or child_name not in link_set:
            raise ValueError(f"joint {joint.get('name')} references a missing link")
        if child_name in child_links:
            raise ValueError(f"link {child_name} has more than one parent")
        child_links.add(child_name)
        if joint.get("type") in {"revolute", "continuous"}:
            revolute_count += 1
            axis = joint.find("axis")
            limit = joint.find("limit")
            if axis is None or limit is None:
                raise ValueError(f"moving joint {joint.get('name')} lacks axis/limit")
            lower = float(limit.get("lower", "nan"))
            upper = float(limit.get("upper", "nan"))
            if not lower < upper:
                raise ValueError(f"invalid limit at {joint.get('name')}: {lower}, {upper}")
    roots = link_set - child_links
    if roots != {"base"}:
        raise ValueError(f"expected root link 'base', got {sorted(roots)}")
    if revolute_count != 16:
        raise ValueError(f"expected 16 revolute joints, got {revolute_count}")


def gen_urdf() -> ET.Element:
    root = load_source_urdf()
    root.set("name", ROBOT_NAME)

    typo_joint = root.find("./joint[@name='righ_elbow_yaw']")
    if typo_joint is None:
        if root.find("./joint[@name='right_elbow_yaw']") is None:
            raise ValueError("expected upstream right elbow joint was not found")
    else:
        typo_joint.set("name", "right_elbow_yaw")

    mapping = load_mapping()
    seen: set[str] = set()
    for mesh in root.findall(".//mesh"):
        source_name = Path(mesh.get("filename", "").replace("\\", "/")).name
        if source_name not in mapping:
            raise ValueError(f"URDF mesh has no frozen mapping: {source_name}")
        mesh.set("filename", f"meshes/{source_name}")
        seen.add(source_name)
    missing = set(mapping) - seen
    if missing:
        raise ValueError(f"mapped meshes are not referenced by URDF: {sorted(missing)}")

    root.insert(
        0,
        ET.Comment(
            "Derived from official Zeroth stompymicro commit 33b0553, the last "
            "URDF revision geometrically compatible with the published Drive STL "
            "bundle. Frames, axes, masses and inertias are preserved. The later "
            "43c5baa frames are intentionally not mixed with these older meshes."
        ),
    )
    _validate_tree(root)
    return root


def prepare_meshes(output_dir: Path) -> list[dict[str, str]]:
    mapping = load_mapping()
    output_dir.mkdir(parents=True, exist_ok=True)
    rows: list[dict[str, str]] = []
    for target_name, downloaded_name in sorted(mapping.items()):
        source = SOURCE_MESH_DIR / downloaded_name
        target = output_dir / target_name
        if not source.is_file():
            raise FileNotFoundError(source)
        # On case-insensitive filesystems copy2() preserves an already-existing
        # directory entry's spelling.  Remove a case-only alias first so a
        # Linux consumer sees the exact filename referenced by the URDF.
        for existing in output_dir.iterdir():
            if (
                existing.is_file()
                and existing.name.casefold() == target_name.casefold()
                and existing.name != target_name
            ):
                existing.unlink()
        shutil.copy2(source, target)
        rows.append(
            {
                "target_name": target_name,
                "downloaded_name": downloaded_name,
                "source_sha256": sha256(source),
                "output_sha256": sha256(target),
                "bytes": str(target.stat().st_size),
            }
        )
    return rows


def write_urdf(output: Path) -> None:
    root = gen_urdf()
    ET.indent(root, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def write_manifest(path: Path, rows: list[dict[str, str]], output: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fields = [
        "target_name",
        "downloaded_name",
        "source_sha256",
        "output_sha256",
        "bytes",
        "urdf_sha256",
    ]
    urdf_digest = sha256(output)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({**row, "urdf_sha256": urdf_digest})


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Generate the owned Zeroth-01 RL reference URDF and mapped meshes."
    )
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    parser.add_argument("--mesh-dir", type=Path, default=None)
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    args = parser.parse_args()

    output = args.output.resolve()
    mesh_dir = args.mesh_dir.resolve() if args.mesh_dir else output.parent / "meshes"
    write_urdf(output)
    rows = prepare_meshes(mesh_dir)
    write_manifest(args.manifest.resolve(), rows, output)
    print(f"URDF={output}")
    print(f"MESH_DIR={mesh_dir}")
    print(f"MESH_COUNT={len(rows)}")
    print(f"MANIFEST={args.manifest.resolve()}")


if __name__ == "__main__":
    main()
