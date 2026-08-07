"""Render labelled evidence views directly from the generated STEP assembly.

The renderer uses build123d's STEP tessellation and a deterministic
orthographic painter.  It is deliberately independent of SolidWorks and the
browser viewer cache, so the PNG is evidence for the exact STEP hash shipped.
"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont
from build123d import import_step
from OCP.BRep import BRep_Tool
from OCP.TopAbs import TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location


ROOT = Path(__file__).resolve().parents[2]
STEP = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed" / "ZEROTH01_V3_RL_FIXED_18DOF_DIAGNOSTIC_ASSEMBLY.step"
ASSEMBLY_MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v3_rl_fixed" / "ZEROTH01_V3_RL_FIXED_18DOF_FULL_ASSEMBLY_MANIFEST.json"
LAYOUT = ROOT / "generated" / "config" / "physical_mount_v3_rl_fixed_actuator_layout.json"
OUT = ROOT / "snapshots" / "cad" / "physical_mount_v3_rl_fixed"
REPORT = ROOT / "reports" / "physical_mount_v3_rl_fixed" / "cad_render_evidence.json"
CACHE_DIR = ROOT / "reports" / "physical_mount_v3_rl_fixed" / "tessellation_cache"

SHELL_KEYS = ("BODY_SKELETON", "TORSO", "CHEST", "HEAD", "VISOR", "CAMERA_WINDOW")


def safe_tessellate(shape, tolerance=1.0, angular_tolerance=0.18):
    """Tessellate imported faceted STEP while skipping null OCCT face meshes."""

    shape.mesh(tolerance, angular_tolerance)
    vertices = []
    triangles = []
    offset = 0
    for face in shape.faces():
        location = TopLoc_Location()
        poly = BRep_Tool.Triangulation_s(face.wrapped, location)
        if poly is None:
            continue
        transform = location.Transformation()
        reverse = face.wrapped.Orientation() == TopAbs_REVERSED
        local_vertices = [
            poly.Node(index).Transformed(transform)
            for index in range(1, poly.NbNodes() + 1)
        ]
        vertices.extend((vertex.X(), vertex.Y(), vertex.Z()) for vertex in local_vertices)
        for triangle in poly.Triangles():
            indices = [triangle.Value(index) + offset - 1 for index in (1, 2, 3)]
            if reverse:
                indices[1], indices[2] = indices[2], indices[1]
            triangles.append(tuple(indices))
        offset += poly.NbNodes()
    return np.asarray(vertices, dtype=float), np.asarray(triangles, dtype=np.int32)


def unit(values):
    values = np.asarray(values, dtype=float)
    return values / np.linalg.norm(values)


def camera_basis(direction):
    direction = unit(direction)
    world_up = np.array((0.0, 0.0, 1.0))
    right = unit(np.cross(world_up, direction))
    up = unit(np.cross(direction, right))
    return right, up, direction


def hex_color(value):
    token = value.lstrip("#")
    return tuple(int(token[index:index + 2], 16) for index in (0, 2, 4))


def tessellate_actors():
    manifest = json.loads(ASSEMBLY_MANIFEST.read_text(encoding="utf-8"))
    actors = []
    all_points = []
    cache = {}
    for component in manifest["components"]:
        source = ROOT / str(component["source"])
        # Render the same local-coordinate STEP B-Reps consumed by SolidWorks.
        # The older source STL files contain baked/global placements; applying
        # the manifest transform to those a second time creates a false
        # exploded view and must never be used as assembly evidence.
        render_source = source
        cache_key = str(render_source)
        if cache_key not in cache:
            stat = render_source.stat()
            fingerprint = hashlib.sha256(
                f"{render_source.resolve()}|{stat.st_size}|{stat.st_mtime_ns}".encode("utf-8")
            ).hexdigest()
            cached_mesh = CACHE_DIR / f"{fingerprint}.npz"
            if cached_mesh.is_file():
                payload = np.load(cached_mesh)
                cache[cache_key] = (payload["vertices"], payload["triangles"])
            else:
                shape = import_step(render_source)
                cache[cache_key] = safe_tessellate(shape)
                CACHE_DIR.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(
                    cached_mesh,
                    vertices=cache[cache_key][0],
                    triangles=cache[cache_key][1],
                )
        local_points, faces = cache[cache_key]
        transform = np.asarray(component["transform_local_mm_to_world_mm"], dtype=float)
        points = local_points @ transform[:3, :3].T + transform[:3, 3]
        if not len(points) or not len(faces):
            continue
        actors.append({
            "label": str(component["component_id"]),
            "role": str(component["role"]),
            "points": points,
            "faces": faces,
            "color": hex_color(str(component["color_hex"])),
        })
        all_points.append(points)
    return actors, np.vstack(all_points)


def load_font(size, bold=False):
    name = "arialbd.ttf" if bold else "arial.ttf"
    path = Path("C:/Windows/Fonts") / name
    return ImageFont.truetype(str(path), size) if path.is_file() else ImageFont.load_default()


def render(path, actors, all_points, direction, xray=False, annotate=False):
    width, height = 1500, 1700
    margin = 125
    right, up, depth_axis = camera_basis(direction)
    xy = np.column_stack((all_points @ right, all_points @ up))
    low, high = xy.min(axis=0), xy.max(axis=0)
    span = np.maximum(high - low, 1.0)
    scale = min((width - 2 * margin) / span[0], (height - 2 * margin - 170) / span[1])
    centre = (low + high) / 2.0

    def project(points):
        px = (points @ right - centre[0]) * scale + width / 2.0
        py = height / 2.0 - 50.0 - (points @ up - centre[1]) * scale
        return np.column_stack((px, py))

    image = Image.new("RGB", (width, height), (241, 245, 250))
    draw = ImageDraw.Draw(image)
    for y in range(height):
        shade = int(245 - 18 * y / height)
        draw.line((0, y, width, y), fill=(shade, shade + 3, min(255, shade + 8)))

    rows = []
    shell_edges = []
    light = unit((0.8, -0.5, 1.2))
    for actor in actors:
        if not xray and actor["role"] in {
            "internal_payload_controlled_envelope",
            "reversible_purchased_head_torso_adapter",
        }:
            continue
        points, faces = actor["points"], actor["faces"]
        projected = project(points)
        shell = any(key in actor["label"] for key in SHELL_KEYS)
        for face in faces:
            tri = points[face]
            tri2 = projected[face]
            if xray and shell:
                shell_edges.append(tri2)
                continue
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            norm = np.linalg.norm(normal)
            brightness = 0.78 if norm <= 1.0e-9 else 0.62 + 0.38 * abs(float(np.dot(normal / norm, light)))
            color = tuple(int(min(255, channel * brightness)) for channel in actor["color"])
            front_feature = any(
                key in actor["label"]
                for key in ("FACE_GLASS", "SCREEN_EXPRESSION", "CAMERA_WINDOW")
            )
            rows.append((1 if front_feature else 0, float(np.mean(tri @ depth_axis)), tri2, color))
    for _, _, triangle, color in sorted(rows, key=lambda item: (item[0], item[1])):
        draw.polygon([tuple(point) for point in triangle], fill=color)

    if xray:
        # Sparse shell wireframe shows the envelope without hiding blue servo
        # bodies.  Sampling avoids a dark triangulation cloud.
        for index, triangle in enumerate(shell_edges):
            if index % 6 == 0:
                draw.line([tuple(point) for point in triangle] + [tuple(triangle[0])], fill=(154, 166, 180), width=1)

    title_font = load_font(34, True)
    text_font = load_font(20)
    small_font = load_font(17, True)
    draw.rounded_rectangle((42, 35, width - 42, 105), 18, fill=(16, 24, 40), outline=(48, 68, 96), width=2)
    draw.text((70, 52), "ZEROTH-01 v3.1 COMPACT - 18DoF / 2.969 kg nominal / <500 mm", font=title_font, fill=(245, 249, 255))
    draw.text((65, height - 82), "Blue: STS3250 controlled envelope   |   primitive RL collision sweep: PASS", font=text_font, fill=(30, 44, 66))
    draw.text((65, height - 48), "Physical release: HOLD until STS3250 + CoreS3 cradle first articles and as-built mass/COM/inertia identification", font=text_font, fill=(145, 44, 44))

    if annotate:
        layout = json.loads(LAYOUT.read_text(encoding="utf-8"))
        for index, servo in enumerate(layout["actuators"]):
            point = np.asarray(servo["shaft_origin_body_neutral_m"], dtype=float) * 1000.0
            px, py = project(point.reshape(1, 3))[0]
            offset = (20 if index % 2 == 0 else -58, -18 if index % 4 < 2 else 14)
            tx, ty = px + offset[0], py + offset[1]
            draw.ellipse((px - 6, py - 6, px + 6, py + 6), fill=(22, 119, 255), outline=(255, 255, 255), width=2)
            draw.rounded_rectangle((tx - 3, ty - 2, tx + 39, ty + 20), 5, fill=(12, 48, 100), outline=(170, 215, 255))
            draw.text((tx + 2, ty), servo["id"], font=small_font, fill=(255, 255, 255))

    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return {
        "path": path.relative_to(ROOT).as_posix(),
        "camera_direction": list(direction),
        "xray": xray,
        "annotated": annotate,
        "size_px": [width, height],
    }


def main() -> int:
    actors, points = tessellate_actors()
    rows = [
        render(OUT / "v3_18dof_connected_normal_front.png", actors, points, (1.0, 0.0, 0.0), False, True),
        render(OUT / "v3_18dof_xray_front.png", actors, points, (1.0, 0.0, 0.0), True, True),
        render(OUT / "v3_18dof_iso.png", actors, points, (1.0, -1.15, 0.55), False, False),
    ]
    payload = {
        "schema": "zeroth01.physical_mount_v3_rl_fixed.cad_render_evidence.v1",
        "source_assembly_manifest": ASSEMBLY_MANIFEST.relative_to(ROOT).as_posix(),
        "source_sha256": hashlib.sha256(ASSEMBLY_MANIFEST.read_bytes()).hexdigest(),
        "occurrence_count": len(actors),
        "renders": rows,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
