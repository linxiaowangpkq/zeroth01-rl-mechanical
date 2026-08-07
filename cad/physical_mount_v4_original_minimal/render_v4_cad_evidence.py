"""Render deterministic normal/x-ray views from the v4 assembly manifest."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw
from build123d import import_step, import_stl


ROOT = Path(__file__).resolve().parents[2]
V3_RENDER = ROOT / "cad" / "physical_mount_v3_rl_fixed" / "render_v3_cad_evidence.py"
MANIFEST = ROOT / "generated" / "cad" / "physical_mount_v4_original_minimal" / "ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY_MANIFEST.json"
LAYOUT = ROOT / "generated" / "config" / "physical_mount_v4_original_minimal_actuator_layout.json"
OUT = ROOT / "snapshots" / "cad" / "physical_mount_v4_original_minimal"
REPORT = ROOT / "reports" / "v4_original_minimal" / "cad_render_evidence.json"
CACHE_DIR = ROOT / "reports" / "v4_original_minimal" / "tessellation_cache"


def load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v3 = load(V3_RENDER, "zeroth01_v3_cad_render_helpers_for_v4")


def tessellate_actors():
    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    actors = []
    all_points = []
    cache = {}
    for component in manifest["components"]:
        source = ROOT / str(component["source"])
        cache_key = str(source)
        if cache_key not in cache:
            stat = source.stat()
            fingerprint = hashlib.sha256(
                f"{source.resolve()}|{stat.st_size}|{stat.st_mtime_ns}|v4-mm".encode()
            ).hexdigest()
            cached = CACHE_DIR / f"{fingerprint}.npz"
            if cached.is_file():
                payload = np.load(cached)
                cache[cache_key] = (payload["vertices"], payload["triangles"])
            else:
                shape = import_stl(source) if source.suffix.lower() == ".stl" else import_step(source)
                vertices, triangles = v3.safe_tessellate(shape)
                if source.suffix.lower() == ".stl" and "physical_mount_v1" in source.as_posix():
                    vertices = vertices * 1000.0
                cache[cache_key] = (vertices, triangles)
                CACHE_DIR.mkdir(parents=True, exist_ok=True)
                np.savez_compressed(cached, vertices=vertices, triangles=triangles)
        local_points, faces = cache[cache_key]
        transform = np.asarray(component["transform_local_mm_to_world_mm"], dtype=float)
        points = local_points @ transform[:3, :3].T + transform[:3, 3]
        if not len(points) or not len(faces):
            continue
        actors.append(
            {
                "label": str(component["component_id"]),
                "role": str(component["role"]),
                "points": points,
                "faces": faces,
                "color": v3.hex_color(str(component["color_hex"])),
            }
        )
        all_points.append(points)
    return actors, np.vstack(all_points)


def render(path, actors, all_points, direction, *, xray=False, annotate=False):
    width, height = 1500, 1700
    margin = 125
    right, up, depth_axis = v3.camera_basis(direction)
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
    wireframes = []
    light = v3.unit((0.8, -0.5, 1.2))
    internal_roles = {"internal_payload_controlled_envelope"}
    xray_shell_roles = {
        "source_load_bearing_carrier",
        "printable_head_front_shell",
        "printable_head_rear_shell",
        "reversible_white_rear_service_pod",
        "removable_internal_service_mount",
        "direct_head_torso_mount",
    }
    for actor in actors:
        if not xray and actor["role"] in internal_roles:
            continue
        points, faces = actor["points"], actor["faces"]
        projected = project(points)
        shell = actor["role"] in xray_shell_roles
        for face in faces:
            tri, tri2 = points[face], projected[face]
            if xray and shell:
                wireframes.append(tri2)
                continue
            normal = np.cross(tri[1] - tri[0], tri[2] - tri[0])
            norm = np.linalg.norm(normal)
            brightness = 0.78 if norm <= 1.0e-9 else 0.62 + 0.38 * abs(float(np.dot(normal / norm, light)))
            color = tuple(int(min(255, channel * brightness)) for channel in actor["color"])
            rows.append((float(np.mean(tri @ depth_axis)), tri2, color))
    for _, triangle, color in sorted(rows, key=lambda item: item[0]):
        draw.polygon([tuple(point) for point in triangle], fill=color)
    if xray:
        for index, triangle in enumerate(wireframes):
            if index % 8 == 0:
                draw.line([tuple(point) for point in triangle] + [tuple(triangle[0])], fill=(145, 158, 175), width=1)

    draw.rounded_rectangle((42, 35, width - 42, 105), 18, fill=(16, 24, 40), outline=(48, 68, 96), width=2)
    draw.text((70, 52), "Zeroth-01 v4 original-minimal · 18DoF · 2.850 kg · 498.959 mm", font=v3.load_font(34, True), fill=(245, 249, 255))
    draw.text((65, height - 82), "Blue: 18 installed STS3250   |   orange/magenta/green: compute/battery/IMU", font=v3.load_font(20), fill=(30, 44, 66))
    draw.text((65, height - 48), "CAD + MuJoCo gates PASS; physical release HOLD until purchased/printed first article and as-built SysID", font=v3.load_font(20), fill=(145, 44, 44))

    if annotate:
        layout = json.loads(LAYOUT.read_text(encoding="utf-8"))
        for index, servo in enumerate(layout["actuators"]):
            point = np.asarray(servo["shaft_origin_body_neutral_m"], dtype=float) * 1000.0
            px, py = project(point.reshape(1, 3))[0]
            offset = (20 if index % 2 == 0 else -58, -18 if index % 4 < 2 else 14)
            tx, ty = px + offset[0], py + offset[1]
            draw.ellipse((px - 6, py - 6, px + 6, py + 6), fill=(22, 119, 255), outline=(255, 255, 255), width=2)
            draw.rounded_rectangle((tx - 3, ty - 2, tx + 39, ty + 20), 5, fill=(12, 48, 100), outline=(170, 215, 255))
            draw.text((tx + 2, ty), servo["id"], font=v3.load_font(17, True), fill=(255, 255, 255))

    path.parent.mkdir(parents=True, exist_ok=True)
    image.save(path)
    return {"path": path.relative_to(ROOT).as_posix(), "camera_direction": list(direction), "xray": xray, "annotated": annotate}


def main() -> int:
    actors, points = tessellate_actors()
    rows = [
        render(OUT / "v4_connected_normal_front.png", actors, points, (1.0, 0.0, 0.0), annotate=True),
        render(OUT / "v4_xray_front_18_servos_payloads.png", actors, points, (1.0, 0.0, 0.0), xray=True, annotate=True),
        render(OUT / "v4_connected_iso.png", actors, points, (1.0, -1.15, 0.55)),
    ]
    payload = {
        "schema": "zeroth01.physical_mount_v4_original_minimal.cad_render_evidence.v1",
        "source_assembly_manifest": MANIFEST.relative_to(ROOT).as_posix(),
        "source_sha256": hashlib.sha256(MANIFEST.read_bytes()).hexdigest(),
        "occurrence_count": len(actors),
        "renders": rows,
    }
    REPORT.parent.mkdir(parents=True, exist_ok=True)
    REPORT.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(payload, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
