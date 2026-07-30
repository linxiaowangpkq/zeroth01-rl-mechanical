from __future__ import annotations

import csv
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
SNAPSHOT_DIR = ROOT / "snapshots" / "solidworks" / "round_v1"
IDENTITY_CONFIG = ROOT / "config" / "round_v2_component_identity.json"
SERVO_REPORT = ROOT / "reports" / "round_v1_servo_axis_alignment.csv"

FRONT_SOURCE = (
    SNAPSHOT_DIR / "zeroth01_round_v3_16_blue_servos_xray_front.png"
)
SERVO_OUTPUT = (
    SNAPSHOT_DIR / "zeroth01_round_v3_16_blue_servos_annotated_front.png"
)
ELECTRONICS_OUTPUT = (
    SNAPSHOT_DIR / "zeroth01_round_v3_electronics_annotated_front.png"
)
SHOULDER_OUTPUT = (
    SNAPSHOT_DIR / "zeroth01_round_v3_shoulder_4_blue_servos_closeup.png"
)

# Calibrated against the orthographic SolidWorks front view.  The generated
# image remains centered by ViewZoomtofit2; +X is image-right and +Z is up.
IMAGE_ORIGIN_PX = (1747.0, 642.0)
PIXELS_PER_MM = 3.42


def font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        Path(
            "C:/Windows/Fonts/arialbd.ttf"
            if bold
            else "C:/Windows/Fonts/arial.ttf"
        ),
        Path(
            "C:/Windows/Fonts/segoeuib.ttf"
            if bold
            else "C:/Windows/Fonts/segoeui.ttf"
        ),
    ]
    for candidate in candidates:
        if candidate.is_file():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


TITLE_FONT = font(42, bold=True)
LABEL_FONT = font(24, bold=True)
SMALL_FONT = font(20)


def world_xz_to_pixel(x_mm: float, z_mm: float) -> tuple[int, int]:
    return (
        round(IMAGE_ORIGIN_PX[0] + PIXELS_PER_MM * x_mm),
        round(IMAGE_ORIGIN_PX[1] - PIXELS_PER_MM * z_mm),
    )


def rgba(value: str, alpha: int = 255) -> tuple[int, int, int, int]:
    token = value.lstrip("#")
    return (
        int(token[0:2], 16),
        int(token[2:4], 16),
        int(token[4:6], 16),
        alpha,
    )


def label_box(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    color: tuple[int, int, int, int],
    text: str,
) -> None:
    draw.rounded_rectangle(
        box,
        radius=12,
        fill=(255, 255, 255, 238),
        outline=color,
        width=5,
    )
    draw.rounded_rectangle(
        (box[0] + 10, box[1] + 8, box[0] + 34, box[3] - 8),
        radius=5,
        fill=color,
    )
    draw.text(
        (box[0] + 46, box[1] + 7),
        text,
        font=LABEL_FONT,
        fill=(22, 22, 24, 255),
    )


def distributed_centers(values: list[int], gap: int = 49) -> list[int]:
    result: list[int] = []
    for value in values:
        result.append(max(value, result[-1] + gap if result else value))
    return result


def render_servo_annotations() -> None:
    identities = json.loads(
        IDENTITY_CONFIG.read_text(encoding="utf-8")
    )["servos"]
    by_joint = {str(item["joint"]): item for item in identities}
    with SERVO_REPORT.open(
        "r", encoding="utf-8-sig", newline=""
    ) as stream:
        rows = list(csv.DictReader(stream))

    image = Image.open(FRONT_SOURCE).convert("RGBA")
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle(
        (40, 28, image.width - 40, 103),
        radius=20,
        fill=(255, 255, 255, 230),
        outline=(30, 30, 34, 220),
        width=3,
    )
    draw.text(
        (70, 42),
        "S01-S16 blue STS3250-C001 diagnostic bodies",
        font=TITLE_FONT,
        fill=(25, 25, 28, 255),
    )

    prepared: list[dict[str, object]] = []
    for row in rows:
        x_mm, _, z_mm = (
            float(value) for value in row["shaft_xyz_world_mm"].split()
        )
        identity = by_joint[row["joint"]]
        prepared.append(
            {
                "id": identity["id"],
                "joint": row["joint"],
                "color": rgba(str(identity["id_color_hex"])),
                "point": world_xz_to_pixel(x_mm, z_mm),
                "side": "left" if x_mm < 0.0 else "right",
            }
        )

    for side in ("left", "right"):
        items = sorted(
            (item for item in prepared if item["side"] == side),
            key=lambda item: item["point"][1],
        )
        centers = distributed_centers(
            [int(item["point"][1]) for item in items]
        )
        for item, center_y in zip(items, centers):
            point_x, point_y = item["point"]
            color = item["color"]
            if side == "left":
                box = (45, center_y - 21, 985, center_y + 21)
                line_end = (box[2], center_y)
            else:
                box = (
                    image.width - 985,
                    center_y - 21,
                    image.width - 45,
                    center_y + 21,
                )
                line_end = (box[0], center_y)
            draw.line(
                [line_end, (point_x, point_y)],
                fill=(32, 32, 36, 215),
                width=4,
            )
            draw.ellipse(
                (point_x - 16, point_y - 16, point_x + 16, point_y + 16),
                fill=(22, 119, 255, 245),
                outline=(255, 255, 255, 250),
                width=4,
            )
            label_box(
                draw,
                box,
                color,
                f"{item['id']}  {item['joint']}",
            )

    footer = (
        "One separate blue SLDPRT is reused at all 16 parent-side joint "
        "frames. It is excluded from URDF mass/collision and is not a "
        "physical mounting signoff."
    )
    draw.rounded_rectangle(
        (180, image.height - 84, image.width - 180, image.height - 24),
        radius=15,
        fill=(255, 255, 255, 235),
        outline=(35, 35, 38, 215),
        width=3,
    )
    draw.text(
        (210, image.height - 70),
        footer,
        font=SMALL_FONT,
        fill=(32, 32, 35, 255),
    )
    image.convert("RGB").save(SERVO_OUTPUT, quality=95)


def render_electronics_annotations() -> None:
    electronics = json.loads(
        IDENTITY_CONFIG.read_text(encoding="utf-8")
    )["electronics"]
    image = Image.open(FRONT_SOURCE).convert("RGBA")
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle(
        (40, 28, 1190, 565),
        radius=22,
        fill=(255, 255, 255, 235),
        outline=(35, 35, 38, 220),
        width=3,
    )
    draw.text(
        (70, 48),
        "Electronics / sensor placement",
        font=TITLE_FONT,
        fill=(25, 25, 28, 255),
    )
    entries = [
        (
            "eye_display_module",
            "Waveshare 4.3-inch DSI/QLED - controlled envelope",
        ),
        ("camera_module", "Camera Module 3 Wide - vendor envelope"),
        ("tof_module", "VL53L5CX ToF - assumed carrier envelope"),
        ("imu_module", "Torso IMU - RL assumption"),
        ("compute_module", "Compute + regulator tray - RL assumption"),
        ("battery_pack", "3S2P battery + BMS - RL assumption"),
        (
            "foot_pressure_sites",
            "Four foot-pressure sites - MJCF sites, not printed parts",
        ),
    ]
    y_pos = 116
    for key, text in entries:
        color = rgba(str(electronics[key]["color_hex"]))
        label_box(draw, (70, y_pos, 1155, y_pos + 49), color, text)
        y_pos += 60

    targets = {
        "eye_display_module": world_xz_to_pixel(0.0, 105.0),
        "camera_module": world_xz_to_pixel(0.0, 151.0),
        "tof_module": world_xz_to_pixel(26.0, 151.0),
        "imu_module": world_xz_to_pixel(0.0, 10.0),
        "compute_module": world_xz_to_pixel(0.0, 15.0),
        "battery_pack": world_xz_to_pixel(0.0, -44.0),
    }
    for key, target in targets.items():
        color = rgba(str(electronics[key]["color_hex"]))
        draw.ellipse(
            (target[0] - 16, target[1] - 16, target[0] + 16, target[1] + 16),
            outline=color,
            width=6,
        )

    footer = (
        "Display clamp fit, installed masses, cable routes, BMS/SBC/IMU "
        "selection and measured centers of mass remain hardware overrides."
    )
    draw.rounded_rectangle(
        (1220, image.height - 84, image.width - 180, image.height - 24),
        radius=15,
        fill=(255, 255, 255, 235),
        outline=(35, 35, 38, 215),
        width=3,
    )
    draw.text(
        (1250, image.height - 70),
        footer,
        font=SMALL_FONT,
        fill=(32, 32, 35, 255),
    )
    image.convert("RGB").save(ELECTRONICS_OUTPUT, quality=95)


def render_shoulder_closeup() -> None:
    image = Image.open(FRONT_SOURCE).convert("RGBA")
    shoulder_points = {
        "S01 right shoulder pitch": world_xz_to_pixel(-78.15, 43.05),
        "S02 left shoulder pitch": world_xz_to_pixel(78.15, 43.05),
        "S03 right shoulder yaw": world_xz_to_pixel(-112.15, 43.05),
        "S06 left shoulder yaw": world_xz_to_pixel(112.15, 43.05),
    }
    x_values = [point[0] for point in shoulder_points.values()]
    y_values = [point[1] for point in shoulder_points.values()]
    crop_box = (
        max(0, min(x_values) - 260),
        max(0, min(y_values) - 280),
        min(image.width, max(x_values) + 260),
        min(image.height, max(y_values) + 250),
    )
    crop = image.crop(crop_box)
    scale = 2
    crop = crop.resize(
        (crop.width * scale, crop.height * scale),
        Image.Resampling.LANCZOS,
    )
    draw = ImageDraw.Draw(crop, "RGBA")
    draw.rounded_rectangle(
        (24, 20, crop.width - 24, 92),
        radius=18,
        fill=(255, 255, 255, 235),
        outline=(30, 30, 34, 220),
        width=4,
    )
    draw.text(
        (50, 32),
        "Shoulder proof: two blue servos per side",
        font=TITLE_FONT,
        fill=(25, 25, 28, 255),
    )
    for index, (label, point) in enumerate(shoulder_points.items()):
        local = (
            (point[0] - crop_box[0]) * scale,
            (point[1] - crop_box[1]) * scale,
        )
        box_y = 120 + index * 58
        if index < 2:
            box = (30, box_y, 630, box_y + 48)
            line_start = (box[2], box_y + 24)
        else:
            box = (crop.width - 630, box_y, crop.width - 30, box_y + 48)
            line_start = (box[0], box_y + 24)
        draw.line(
            [line_start, local],
            fill=(20, 80, 180, 230),
            width=5,
        )
        draw.ellipse(
            (local[0] - 18, local[1] - 18, local[0] + 18, local[1] + 18),
            fill=(22, 119, 255, 245),
            outline=(255, 255, 255, 250),
            width=4,
        )
        label_box(draw, box, (22, 119, 255, 255), label)
    crop.convert("RGB").save(SHOULDER_OUTPUT, quality=95)


def main() -> None:
    render_servo_annotations()
    render_electronics_annotations()
    render_shoulder_closeup()
    print(f"SERVO_ANNOTATION={SERVO_OUTPUT}")
    print(f"ELECTRONICS_ANNOTATION={ELECTRONICS_OUTPUT}")
    print(f"SHOULDER_CLOSEUP={SHOULDER_OUTPUT}")


if __name__ == "__main__":
    main()
