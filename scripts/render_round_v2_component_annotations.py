from __future__ import annotations

import csv
import json
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


THIS_FILE = Path(__file__).resolve()
ROOT = THIS_FILE.parents[1]
SNAPSHOT_DIR = ROOT / "snapshots" / "solidworks" / "round_v1"
IDENTITY_CONFIG = ROOT / "config" / "round_v2_component_identity.json"
JOINT_REPORT = ROOT / "reports" / "round_v1_servo_axis_alignment.csv"

FRONT_SOURCE = SNAPSHOT_DIR / "zeroth01_round_v1_robot_front.png"
TRANSPARENT_SOURCE = (
    SNAPSHOT_DIR / "zeroth01_round_v2_electronics_transparent_front.png"
)
JOINT_OUTPUT = (
    SNAPSHOT_DIR / "zeroth01_round_v2_joint_identity_front.png"
)
ELECTRONICS_OUTPUT = (
    SNAPSHOT_DIR / "zeroth01_round_v2_electronics_annotated_front.png"
)

# Calibrated once against the orthographic SolidWorks front snapshot. Robot
# +X points to image right and +Z points upward.
IMAGE_ORIGIN_PX = (1747.0, 642.0)
PIXELS_PER_MM = 3.42


def font(size: int, *, bold: bool = False) -> ImageFont.FreeTypeFont:
    candidates = [
        Path("C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"),
        Path("C:/Windows/Fonts/segoeuib.ttf" if bold else "C:/Windows/Fonts/segoeui.ttf"),
    ]
    for candidate in candidates:
        if candidate.is_file():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


TITLE_FONT = font(44, bold=True)
LABEL_FONT = font(26, bold=True)
SMALL_FONT = font(22)


def world_xz_to_pixel(x_mm: float, z_mm: float) -> tuple[int, int]:
    return (
        round(IMAGE_ORIGIN_PX[0] + PIXELS_PER_MM * x_mm),
        round(IMAGE_ORIGIN_PX[1] - PIXELS_PER_MM * z_mm),
    )


def rgba(hex_color: str, alpha: int = 255) -> tuple[int, int, int, int]:
    value = hex_color.lstrip("#")
    return (
        int(value[0:2], 16),
        int(value[2:4], 16),
        int(value[4:6], 16),
        alpha,
    )


def rounded_label(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    color: tuple[int, int, int, int],
    text: str,
) -> None:
    draw.rounded_rectangle(
        box,
        radius=14,
        fill=(255, 255, 255, 235),
        outline=color,
        width=5,
    )
    swatch = (box[0] + 12, box[1] + 9, box[0] + 38, box[3] - 9)
    draw.rounded_rectangle(swatch, radius=6, fill=color)
    draw.text(
        (box[0] + 52, box[1] + 8),
        text,
        font=LABEL_FONT,
        fill=(25, 25, 25, 255),
    )


def distributed_label_centers(
    requested: list[int],
    minimum_gap: int = 51,
) -> list[int]:
    result: list[int] = []
    for value in requested:
        result.append(max(value, result[-1] + minimum_gap if result else value))
    return result


def render_joint_annotations() -> None:
    identity = json.loads(
        IDENTITY_CONFIG.read_text(encoding="utf-8")
    )["servos"]
    identity_by_joint = {
        str(item["joint"]): item for item in identity
    }
    with JOINT_REPORT.open("r", encoding="utf-8-sig", newline="") as stream:
        rows = list(csv.DictReader(stream))

    image = Image.open(FRONT_SOURCE).convert("RGBA")
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle(
        (40, 30, image.width - 40, 105),
        radius=22,
        fill=(255, 255, 255, 228),
        outline=(40, 40, 40, 220),
        width=3,
    )
    draw.text(
        (70, 43),
        "S01-S16 joint positions — colored annotations, not added hardware",
        font=TITLE_FONT,
        fill=(30, 30, 30, 255),
    )

    prepared: list[dict[str, object]] = []
    for row in rows:
        x_mm, _, z_mm = (
            float(value) for value in row["shaft_xyz_world_mm"].split()
        )
        px, py = world_xz_to_pixel(x_mm, z_mm)
        item = identity_by_joint[row["joint"]]
        prepared.append(
            {
                "id": item["id"],
                "joint": row["joint"],
                "color": rgba(str(item["color_hex"])),
                "point": (px, py),
                "side": "left" if x_mm < 0.0 else "right",
            }
        )

    for side in ("left", "right"):
        items = sorted(
            (item for item in prepared if item["side"] == side),
            key=lambda item: item["point"][1],
        )
        centers = distributed_label_centers(
            [int(item["point"][1]) for item in items]
        )
        for item, label_y in zip(items, centers):
            point_x, point_y = item["point"]
            color = item["color"]
            if side == "left":
                box = (45, label_y - 22, 1015, label_y + 22)
                line_end = (box[2], label_y)
            else:
                box = (
                    image.width - 1015,
                    label_y - 22,
                    image.width - 45,
                    label_y + 22,
                )
                line_end = (box[0], label_y)
            draw.line(
                [line_end, (point_x, point_y)],
                fill=(35, 35, 35, 215),
                width=4,
            )
            draw.ellipse(
                (point_x - 18, point_y - 18, point_x + 18, point_y + 18),
                fill=(255, 255, 255, 235),
                outline=(20, 20, 20, 255),
                width=4,
            )
            draw.ellipse(
                (point_x - 12, point_y - 12, point_x + 12, point_y + 12),
                fill=color,
            )
            rounded_label(
                draw,
                box,
                color,
                f"{item['id']}  {item['joint']}",
            )

    footer = (
        "Authority: frozen 17-link Zeroth-01 mechanism. Markers follow the "
        "parent-side joint frames and do not represent horns, gears or cages."
    )
    draw.rounded_rectangle(
        (200, image.height - 86, image.width - 200, image.height - 25),
        radius=16,
        fill=(255, 255, 255, 232),
        outline=(40, 40, 40, 210),
        width=3,
    )
    draw.text(
        (230, image.height - 72),
        footer,
        font=SMALL_FONT,
        fill=(35, 35, 35, 255),
    )
    image.convert("RGB").save(JOINT_OUTPUT, quality=95)


def render_electronics_annotations() -> None:
    identity = json.loads(
        IDENTITY_CONFIG.read_text(encoding="utf-8")
    )["electronics"]
    image = Image.open(TRANSPARENT_SOURCE).convert("RGBA")
    draw = ImageDraw.Draw(image, "RGBA")
    draw.rounded_rectangle(
        (40, 30, 1155, 580),
        radius=24,
        fill=(255, 255, 255, 232),
        outline=(35, 35, 35, 220),
        width=3,
    )
    draw.text(
        (70, 54),
        "Electronics / sensor placement",
        font=TITLE_FONT,
        fill=(25, 25, 25, 255),
    )
    entries = [
        ("eye_display_module", "Waveshare dual-eye display — vendor STEP"),
        ("camera_module", "Camera Module 3 Wide — vendor envelope"),
        ("tof_module", "VL53L5CX ToF — assumed carrier envelope"),
        ("imu_module", "Torso IMU — RL assumption"),
        ("compute_module", "Compute + regulator tray — RL assumption"),
        ("battery_pack", "3S2P battery + BMS — RL assumption"),
        (
            "foot_pressure_sites",
            "Four foot-pressure sites — MJCF sites, not printed parts",
        ),
    ]
    y = 125
    for key, text in entries:
        color = rgba(str(identity[key]["color_hex"]))
        box = (70, y, 1125, y + 51)
        rounded_label(draw, box, color, text)
        y += 62

    targets = {
        "eye_display_module": world_xz_to_pixel(0.0, 115.0),
        "camera_module": world_xz_to_pixel(0.0, 91.0),
        "tof_module": world_xz_to_pixel(31.0, 92.0),
        "imu_module": world_xz_to_pixel(0.0, 10.0),
        "compute_module": world_xz_to_pixel(0.0, 15.0),
        "battery_pack": world_xz_to_pixel(0.0, -44.0),
    }
    for key, target in targets.items():
        color = rgba(str(identity[key]["color_hex"]))
        draw.ellipse(
            (target[0] - 16, target[1] - 16, target[0] + 16, target[1] + 16),
            outline=color,
            width=6,
        )
    for x_mm in (-46.0, 46.0):
        target = world_xz_to_pixel(x_mm, -302.0)
        color = rgba(str(identity["foot_pressure_sites"]["color_hex"]))
        draw.ellipse(
            (target[0] - 17, target[1] - 10, target[0] + 17, target[1] + 10),
            outline=color,
            width=6,
        )

    footer = (
        "Exact installed masses, cable routes, BMS/SBC/IMU selection and "
        "measured centers of mass remain hardware overrides before deployment."
    )
    draw.rounded_rectangle(
        (1220, image.height - 88, image.width - 180, image.height - 25),
        radius=16,
        fill=(255, 255, 255, 232),
        outline=(40, 40, 40, 210),
        width=3,
    )
    draw.text(
        (1250, image.height - 73),
        footer,
        font=SMALL_FONT,
        fill=(35, 35, 35, 255),
    )
    image.convert("RGB").save(ELECTRONICS_OUTPUT, quality=95)


def main() -> None:
    render_joint_annotations()
    render_electronics_annotations()
    print(f"JOINT_ANNOTATION={JOINT_OUTPUT}")
    print(f"ELECTRONICS_ANNOTATION={ELECTRONICS_OUTPUT}")


if __name__ == "__main__":
    main()
