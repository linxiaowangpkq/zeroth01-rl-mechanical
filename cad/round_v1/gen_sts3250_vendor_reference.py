from pathlib import Path

from build123d import import_step


ROOT = Path(__file__).resolve().parents[2]


def gen_step():
    shape = import_step(
        ROOT
        / "source_assets"
        / "vendor"
        / "sts3250"
        / "FEETECH_STS3250.step"
    )
    shape.label = "FEETECH_STS3250_VENDOR_REFERENCE"
    return shape
