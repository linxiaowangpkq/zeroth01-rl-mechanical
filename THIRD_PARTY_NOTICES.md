# Third-party notices

The top-level MIT License applies to original code, documentation, validation
logic, and round-v2 minimal-cosmetic geometry created for this package. It
does not erase copyright, trademarks, or attribution attached to third-party
source assets.

## Zeroth / KScale geometry and robot description

- Source project: `kscalelabs/zeroth-sim`
- Geometry-compatible source commit:
  `33b0553bd085ff6360495497a8e86afaa801785d`
- Copyright: 2023 KScale Labs
- License: MIT; copied at
  [`LICENSES/ZEROTH_SIM_MIT.txt`](LICENSES/ZEROTH_SIM_MIT.txt)
- Scope here: the 17 source-link STL meshes, compatible robot frames,
  source mass/inertia data, and derivatives used by URDF/MJCF and SolidWorks
  surface parts.

Additional provenance references:

- `zeroth-robotics/zeroth-bot`, copyright 2024 Jingxiang Mo, MIT; license at
  [`LICENSES/ZEROTH_BOT_MIT.txt`](LICENSES/ZEROTH_BOT_MIT.txt).
- `kscalelabs/onshape`, copyright 2023 Benjamin Bolte, MIT; license at
  [`LICENSES/KSCALE_ONSHAPE_MIT.txt`](LICENSES/KSCALE_ONSHAPE_MIT.txt).

The later Zeroth commit
`43c5baa1287db078bef638308ef077445704be1d` changed frames, inertials, and mesh
names without a matching public replacement mesh bundle, so this release does
not mix it with the older geometry.

## Quarantined FEETECH catalog CAD

- Catalog record: `feetech_sts3250`
- Catalog: [step.parts](https://www.step.parts/parts/feetech_sts3250)
- Source repository:
  [earthtojake/step.parts](https://github.com/earthtojake/step.parts)
- Pinned source commit:
  `c6113328a5695b976a010a203a90fe86191769bf`
- SHA-256:
  `cf46f17da455e1f158114791bb31404c24d925e8a758bbd6189f8ee815a571bf`

The embedded STEP product header is `ST-3235M-20211119-A_ASM`, not the current
`ST-3250-C001`. Its measured shaft axis and one overall dimension also differ
from the current C001 drawing. The file is retained only as quarantined
provenance/dimensional evidence and is not inserted into the selected
SolidWorks, URDF, or MJCF assembly.

The step.parts record has no separate third-party `stepSource`; the
step.parts repository describes its original project material as MIT and its
catalog as open-source. The source is attributed here and is not endorsed by
FEETECH. Machine-readable details are in
[`source_assets/vendor/sts3250/PROVENANCE.json`](source_assets/vendor/sts3250/PROVENANCE.json).

## Head electronics CAD

- Waveshare 0.71inch DualEye LCD Module:
  [vendor documentation](https://www.waveshare.com/wiki/0.71inch_DualEye_LCD_Module).
- Raspberry Pi Camera Module 3 Wide:
  [vendor documentation](https://www.raspberrypi.com/products/camera-module-3/).

The exact vendor STEP files are included for interface review. The automated
SolidWorks assembly uses the exact Waveshare STEP and a measured camera
envelope because the detailed camera model contains 631 solids and is not
suitable for unattended import. Vendor marks and third-party rights remain
with their respective owners. Machine-readable provenance is at
[`source_assets/vendor/head_electronics/PROVENANCE.json`](source_assets/vendor/head_electronics/PROVENANCE.json).

## Reference image

The user's visual reference image is not redistributed in this repository.
The round-v2 CAD is a new simplified styling interpretation, not a copy of the
depicted product.
