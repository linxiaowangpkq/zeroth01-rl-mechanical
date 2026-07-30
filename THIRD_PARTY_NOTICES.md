# Third-party notices

The top-level MIT License applies to original code, documentation, validation
logic, and newly parameterized geometry in this package. It does not replace
the licenses, copyrights, trademarks, or attribution attached to third-party
sources.

## Zeroth / KScale robot description and link geometry

- Geometry-compatible source: `kscalelabs/zeroth-sim`
- Pinned commit: `33b0553bd085ff6360495497a8e86afaa801785d`
- Copyright: 2023 KScale Labs
- License: MIT; copied at
  [`LICENSES/ZEROTH_SIM_MIT.txt`](LICENSES/ZEROTH_SIM_MIT.txt)
- Scope: the 17 frozen source-link STL meshes, frames, source mass/inertia
  data, and derivatives used by the URDF, MJCF, and SolidWorks review parts.

Additional provenance references:

- `zeroth-robotics/zeroth-bot`, copyright 2024 Jingxiang Mo, MIT; license at
  [`LICENSES/ZEROTH_BOT_MIT.txt`](LICENSES/ZEROTH_BOT_MIT.txt).
- `kscalelabs/onshape`, copyright 2023 Benjamin Bolte, MIT; license at
  [`LICENSES/KSCALE_ONSHAPE_MIT.txt`](LICENSES/KSCALE_ONSHAPE_MIT.txt).

The later Zeroth commit
`43c5baa1287db078bef638308ef077445704be1d` changed frames, inertials, and mesh
names without a matching public replacement mesh bundle. This release does
not mix those frames with the older geometry.

## Poppy Eva head topology

- Official source:
  [`poppy-project/Poppy-eva-head-design`](https://github.com/poppy-project/Poppy-eva-head-design)
- Pinned commit: `844654a0b29fb771c23b7400997d1de3d42e0e2e`
- License: CC BY-SA 4.0; copied at
  [`LICENSES/POPPY_EVA_HEAD_CC_BY_SA_4_0.md`](LICENSES/POPPY_EVA_HEAD_CC_BY_SA_4_0.md)
- Vendored source:
  `source_assets/open_source_head/Poppy-eva-head-design/`

The selected Zeroth-01 head is a new parameterized derivative using the
source split/mounting topology as a reference. It is rebuilt around the
frozen Zeroth-01 shoulder keep-outs and selected screen envelope, rather than
being a direct scale of the source STL. The derivative CAD and redistributed
source remain subject to the CC BY-SA 4.0 terms.

## Selected head electronics

- Display: official
  [Waveshare 4.3inch DSI QLED](https://www.waveshare.com/product/4.3inch-dsi-qled.htm),
  selected as an `800×480` product with a controlled
  `105.5 × 8 × 67.2 mm` packaging envelope. No exact supplier STEP is claimed.
- Camera: official
  [Raspberry Pi Camera Module 3 Wide](https://www.raspberrypi.com/products/camera-module-3/).
  The exact review STEP is retained at
  `source_assets/vendor/head_electronics/Raspberry_Pi_Camera_Module_3_Wide.step`.
- ToF: [ST VL53L5CX](https://www.st.com/en/imaging-and-photonics-solutions/vl53l5cx.html);
  the `12 × 10 × 3 mm` carrier-board envelope is still an explicit assumption.

Vendor marks and third-party rights remain with their owners. Machine-readable
details are in
`source_assets/vendor/head_electronics/PROVENANCE.json`.

The older Waveshare 0.71-inch DualEye module was investigated historically but
is not selected and is not redistributed in this release.

## Quarantined FEETECH catalog CAD

The historical step.parts record labelled `feetech_sts3250` embeds the STEP
product name `ST-3235M-20211119-A_ASM`, has a different local output axis, and
does not match the current STS3250-C001 drawing. It is therefore not
redistributed or inserted into the selected assembly.

The blue `ZEROTH01_STS3250_C001_BLUE_DIAGNOSTIC` part is an original,
dimension-controlled visibility envelope made from the user-supplied drawing.
It is not a vendor installation model and is excluded from URDF/MJCF mass and
collision.

## User reference image

The user's visual reference image is not redistributed. The white rounded
exterior, small ears, arm sleeves, and fixed chibi palms are new simplified
styling geometry.
