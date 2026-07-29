# Third-party notices

The top-level MIT License applies to original code, documentation, validation
logic, and round-v1 geometry created for this package. It does not erase
copyright or attribution attached to third-party source assets.

## Zeroth / KScale geometry and robot description

- Source project: `kscalelabs/zeroth-sim`
- Geometry-compatible source commit:
  `33b0553bd085ff6360495497a8e86afaa801785d`
- Copyright: 2023 KScale Labs
- License: MIT; copied at
  [`LICENSES/ZEROTH_SIM_MIT.txt`](LICENSES/ZEROTH_SIM_MIT.txt)
- Scope here: the 17 source-link STL meshes, their compatible robot frames,
  source mass/inertia data, and derivatives used by the URDF/MJCF and
  SolidWorks surface parts.

Additional provenance references:

- `zeroth-robotics/zeroth-bot`, copyright 2024 Jingxiang Mo, MIT; license at
  [`LICENSES/ZEROTH_BOT_MIT.txt`](LICENSES/ZEROTH_BOT_MIT.txt).
- `kscalelabs/onshape`, copyright 2023 Benjamin Bolte, MIT; license at
  [`LICENSES/KSCALE_ONSHAPE_MIT.txt`](LICENSES/KSCALE_ONSHAPE_MIT.txt).

The later Zeroth commit
`43c5baa1287db078bef638308ef077445704be1d` changed frames/inertials/mesh names
without a matching public replacement mesh bundle, so this release does not
mix it with the older meshes.

## FEETECH STS3250 CAD

- Catalog record: `feetech_sts3250`
- Catalog: [step.parts](https://www.step.parts/parts/feetech_sts3250)
- Source repository: [earthtojake/step.parts](https://github.com/earthtojake/step.parts)
- Pinned source commit:
  `c6113328a5695b976a010a203a90fe86191769bf`
- Source file:
  `catalog/step/feetech_sts3250.step`
- SHA-256:
  `cf46f17da455e1f158114791bb31404c24d925e8a758bbd6189f8ee815a571bf`
- License basis: the step.parts record has no separate third-party
  `stepSource`; the step.parts repository describes its original project
  material as MIT and its catalog as open-source. The original STEP and the
  SolidWorks-format conversion are attributed here and are not endorsed by
  FEETECH.

Machine-readable details are in
[`source_assets/vendor/sts3250/PROVENANCE.json`](source_assets/vendor/sts3250/PROVENANCE.json).

## Reference image

The user's visual reference image is not redistributed in this repository.
The round-v1 CAD is a new simplified mechanical styling interpretation, not a
copy of the depicted product.
