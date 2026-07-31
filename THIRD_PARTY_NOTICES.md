# Third-party notices

The top-level MIT License covers original scripts, validation logic,
documentation and newly generated reference/gauge geometry. It does not
replace third-party licenses or vendor rights.

## Zeroth / K-Scale Z-Bot source

- Repository: <https://github.com/kscalelabs/kscale-assets>
- Pinned commit: `f51d6ea19b8b824dd7661600c7a87f3691f770be`
- Source path: `zbot/`
- Local release path: `upstream/kscale-assets/zbot/`
- Repository status: archived/read-only.

The 16 installed servo regions and 20 carrier meshes are derived from those
pinned triangulated assets. The source did not include native SolidWorks or
STEP B-Rep parts; this release does not claim otherwise.

Related MIT notices retained in `LICENSES/`:

- `ZEROTH_SIM_MIT.txt`
- `ZEROTH_BOT_MIT.txt`
- `KSCALE_ONSHAPE_MIT.txt`

Machine-readable provenance is in
`source_assets/PHYSICAL_MOUNT_V1_SOURCE_LOCK.json`.

## FEETECH

STS3215 and STS3250 specifications/drawings remain copyright of FEETECH. They
are redistributed only as engineering references; FEETECH names and marks
belong to their owner.

- STS3250 official specification:
  <https://www.feetechrc.com/Data/feetechrc/upload/file/20240120/6384135881578380868917773.pdf>
- STS3215 official specification:
  <https://www.feetechrc.com/Data/feetechrc/upload/file/20200611/6372749961523760249976542.pdf>

The separate `FEETECH_STS3250_C001_DIMENSION_REFERENCE.step` is newly
constructed from published nominal dimensions. It is not supplier-native CAD.
The installed blue SolidWorks parts are explicitly labelled
`INSTALLED_STS3215_FAMILY_REFERENCE`; they are source placement references,
not exact vendor STS3250 B-Rep files.
