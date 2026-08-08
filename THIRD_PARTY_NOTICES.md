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

V4 uses 18 independent occurrences normalized from the downloadable STS3250
CAD hosted by step.parts:

- Catalog page: <https://www.step.parts/parts/feetech_sts3250>
- Preserved download: `source_assets/step_parts/feetech_sts3250.step`
- SHA-256: `cf46f17da455e1f158114791bb31404c24d925e8a758bbd6189f8ee815a571bf`

This is substantially more detailed than the former drawing-derived box, but
it is still third-party reference CAD and not a substitute for checking the
purchased actuator, threads, spline, cable exit and production tolerances.

## Engineering workflow references

V4 does not redistribute or copy geometry from the following projects. Their
public documentation informed service splitting, double-supported brackets,
cable routing, MJX/SysID and training workflow only; their own licenses and
trademarks remain with their authors.

- ToddlerBot: <https://github.com/hshi74/toddlerbot>
- Open Duck Mini: <https://github.com/apirrone/Open_Duck_Mini>
- KHR-3HV documentation: <https://kondo-robot.com/faq/khr-3hv-erection-diagram>
- TonyPi documentation: <https://docs.hiwonder.com/projects/TonyPi/en/latest/>

M5Stack UnitV2 dimensions, camera and microphone descriptions are vendor
facts. The v4 envelope and cradle are newly modelled references, not vendor
CAD: <https://docs.m5stack.com/en/unit/unitv2>.
