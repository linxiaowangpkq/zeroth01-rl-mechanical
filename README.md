# Zeroth-01 Physical Mount V2 Minimal

This release keeps the collision-validated 16-DoF Zeroth mechanism and only
changes replaceable exterior/print parts: a directly nested rounded head,
an 8.775 mm shallow chest panel, compact fixed Q-hands, and 9 mm soles.

- The retained source head post is exposed by **1.404 mm** (gate: <=5 mm).
- There is no external neck component.
- The native SolidWorks review assembly contains 51 components, including
  16 separate blue source-installed servo parts and zero old claw parts.
- The canonical RL model is
  `generated/urdf/physical_mount_v2_minimal/zeroth01_physical_mount_v2_minimal.urdf`.
- `physical_mount_v2_minimal_rl_handoff.json` contains all joint limits,
  link inertials, actuator metadata, electronics/sensor positions, optical
  frames, sole contacts, validation evidence and domain-randomization ranges.
- MuJoCo passes neutral, 61 samples per joint over all guarded ranges and 73
  coordinated poses with zero non-adjacent-link collision failures.

Run `python scripts/validate_minimal_v2_release.py` after cloning.
The native SolidWorks Pack and Go package is flattened under
`generated/solidworks/physical_mount_v2_minimal/portable_flat/`; open the
`OPEN_FIRST_...XRAY.SLDASM` file there. It contains two assemblies and 51
separate part files without development-machine path dependencies.

The blue parts are source STS3215-family installed geometry used as placement
truth for the target FEETECH STS3250-C001. A purchased STS3250 must still pass
the included 4xM2/horn/rear-axis first-article gauge. Full-robot printing and
sim-to-real therefore remain on hold until first-article fit, wiring and
as-built mass/inertia/calibration gates are complete.

Read [README_zh.md](README_zh.md) for the complete Chinese handoff.
