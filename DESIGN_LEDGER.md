# Zeroth-01 derived mechanical model — decision ledger

## Status

- Primary consumer: RL simulation and mechanical reference.
- Simulation readiness: `PASS`.
- Hardware deployment readiness: `BLOCKED_BY_PHYSICAL_CALIBRATION`.
- Manufacturing release: `NOT_CLAIMED`.

## D-001 — Freeze geometry to `zeroth-sim@33b0553`

The published 17-file Drive mesh bundle matches commit
`33b0553bd085ff6360495497a8e86afaa801785d`.

Commit `43c5baa1287db078bef638308ef077445704be1d` changed joint frames,
inertials and mesh names without publishing its replacement mesh bundle. Pairing
those later frames with the older meshes caused detached feet and inconsistent
COM-to-mesh placement. Therefore all generated descriptions read the historical
URDF through `git show` and keep the exact matching STL payload.

Evidence:

- `reports/source_lock.json`
- `reports/source_asset_manifest.csv`
- `reports/mesh_frame_audit.json`

## D-002 — Preserve upstream link frames and normalize only the joint typo

No mesh recentering, axis permutation or silent scale is allowed. The public
link vocabulary is preserved even where left/right mesh names are unintuitive.
The interface typo `righ_elbow_yaw` is normalized to `right_elbow_yaw` because
the official Python control interface already uses the corrected joint name.

## D-003 — Use three robot-description layers

1. `zeroth01_rl_reference.urdf`: frozen geometry-compatible upstream reference.
2. `zeroth01_rl_audited.urdf`: source/control/per-axis audit intersection.
3. `zeroth01_rl_ready.urdf`: recommended guarded multi-joint startup envelope.

The third file is the training default. Full mechanical ranges may be explored
only with online collision query/action projection and collision termination.

## D-004 — Treat four neutral overlaps as assembly exclusions

Only the Torso-to-left/right hip-yaw and Torso-to-left/right shoulder-yaw pairs
are allowed assembly overlaps. They are present at neutral and are excluded
from self-contact generation in the native MJCF. Every other self-contact is
prohibited.

The 30% startup box passed 20,000 random samples and every one of 65,536
corners; native MJCF validation then passed 100,000 random samples. This is
statistical mesh-level evidence, not continuous or manufacturing clearance
proof.

## D-005 — Keep aggregate inertias; do not double-count servos

The official link inertias include structure and actuator mass. The total model
mass is `3.0954718282 kg`. Adding 16 independent `0.0745 kg` servo bodies would
double-count mass and invalidate COM and inertia.

## D-006 — Separate candidate actuator metadata from measured calibration

All 16 joints use Feetech STS3250 metadata. Candidate IDs are retained to speed
up wiring work, but are not declared as exact hardware truth. The following
remain null until physical calibration:

- confirmed bus ID;
- URDF-to-servo direction sign;
- per-unit zero offset and hard stops;
- backlash/deadband;
- torque-current and thermal continuous-duty envelope;
- identified damping, armature and friction.

Hardware deployment remains false until
`generated/config/zeroth01_hardware_calibration_template.csv` is completed and
checked.

## D-007 — SolidWorks is a geometric FK review, not native mate Motion

The public open-surface STL files do not expose stable cylindrical B-Rep faces
for robust mates. SolidWorks is driven through COM Transform2 using the same
URDF FK. The final gate covers 17 connected components, 16 joints and 48
lower/zero/upper poses. A native mate/motor Motion Study is explicitly
`NOT_CLAIMED`.

## D-008 — Keep generated work under `roboto_xw`

Upstream repositories stay read-only under `upstream/`. Owned generators,
overlays, reports and review artifacts remain in this directory; no change is
required in `roboto_origin/`.
