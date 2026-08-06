# Zeroth-01 Physical Mount v3 RL-Fixed Design Ledger

## Robot metadata

- Robot: `zeroth01_physical_mount_v3_rl_fixed_18dof`.
- Baseline: released v2-minimal appearance and source-carrier geometry.
- Consumers: MuJoCo/MJX RL, URDF viewers, SolidWorks/CAD review, first-article print review.
- Units: URDF/MJCF use m, kg, s, rad; CAD uses mm.
- Body convention: right-handed, `+X` forward, `+Y` left, `+Z` up.
- Mesh source: v2-minimal STL files are millimetre meshes and therefore use URDF scale `0.001`.

## Why v3 is 18DoF

The v2 topology has shoulder-yaw, shoulder-pitch and elbow-yaw on each arm,
plus hip-yaw, hip-roll, hip-pitch, knee-pitch and ankle-pitch on each leg.
It cannot independently regulate sole roll and torso roll in single support.
v3 adds one ankle-roll joint and STS3250 package per side.  Retaining 16DoF
would leave BUG-009 unresolved and therefore cannot be called an all-issues fix.

## Kinematic frames and positive motion

All movable child frames are located on their physical shaft centre and are
world-axis aligned in the neutral pose.  This removes the old mesh-frame RPY
coupling that mislabeled hip yaw/roll axes.  Visual transforms preserve the
v2 neutral appearance; collision and inertial frames are authored independently.

| Joint family | Left neutral axis | Right neutral axis | Positive motion |
|---|---:|---:|---|
| shoulder yaw | `+Z` | `-Z` | arm moves forward under mirrored controls |
| shoulder pitch | `+Y` | `+Y` | arm pitches forward |
| elbow yaw (historical name) | `+X` | `-X` | forearm hinges under mirrored controls |
| hip yaw | `+Z` | `-Z` | toe direction yaws outward under mirrored controls |
| hip roll | `+X` | `-X` | pelvis shifts toward the named support side |
| hip pitch | `+Y` | `+Y` | thigh moves forward (`+X`) |
| knee pitch | `+Y` | `+Y` | knee flexes backward |
| ankle pitch | `+Y` | `+Y` | toe pitches upward |
| ankle roll | `+X` | `-X` | sole medial edge rises under mirrored controls |

Left/right joint anchors are generated from one averaged source datum and an
explicit reflection across `Y=0`.  The release tolerance is 0.25 mm, tighter
than the failed v2 0.75 mm gate.

## Mass and inertia ledger

- Canonical original Zeroth geometry-matched URDF mass: `3.095471828 kg`.
- v3 nominal RL mass target: exactly `3.095471828 kg`.
- STS3250-C001 model mass: `0.0745 kg` each, 18 units, total `1.341 kg`.
- Remaining nominal mass covers source carriers, fasteners, lightened v2
  appearance parts, battery, compute, the purchased interaction head and IMUs.
- The former printed head/display/camera/ToF stack is deleted.  Its replacement
  is one complete M5Stack StackChan K151 purchased module at the vendor mass of
  `0.187 kg`, plus an `0.018 kg` 3 mm 6061 reversible adapter.  Aggregate torso
  mass is reduced so the locked total remains `3.095471828 kg`.
- Actuator mass is owned exactly once: each of the 18 actuated child links owns
  one 74.5 g STS3250 contribution, so no link is lighter than its contained servo.
- Every link inertia is calculated from its primitive collision envelope and
  nominal link mass.  Confidence is `estimated_from_envelope`, not measured.
- `MASS_IDENTIFIED_PASS` remains false until the assembled links are weighed
  and their centres of mass are measured.  RL may use the nominal model with
  the published randomisation range, but it must not be called as-built truth.

## Collision ledger

- RL collision geometry is primitive-only: boxes, spheres, cylinders and capsules.
- Limb capsules are generated from the true joint-to-child vector rather than a
  hard-coded vertical box, so collision geometry follows the actual kinematic chain.
- The torso collision proxy is the central structural spine and deliberately
  excludes the shoulder cut-outs used by the two shoulder chains.
- Each sole has its own symmetric box collision and four contact frames.
- Detailed v2 visual meshes never participate in MJX contact.
- The printable 9 mm sole is lightened with a perimeter ring and crossed ribs;
  its collision envelope remains a closed 9 mm box.

## STS3250 interface ledger

- Controlled dimensions: case `45.22 x 24.72 x 35.0 mm`, shaft centre 12.5 mm
  from the short end, 25T/5.9 mm output, M3x6 retention.
- Drawing-derived case holes: 4-M2.0, centres `X={-28.50, 8.30} mm`,
  `Y={-10.25, 10.25} mm` in the controlled shaft frame.
- The step.parts record `feetech_sts3250` is checksum-verified but its embedded
  STEP product is `ST-3235M-20211119-A_ASM`; it remains quarantined and is not
  treated as an exact C001 installation B-Rep.
- v3 uses the dimension-controlled STEP and the existing source carrier
  datums.  Full-print release remains blocked until one purchased STS3250
  passes the 4-M2/25T/rear-support first-article gauge.

## Full assembly ownership

- The native/full CAD handoff is driven by the external-part assembly manifest,
  not by a fused display solid.  It contains 58 independently addressable
  components: retained v2 carriers and white shells, 18 blue STS3250 instances,
  two ankle-roll carrier/horn sets, two inboard hip adapters and two 9 mm soles.
- Servo instance IDs `S01`--`S18`, shaft frames and source part paths are recorded
  in the manifest and actuator-layout JSON so SolidWorks and RL use one datum set.
- Legacy claw parts, the legacy 16 placeholder servo bodies and all nine custom
  head/display/camera parts are excluded.

## Purchased interaction head

- Selected module: M5Stack StackChan K151 (complete off-the-shelf unit), vendor
  envelope `61.5 x 54.0 x 70.5 mm`, mass `187 g`.
- Integrated functions: 2-inch capacitive colour display, GC0308 0.3 MP camera,
  dual microphones, 1 W speaker, proximity/ambient sensor, 9-axis IMU, battery,
  Wi-Fi/BLE and its own two-axis expression mechanism.
- During walking/RL the internal StackChan pan/tilt remains centred and its full
  mass is modeled as one fixed payload.  It may be commanded only outside the
  locomotion policy until its moving-mass perturbation is identified.
- Vendor mounting rectangle is `48 x 32 mm` with M3 hardware.  A flat 3 mm 6061
  adapter uses that exact pattern and four closed torso-side adjustment slots;
  the module sits directly on the plate, so there is no external neck and the
  visible gap is `0 mm`.
- Official model-size PDF and official structure STLs are archived under
  `vendor/m5stack_stackchan_k151/`; the generated STEP is a controlled purchased
  envelope, not an instruction to print a substitute head.

## Independent release gates

1. `CAD_PORTABLE_PASS`: generated STEP/STL/SolidWorks files open and resolve.
2. `KINEMATIC_SYMMETRY_PASS`: paired joint anchors/axes satisfy the v3 mirror rule.
3. `MJX_PRIMITIVE_COLLISION_PASS`: no mesh collision enters the RL model.
4. `GROUND_CONTACT_PASS`: official standing keyframe has two level soles at ground.
5. `ACTUATOR_STATIC_FEASIBILITY_PASS`: conservative single-support torque is
   below the 1.255 N.m continuous design value.
6. `MASS_TARGET_PASS`: nominal mass is in 3.0--3.3 kg and equals the locked target.
7. `MASS_IDENTIFIED_PASS`: only after as-built measurements; expected `HOLD` now.
8. `STS3250_FIRST_ARTICLE_PASS`: only after purchased-hardware fit; expected `HOLD` now.
