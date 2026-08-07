# Zeroth-01 Physical Mount v3.1 Compact RL-Fixed Design Ledger

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

## Standing envelope, mass and inertia ledger

- Pre-compaction v3 nominal mass: `3.095471828 kg`.
- v3.1 nominal RL mass target: exactly `2.969171828 kg`, below the hard
  `3.000 kg` ceiling by `30.828 g` before as-built identification.
- Native SolidWorks standing-height hard gate: `<=500.0 mm`.  The previous
  STEP measured `572.678 mm`; the compact model sinks the head into the upper
  torso, re-clocks only the two new ankle-roll servos so their 45.22 mm case
  axis is horizontal, and uses a verified `30 mm` ankle-roll centre spacing.
- Final native SolidWorks occurrence-box height is `499.236652 mm` (`PASS`);
  the independent diagnostic STEP height is `494.818 mm`.  The SolidWorks
  value is the conservative release gate, while the built robot must still be
  measured because the remaining nominal margin is only `0.763 mm`.
- STS3250-C001 model mass: `0.0745 kg` each, 18 units, total `1.341 kg`.
- Remaining nominal mass covers source carriers, fasteners, lightened v2
  appearance parts, battery, compute, the purchased interaction head and IMUs.
- The former K151 head and large plate are deleted.  Their replacement is one
  purchasable M5Stack CoreS3 K128 main unit at the official
  `54.0 x 54.0 x 15.5 mm` envelope.  The official `72.7 g` whole-set weight is
  conservatively assigned to the main unit even though the DinBase is omitted,
  plus a `6 g` allowance for the hidden 2 mm 6061 U-cradle.
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
- The printable 7 mm sole is lightened with a 2 mm skin and 5 mm perimeter/rib
  stack; its collision envelope remains a closed 7 mm box.

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
  not by a fused display solid.  It contains 51 independently addressable
  components: retained v2 carriers and white shells, 18 blue STS3250 instances,
  two ankle-roll carrier/horn sets, two 7 mm soles, CoreS3 and its hidden cradle.
- Servo instance IDs `S01`--`S18`, shaft frames and source part paths are recorded
  in the manifest and actuator-layout JSON so SolidWorks and RL use one datum set.
- Legacy claw parts, the legacy 16 placeholder servo bodies and all nine custom
  head/display/camera parts are excluded.

## Purchased interaction head

- Selected module: M5Stack CoreS3 K128, official main-unit envelope
  `54.0 x 54.0 x 15.5 mm`; conservative assigned mass `72.7 g`.
- Integrated functions: 2-inch 320x240 capacitive IPS display, GC0308 0.3 MP
  camera, dual microphones, 1 W speaker, proximity/ambient sensor, BMI270
  6-axis IMU, BMM150 magnetometer, Wi-Fi and internal 500 mAh battery.
- The module is fixed during walking and introduces no moving head axes.  Its
  height spans `z=-9..45 mm` and is flush-sunk into the body installation
  pocket, so it creates neither a neck nor any extra standing height.
- A hidden 2 mm 6061 U-cradle touches the rear and three perimeter faces of the
  controlled module envelope.  The body contains the matching recessed slot,
  four M3 passages and `0.20 mm` cradle clearance.  Fit remains a first-article
  item because the official repository provides an STL rather than a tolerance
  drawing for this robot-specific interface.
- The body also contains `0.30 mm` case clearance around both hip-yaw servos.
  SolidWorks reports `0` cross-component physical interferences.  Its five
  remaining same-component rows are intentional union overlaps between bodies
  inside the one legacy carrier SLDPRT, not collisions between installed parts.
- Official source: `https://docs.m5stack.com/en/core/CoreS3` and the K128
  structure repository.  The generated STEP is a controlled purchased envelope,
  not an instruction to print a substitute controller.

## Independent release gates

1. `CAD_PORTABLE_PASS`: generated STEP/STL/SolidWorks files open and resolve.
2. `KINEMATIC_SYMMETRY_PASS`: paired joint anchors/axes satisfy the v3 mirror rule.
3. `MJX_PRIMITIVE_COLLISION_PASS`: no mesh collision enters the RL model.
4. `GROUND_CONTACT_PASS`: official standing keyframe has two level soles at ground.
5. `ACTUATOR_STATIC_FEASIBILITY_PASS`: nominal whole-body static check is below
   the rated limit, but the detailed quasistatic sweep peaks at `1.414238 N.m`
   on left ankle pitch, above the `1.255251 N.m` continuous design limit and
   below the `1.569064 N.m` rated limit; dynamic walking therefore remains HOLD
   until constrained RL torque/current/thermal traces pass.
6. `MASS_TARGET_PASS`: nominal mass is positive, `<=3.0 kg`, and equals the locked target.
7. `STANDING_HEIGHT_LIMIT_PASS`: native SolidWorks occurrence union is `<=500 mm`.
8. `MASS_IDENTIFIED_PASS`: only after as-built measurements; expected `HOLD` now.
9. `STS3250_FIRST_ARTICLE_PASS`: only after purchased-hardware fit; expected `HOLD` now.
