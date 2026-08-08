# Zeroth-01 v5 original 16-DoF physical/RL baseline

v5 restores the released Zeroth-01 16-joint topology and proportions. It adds no ankle-roll motors, shortened shins, claws or large palms. The bounded changes are a rounded head enlarged by 5 mm on every side, a removable M5Stack UnitV2 camera/microphone, a direct no-neck head mount, two compact fixed wrist supports, two 9 mm white lower-wider replaceable soles, and explicit FEETECH STS3250 housing/output interfaces.

## Authoritative RL files

- URDF: `generated/urdf/physical_mount_v5_original_16dof_solidworks_motion/zeroth01_physical_mount_v5_original_16dof_solidworks_motion.urdf`
- MJCF/MJX: `generated/mujoco/physical_mount_v5_original_16dof_solidworks_motion/zeroth01_physical_mount_v5_original_16dof_solidworks_motion_mjx.xml`
- Actuator layout: `generated/config/physical_mount_v5_original_16dof_solidworks_motion_actuator_layout.json`
- Hardware calibration template: `generated/config/physical_mount_v5_original_16dof_solidworks_motion_hardware_calibration.csv`
- RL handoff: `generated/config/physical_mount_v5_original_16dof_solidworks_motion_rl_handoff.json`
- SHA-256 delivery ledger: `generated/config/physical_mount_v5_original_16dof_solidworks_motion_delivery_manifest.json`
- Release truth boundary: `reports/v5_original_16dof_solidworks_motion/release_gate.json`
- 85-component CAD manifest: `generated/cad/physical_mount_v5_original_16dof_solidworks_motion/ZEROTH01_V5_ORIGINAL_16DOF_SOLIDWORKS_MOTION_ASSEMBLY_MANIFEST.json`
- 85-component coloured neutral review GLB: `generated/cad/physical_mount_v5_original_16dof_solidworks_motion/ZEROTH01_V5_ORIGINAL_16DOF_NEUTRAL_REVIEW.glb`
- 17 rigid-link STEP files: `generated/cad/physical_mount_v5_original_16dof_solidworks_motion/motion_link_parts/`

Current digital evidence: 16 revolute joints and 16 purchased-exact STS3250 STEP occurrences; 85 components; 17 positive-volume valid B-Rep motion links; zero positive interference among all 3,570 neutral-pose component pairs; 490.489 mm CAD envelope height; 416.448 mm MJCF standing kinematic height; 2.745759 kg nominal URDF/MJCF mass; and MuJoCo 3.3.7 compile at `nq=23`, `nv=22`, `nu=16`.

The hip-yaw case path is physical CAD rather than a floating marker: two integrated torso ribs/bosses per side accept two M2x8 rear-case screws, while a PCD14 four-hole bridge and four M3 sleeves drive the U-hip. The mount and screw stack is clear of each moving U-hip over its unchanged full joint limit.

Native SOLIDWORKS static rebuild and 32 isolated lower/upper-limit Motion runs for the current manifest remain **HOLD** because the existing SOLIDWORKS process is unresponsive. Old native gate files are stale and are not acceptance evidence; no Motion GIF is claimed until the current assembly produces real frames.

This is a digital RL release, not a physical manufacturing sign-off. Purchased-part fit, printed strength, screw/tool access, exact internal electronics, harness flex, bus IDs/zeros/directions, as-built link mass/COM/inertia, current, thermal behavior and policy traces require first-article measurement and SysID.
