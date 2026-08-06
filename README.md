# Zeroth-01 Physical Mount v3 RL-Fixed

This repository's only canonical RL/mechanical baseline is **v3 RL-Fixed**. It preserves the released v2 load-bearing assembly and all 16 original 6D joint frames, axes, and limits; adds two serial 50 mm ankle-roll stages; uses 18 separately identifiable FEETECH STS3250-C001 actuators; and replaces the custom head with a purchasable M5Stack StackChan K151 on a reversible 3 mm 6061 adapter.

Start with [README_zh.md](README_zh.md), then consume only:

- `generated/urdf/physical_mount_v3_rl_fixed/zeroth01_physical_mount_v3_rl_fixed_18dof.urdf`
- `generated/mujoco/physical_mount_v3_rl_fixed/zeroth01_physical_mount_v3_rl_fixed_18dof_mjx.xml`
- `generated/config/physical_mount_v3_rl_fixed_actuator_layout.json`
- `generated/config/physical_mount_v3_rl_fixed_rl_handoff.json`
- `reports/physical_mount_v3_rl_fixed/release_gates.json`

Nominal mass is 3.095471828 kg. The native SolidWorks assembly contains 51/51 components and has zero physical volume intersections; three permitted overlaps are display-only K151 face references. All S01–S18 URDF visuals use the same dimension-controlled STS3250 mesh and their full 6D installed transforms match the SolidWorks manifest. URDF-to-SolidWorks FK, joint-limit sweeps, ground contact, coordinated 64-frame motion, and nominal quasi-static torque all pass. Peak quasi-static torque is 1.186408681 N·m at the left ankle pitch, 94.52% of the 1.2552512 N·m continuous design limit.

Open first:

- `generated/solidworks/physical_mount_v3_rl_fixed/portable_flat/OPEN_FIRST_ZEROTH01_V3_RL_FIXED_CONNECTED_WHITE_18_BLUE_STS3250.SLDASM`
- optional x-ray: `generated/solidworks/physical_mount_v3_rl_fixed/portable_flat/OPTIONAL_XRAY_ZEROTH01_V3_RL_FIXED_18_BLUE_STS3250.SLDASM`

Digital RL release is PASS. Physical release remains on HOLD until one purchased STS3250 and the K151 adapter pass first-article fit, as-built mass/COM/inertia are identified, and an RL rollout plus instrumented robot demonstrate acceptable peak/RMS torque, current, and thermal behavior.
