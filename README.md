# Zeroth-01 v3 RL-Fixed Mechanical Package

This repository publishes the canonical 18DoF mechanical/RL baseline derived from the connected Zeroth-01 v2 carrier assembly. The old custom head, long neck, chest add-on plate, claws, K151 concept, 9 mm soles and 50 mm ankle-roll draft are not release inputs.

Open first in SolidWorks:

`generated/solidworks/physical_mount_v3_rl_fixed/portable_flat/OPEN_FIRST_ZEROTH01_V3_RL_FIXED_CONNECTED_WHITE_18_BLUE_STS3250.SLDASM`

Canonical RL inputs:

- URDF: `generated/urdf/physical_mount_v3_rl_fixed/zeroth01_physical_mount_v3_rl_fixed_18dof.urdf`
- MuJoCo/MJX: `generated/mujoco/physical_mount_v3_rl_fixed/zeroth01_physical_mount_v3_rl_fixed_18dof_mjx.xml`
- actuator shaft frames: `generated/config/physical_mount_v3_rl_fixed_actuator_layout.json`
- mass, sensor, randomization and gate handoff: `generated/config/physical_mount_v3_rl_fixed_rl_handoff.json`
- release gates: `reports/physical_mount_v3_rl_fixed/release_gates.json`

Verified digital facts: 18 revolute joints, 18 separately addressable blue STS3250-C001 envelopes, nominal URDF/MJCF mass `2.969171828 kg`, conservative native SolidWorks height `499.236652 mm`, diagnostic STEP height `494.817994 mm`, 51/51 native assembly components, zero cross-component physical volume intersections, and a 64-frame MuJoCo motion sample with zero non-adjacent penetrations.

The interaction module is a fixed, purchasable M5Stack CoreS3 K128 (`54 × 54 × 15.5 mm`, conservatively assigned `72.7 g`) flush-mounted in the upper torso. Its camera, dual microphones, speaker, display and IMU frames are in the URDF; a reversible 2 mm 6061 U-cradle and matching body pocket/M3 passages replace the former neck/head assembly. Compute, battery and torso IMU are hidden in the normal assembly and visible only in the optional X-ray assembly.

Digital RL release is ready, but physical and dynamic release remains on HOLD. The coordinated quasistatic sweep peaks at `1.414238 N·m` on left ankle pitch: below the `1.569064 N·m` rated value but above the `1.255251 N·m` continuous design value. Training must enforce torque/current/thermal penalties and return peak/RMS traces before STS3250 walking is accepted. Purchased-servo fit, CoreS3 cradle fit, bus IDs, zero counts, direction signs, and as-built mass/COM/inertias also require first-article measurement.

See [README_zh.md](README_zh.md), [ASSEMBLY_GUIDE_zh.md](ASSEMBLY_GUIDE_zh.md), and [one-seq.md](one-seq.md).
