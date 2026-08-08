# Zeroth-01 v4 connected 18DoF

This is the single recommended mechanical/RL baseline in this repository. It keeps the released Zeroth-01 load-bearing torso, shoulders, arms, hips, thighs, joint axes and source feet. Changes are limited to a +5 mm-per-side rounded two-piece head with a removable M5Stack UnitV2 camera/microphone, direct no-neck mounting, two direct-drive ankle-roll joints, 18 mm shorter straight lower-leg spans, and explicit STS3250 case/output fastener stacks.

The former square palms, external rear service pod, added black 7 mm soles, remote-ankle experiment and fictitious full-disc hip spacers are not installed and their stale native parts were removed. Compute, battery and IMU must ultimately be packaged inside the torso; exact hardware/trays remain a physical first-article HOLD.

## Authoritative files

- SolidWorks: `generated/solidworks/physical_mount_v4_original_minimal/portable_flat/OPEN_FIRST_ZEROTH01_V4_ORIGINAL_MINIMAL_WHITE_18_BLUE_STS3250.SLDASM`
- X-ray SolidWorks: `generated/solidworks/physical_mount_v4_original_minimal/portable_flat/OPTIONAL_XRAY_ZEROTH01_V4_ORIGINAL_MINIMAL_INTERNAL_LAYOUT.SLDASM`
- Full assembly STEP: `generated/cad/physical_mount_v4_original_minimal/ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY.step`
- URDF: `generated/urdf/physical_mount_v4_original_minimal/zeroth01_physical_mount_v4_original_minimal_18dof.urdf`
- MJCF/MJX: `generated/mujoco/physical_mount_v4_original_minimal/zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml`
- Actuator ledger: `generated/config/physical_mount_v4_original_minimal_actuator_layout.json`
- RL handoff: `generated/config/physical_mount_v4_original_minimal_rl_handoff.json`
- Release gate: `reports/v4_original_minimal/release_gate.json`

Current digital evidence: 18 movable joints; 75/75 SolidWorks occurrences; 18 purchased-exact STS3250 STEP occurrences; 489.989 mm SolidWorks height; 2.850 kg nominal URDF/MJCF mass; zero unapproved physical cross-component interference; MuJoCo `nq=25`, `nv=24`, `nu=18`; and zero non-ground penetration in a 64-pose coordinated sweep. Peak nominal quasi-static gravity torque is 0.474776 N·m at the left ankle pitch, below the 1.255251 N·m continuous design limit.

This is a digital RL release, not a print-ready manufacturing sign-off. Purchased-servo fit, printed strength, fastener access, internal electronics packaging, harness flex, bus IDs/zeros/directions, as-built mass/inertia, current, thermal behavior and dynamic policy traces remain HOLD.
