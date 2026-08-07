# Zeroth-01 v4 original-minimal 18DoF

This is the repository's single recommended mechanical baseline for RL. It restores the original Zeroth-01 load path, shoulder/hip spacing, limb interfaces and compact proportions. Changes are limited to a nominal +5 mm head envelope on each side with 5 mm edge radii, direct no-neck mounting, two ankle-roll joints, 7 mm soles, an 18 mm reduction in each lower-leg straight span, compact fixed palms, and a reversible white rear service pod.

ToddlerBot, KHR-3HV, TonyPi and Open Duck Mini inform maintainability, double support, cable routing and MJX/SysID workflow; none replaces Zeroth-01 geometry or topology.

## Authoritative files

- SolidWorks: `generated/solidworks/physical_mount_v4_original_minimal/portable_flat/OPEN_FIRST_ZEROTH01_V4_ORIGINAL_MINIMAL_WHITE_18_BLUE_STS3250.SLDASM`
- X-ray SolidWorks: `generated/solidworks/physical_mount_v4_original_minimal/portable_flat/OPTIONAL_XRAY_ZEROTH01_V4_ORIGINAL_MINIMAL_INTERNAL_LAYOUT.SLDASM`
- STEP: `generated/cad/physical_mount_v4_original_minimal/ZEROTH01_V4_ORIGINAL_MINIMAL_18DOF_FULL_ASSEMBLY.step`
- URDF: `generated/urdf/physical_mount_v4_original_minimal/zeroth01_physical_mount_v4_original_minimal_18dof.urdf`
- MJCF/MJX: `generated/mujoco/physical_mount_v4_original_minimal/zeroth01_physical_mount_v4_original_minimal_18dof_mjx.xml`
- Actuator ledger: `generated/config/physical_mount_v4_original_minimal_actuator_layout.json`
- RL handoff: `generated/config/physical_mount_v4_original_minimal_rl_handoff.json`
- Release gate: `reports/v4_original_minimal/release_gate.json`

Digital gates pass at 18 joints, 57 SolidWorks components, 18 independent STS3250 occurrences, 498.959 mm standing height, 2.850 kg nominal mass, zero physical cross-component interference, runtime-compilable MuJoCo, and zero non-ground penetration in the 64-pose coordinated sweep. Quasi-static gravity torque peaks at 0.534496 N·m, below the 1.255251 N·m continuous design limit.

The digital baseline is released for RL; the physical first article remains **HOLD** pending purchased-actuator inspection, bus/zero/direction calibration, as-built mass and SysID, fused current/thermal testing, and dynamic policy traces. See `README_zh.md` and `ASSEMBLY_GUIDE_zh.md`.
