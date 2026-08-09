# Zeroth-01 v5 16DoF RL model contract

This ledger freezes the assumptions shared by MuJoCo/MJX training, checkpoint
metadata and later hardware deployment. The generated source of truth is
`build_v5_mjcf.py`; `reports/v5_original_16dof_solidworks_motion/mjcf_compile_gate.json`
contains the machine-checked mapping and the SHA-256 of the resulting MJCF.

## Frames and units

- SI units: metres, kilograms, seconds, radians and newton-metres.
- World/body-neutral axes: +X forward, +Y left, +Z up.
- Floating-base `qpos[0:7]`: position XYZ followed by MuJoCo quaternion WXYZ.
- Floating-base `qvel[0:6]`: linear XYZ followed by angular XYZ velocity.
- The released CAD/URDF joint axes, limits, geometry, collision primitives,
  mass, COM, inertia and the 1.2552512 N m continuous training torque cap are
  unchanged by the RL contract repair.

## Canonical index order

The compiler's depth-first hinge order is authoritative:

```text
0  left_shoulder_yaw       qpos 7   qvel 6
1  left_shoulder_pitch     qpos 8   qvel 7
2  left_elbow_yaw          qpos 9   qvel 8
3  right_shoulder_yaw      qpos 10  qvel 9
4  right_shoulder_pitch    qpos 11  qvel 10
5  right_elbow_yaw         qpos 12  qvel 11
6  left_hip_yaw            qpos 13  qvel 12
7  left_hip_roll           qpos 14  qvel 13
8  left_hip_pitch          qpos 15  qvel 14
9  left_knee_pitch         qpos 16  qvel 15
10 left_ankle_pitch        qpos 17  qvel 16
11 right_hip_yaw           qpos 18  qvel 17
12 right_hip_roll          qpos 19  qvel 18
13 right_hip_pitch         qpos 20  qvel 19
14 right_knee_pitch        qpos 21  qvel 20
15 right_ankle_pitch       qpos 22  qvel 21
```

`ctrl[i]`, policy action `i` and the joint on row `i` above are identical.
Generation fails if actuator order, compiled order or qpos/qvel addresses
diverge. Checkpoints must record this order and the canonical MJCF SHA-256.

## Reset poses

- `official_standing` is the released all-zero calibration pose. It is retained
  for compatibility and standing-checkpoint replay, but four knee/ankle joints
  are on a one-sided model limit, so it is not the locomotion reset.
- `gait_neutral` is the required training reset. Its leg values are left
  `(+0.18, -0.36, -0.18)` and right `(-0.18, +0.36, +0.18)` for
  hip-pitch, knee-pitch and ankle-pitch. Its base Z is solved from both sole
  boxes. `symmetric_crouch` is an exact backward-compatible alias.
- Generation checks all values against limits, both sole bottoms against the
  ground, left/right world mirror error and at least 5 degrees of positive and
  negative knee/ankle travel.

## Foot sites and unresolved physical truth

Foot sites are named by world semantics: front sites have greater world X than
rear sites, and medial sites have smaller absolute world Y than lateral sites.
Both conditions are checked from `official_standing` forward kinematics.

The SolidWorks physical-zero audit and powered servo zero/direction calibration
remain HOLD. Therefore this repair does not reinterpret physical hard stops or
claim that an encoder count corresponds to `official_standing` or
`gait_neutral`; those mappings belong in the hardware calibration CSV after
first-article measurement.
