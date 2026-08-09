# Zeroth-01 v5 16DoF RL milestones

This directory stores only checkpoints that passed an explicit milestone gate.
The integer in each milestone name is the PPO update number, not an assertion of
walking distance or convergence.

## Current status

| Milestone | PPO update | Status | Evidence |
| --- | ---: | --- | --- |
| standing (`N`, canonical `gait_neutral`) | 12 | PASS | 12/12 deterministic 4 s episodes healthy; mean return 1277.60 versus random 290.93 |
| standing (`N`, historical `official_standing`) | 12 | PASS / regression only | retained for old-model replay; not a locomotion reset |
| walking (`M`) | — | NOT ACHIEVED | canonical updates 16–64 all failed; best fully surviving update 24 reached only 0.0040 m/s |
| running (`Z`) | — | NOT ACHIEVED | walking gate has not been passed |

`N12_gait_neutral_standing/ckpt.12.bin` is the current canonical standing smoke
checkpoint. `N12_standing/ckpt.12.bin` is the historical all-zero standing
regression checkpoint. Neither is a walking or running policy, and neither may
be used as evidence of locomotion or hardware readiness.

The canonical `gait_neutral` walking stage ran through PPO update 64 with 128
environments on the same RTX 4070 Laptop GPU. Update 20 was fastest at 0.0125
m/s but survived only 3/4 episodes. The best fully surviving policy, update 24,
reached 0.0040 m/s with only 0.75% single support. Later policies converged back
to double support. No checkpoint met the 0.05 m/s gait gate, so rejected walk
checkpoints are not stored in this repository and running training did not start.

The policy action is 16-dimensional: all six arm joints and all ten leg joints
are policy outputs. MuJoCo actuator torque is clamped to the mechanical baseline
continuous limit of +/-1.2552512 N m at every joint.

See each checkpoint's `metadata.json` for its exact mechanical source commit,
model hashes, training configuration, GPU restore result and acceptance data.

The historical N12 policy was trained from the historical source MJCF plus the runtime-only
index/keyframe repair recorded by those two hashes. The current canonical MJCF
now applies that index repair at generation time and adds the validated
`gait_neutral` reset. Keep N12 as a standing regression checkpoint; new walking
training must use `gait_neutral` and record the canonical MJCF SHA-256,
16-joint control order and source commit in its own metadata. The current N12
does so and also records the open embedded-hash inconsistency in BUG-RL-V5-006.
