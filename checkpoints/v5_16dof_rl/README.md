# Zeroth-01 v5 16DoF RL milestones

This directory stores only checkpoints that passed an explicit milestone gate.
The integer in each milestone name is the PPO update number, not an assertion of
walking distance or convergence.

## Current status

| Milestone | PPO update | Status | Evidence |
| --- | ---: | --- | --- |
| standing (`N`) | 12 | PASS | 12/12 deterministic 4 s episodes healthy; 10 s replay upright fraction 1.0 |
| walking (`M`) | — | NOT ACHIEVED | warm-start updates 16–64 all failed the gait gate; fastest was update 16 at 0.0057 m/s |
| running (`Z`) | — | NOT ACHIEVED | walking gate has not been passed |

`N12_standing/ckpt.12.bin` is therefore a standing smoke checkpoint only. It is
not a walking or running policy and must not be used as evidence of locomotion or
hardware readiness.

The follow-up walking stage ran through PPO update 64 with 128 environments on
the same RTX 4070 Laptop GPU. All screened checkpoints survived 4 s, but none
reached the required 0.05 m/s mean forward speed. Update 64 had zero single-
support frames and 100% double support, so rejected walk checkpoints are not
stored in this repository.

The policy action is 16-dimensional: all six arm joints and all ten leg joints
are policy outputs. MuJoCo actuator torque is clamped to the mechanical baseline
continuous limit of +/-1.2552512 N m at every joint.

See `N12_standing/metadata.json` for the exact mechanical source commit, model
hashes, training configuration, GPU restore result, and acceptance measurements.

The N12 policy was trained from the historical source MJCF plus the runtime-only
index/keyframe repair recorded by those two hashes. The current canonical MJCF
now applies that index repair at generation time and adds the validated
`gait_neutral` reset. Keep N12 as a standing regression checkpoint; new walking
training must use `gait_neutral` and record the new canonical MJCF SHA-256,
16-joint control order and source commit in its own metadata.
