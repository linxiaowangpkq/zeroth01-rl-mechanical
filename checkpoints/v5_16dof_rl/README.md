# Zeroth-01 v5 16DoF RL milestones

This directory stores only checkpoints that passed an explicit milestone gate.
The integer in each milestone name is the PPO update number, not an assertion of
walking distance or convergence.

## Current status

| Milestone | PPO update | Status | Evidence |
| --- | ---: | --- | --- |
| standing (`N`) | 12 | PASS | 12/12 deterministic 4 s episodes healthy; 10 s replay upright fraction 1.0 |
| walking (`M`) | — | NOT ACHIEVED | checkpoint 12 mean forward speed is only 0.0021 m/s |
| running (`Z`) | — | NOT ACHIEVED | walking gate has not been passed |

`N12_standing/ckpt.12.bin` is therefore a standing smoke checkpoint only. It is
not a walking or running policy and must not be used as evidence of locomotion or
hardware readiness.

The policy action is 16-dimensional: all six arm joints and all ten leg joints
are policy outputs. MuJoCo actuator torque is clamped to the mechanical baseline
continuous limit of +/-1.2552512 N m at every joint.

See `N12_standing/metadata.json` for the exact mechanical source commit, model
hashes, training configuration, GPU restore result, and acceptance measurements.
