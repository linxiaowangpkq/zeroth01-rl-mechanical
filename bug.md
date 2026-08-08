# v4 connected mechanical issue ledger

## Resolved in `codex/zeroth01-v4-connected`

- Removed non-load-bearing square palms/hand blocks; original lightweight wrist termination remains.
- Removed the external rear service pod/backplate and its exposed payload envelopes from the installed CAD, URDF and MJCF.
- Removed added black 7 mm soles; source feet are the only installed contact geometry.
- Replaced simplified servo boxes with 18 purchased-exact FEETECH STS3250 STEP occurrences.
- Restored original Zeroth joint axes/carriers; left/right shoulder, hip, knee and ankle placements are generated from the same joint ledger.
- Replaced the asymmetric/remote ankle experiment with mirrored direct-drive ankle-roll carriers and explicit PCD14 output bridges.
- Added an explicit case/output torque path at all 18 joints.
- Cleared the exact hip-yaw servo/torso clash with a 4 mm axial-only shift and 4×M2 case plus 4×M3 output fastener stacks; joint axes remain unchanged.
- Removed full-disc hip spacers that collided with the servo horn/body.
- SolidWorks B-Rep gate: 0 unapproved physical interferences. Eight bounded detections are the intended M3 screw engagement with the two STS3250 PCD14 tapped outputs; four are zero-volume contact.
- MuJoCo 64-pose coordinated sweep: 0 non-ground penetrations.

## Open physical-first-article blockers

- Verify purchased servo tolerances, output spline/threads and screw lengths on one joint coupon.
- Freeze the actual compute board, battery/BMS, IMU, fuse, emergency stop and internal torso trays.
- Route and flex-test the complete harness through joint limits.
- Measure bus IDs, mechanical zeros, direction signs, as-built link mass/COM/inertia and SysID parameters.
- Validate printed strength, fastener access/retention, current, voltage drop, thermal behavior and dynamic-policy torque traces.

These blockers do not invalidate the digital RL baseline, but they prohibit changing `physical_first_article_gate` from `HOLD`.
