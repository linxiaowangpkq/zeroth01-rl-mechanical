# Zeroth-01 v4 connected design brief

## Frozen baseline

- Preserve the released Zeroth-01 load-bearing torso, shoulder/hip spacing, upper limbs, thighs, source feet and all neutral joint axes.
- ToddlerBot, KHR-3HV, TonyPi and Open Duck Mini inform serviceability, double support, cable routing and MJX/SysID workflow only; their geometry is not substituted.
- The v3 RL frame/axis corrections remain frozen.

## Released changes

1. Two-piece rounded head, nominally +5 mm left/right/top/bottom, source depth retained, hidden 0.8 mm shoulder clearances, no visible neck.
2. Removable M5Stack UnitV2 camera/microphone cradle and direct four-M3 torso nut plate.
3. Original lightweight wrist termination only: no square palm, claw, gripper, ball hand or heavy Q-hand.
4. No external rear service pod or backplate. Compute, battery and IMU packaging must be frozen inside the torso after exact components are selected.
5. No added black sole; source feet define the contact surface.
6. Eighteen purchased-exact STS3250 occurrences. Every output has a 2.05 mm PCD14 bridge; required child-side standoffs close the torque path.
7. Hip-yaw servos move 4 mm only along their unchanged axes to clear the source torso. Four M2 case screws and four M3 PCD14 output tie rods close each joint; no fictitious solid spacer is installed.
8. Mirrored direct-drive ankle-roll carriers retain the original foot path; lower-leg straight spans are shortened 18 mm without altering terminal interfaces.

## Blocking gates

- 18/18 exact actuators and complete case/output torque paths.
- Bilateral axis/placement symmetry.
- 75/75 native SolidWorks occurrences and portable dependencies.
- Zero unapproved B-Rep cross-component interference. Threaded fastener engagement is allowed only for the explicitly paired PCD14 M3 shanks and only below 1.25 mm³ per detected engagement.
- Height ≤500 mm; nominal mass ≤3 kg.
- URDF/MJCF compile and 64-pose primitive collision sweep passes.
- Physical walking/running remains HOLD until first-article fit, printed strength, internal electronics/harness packaging, measured mass/SysID, current, thermal and policy traces are complete.
