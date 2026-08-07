# Zeroth-01 v4 original-minimal design brief

## Frozen baseline

- The official Zeroth-01 source geometry, neutral joint centres, limb lengths,
  shoulder width, hip spacing and torso proportions are the only geometric
  baseline.
- The released v3 18-DoF ankle-roll correction and RL frame/axis fixes remain
  in scope. They may not silently move an existing source joint.
- ToddlerBot, KHR-3HV, TonyPi and Open Duck Mini are engineering references,
  not replacement geometries. No third-party shape is copied into v4.

## Allowed geometry changes

1. Replace only the source head cover by a two-piece printable head cover.
2. Relative to the measured source head cover envelope, add a nominal 5 mm
   outer envelope on left, right, top and bottom (10 mm total width/height).
   The hidden lower corners contain exact shoulder-servo B-Rep pockets with
   0.8 mm clearance; their local solid expansion is about 3.9 mm. Extending
   those corners to a solid 5 mm would recreate the measured interference.
3. Keep the source depth unless a purchased camera/microphone module needs a
   documented clearance increase. Use a small edge radius only.
4. Mount the head directly to the original torso datum. No visible neck and a
   maximum nominal head-to-torso gap of 2 mm.
5. Preserve the original hands and arms. A light removable wrist bumper is
   allowed, but no gripper, claw, ball hand or heavy Q-hand.
6. Preserve the source upper and lower body carriers. New service parts may
   add real fasteners, double-supported servo brackets, cable strain relief,
   battery/compute trays and removable covers without changing kinematics.
7. Retain the lightened 7 mm sole and the 18-DoF ankle-roll correction only if
   the regenerated interference and motion gates pass.
8. The only released proportion change below the torso is an 18 mm cut from
   each lower leg's straight middle span; both terminal interfaces are kept.
   This recovers the direct ankle-roll stack height without changing axes.

## Engineering patterns borrowed without copying geometry

- ToddlerBot: split printable shells, maintainable electronics packaging,
  SysID and simulation handoff discipline.
- KHR-3HV: double-supported servo output and replaceable U-bracket practice.
- TonyPi: serviceable camera, compute and battery placement.
- Open Duck Mini: compact printable links, cable relief and assembly-oriented
  part splitting.

## Release gates

- Exactly 18 separately visible blue STS3250 controlled envelopes.
- Joint origins and axes are generated once and shared by CAD, URDF and MJCF.
- Bilateral symmetry is a blocking release gate; failures may not be omitted.
- Native SolidWorks assembly opens without suppressed/missing components.
- Zero non-adjacent interference at neutral and sampled joint motion.
- Standing height no greater than 500 mm; nominal mass no greater than 3 kg.
- Camera, microphones, IMU, compute, battery, power board and harness service
  volumes have physical mounts and cable exits.
- Walking/running hardware release remains HOLD until purchased-actuator,
  as-built mass/inertia, current, thermal and dynamic-policy traces exist.
