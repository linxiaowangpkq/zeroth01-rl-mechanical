# Zeroth-01 Physical Mount V1

This release replaces the superseded blue servo-envelope overlay with a
source-derived 16-DoF mechanical chain.

- 16 installed STS3215-family actuator regions are extracted in-place from
  the pinned Zeroth/K-Scale assembled meshes.
- 20 carrier parts and 16 blue actuator reference parts are delivered as 36
  separate SolidWorks surface parts and 20 rigid link subassemblies.
- Joint origins/axes come from the source URDF; all 16 shaft-to-case offsets
  pass the nominal 12.5 mm audit.
- MuJoCo passes neutral, 61 samples over each guarded joint range, and 73
  coordinated motion poses with zero nonadjacent-link collision failures.
- The canonical RL model is
  `generated/urdf/physical_mount_v1/zeroth01_physical_mount_v1.urdf`.
- Full-robot 3D-print release remains **HOLD** until one purchased STS3250
  passes the included 4×M2/horn/rear-axis first-article gauge.

The installed blue parts are deliberately named
`INSTALLED_STS3215_FAMILY_REFERENCE`; they are not misrepresented as exact
vendor STS3250 B-Rep CAD. A separate dimension-controlled STS3250 reference
STEP and first-article gauge are included.

Read [README_zh.md](README_zh.md) for the complete Chinese handoff, evidence,
assembly order, known source asymmetries and RL claim boundaries.
