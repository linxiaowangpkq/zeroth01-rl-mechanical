# STS3250 / Zeroth-01 round-v1 feasibility

- Round-v1 nominal mass: `4.997343 kg`
- Baseline mass: `3.095472 kg`
- Static samples: `100000`
- Worst sampled quasi-static joint torque: `0.339869 N.m` at `right_hip_pitch`
- Manufacturer rated / legacy simulation / stall: `1.569064 / 2.000000 / 4.903325 N.m`
- Quasi-static gravity gate versus rated torque: **PASS**
- Walking feasibility: **UNVERIFIED**

Static gravity margin does not validate walking. A trained policy must pass torque-speed, RMS current, thermal, impact, tracking-error and bus-voltage tests before STS3250 is accepted for hardware walking.
