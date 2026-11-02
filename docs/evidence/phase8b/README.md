# Phase 8B evidence index

Phase 8B remains a headless, simulation-only feasibility assessment. The Phase 8 safe-stop lock remained active during camera candidate runs. Isolated stopping-distance trials ran in a separate empty S00 simulation process.

- [phase8b-report.md](phase8b-report.md): candidate comparison, stopping-distance results, gate calculation, limitations, and decision.
- [motion-gate.json](motion-gate.json): machine-readable input values and BLOCKED result.
- [baseline-build.log](baseline-build.log), [final-build.log](final-build.log): seven-package workspace build evidence; final build passed.
- [phase6-static-validation.log](phase6-static-validation.log), [phase7-static-validation.log](phase7-static-validation.log), [phase7-fault-injection.json](phase7-fault-injection.json), [phase8-static-validation.json](phase8-static-validation.json): regression checks for existing phases.
- [camera-datasets/](camera-datasets/): C0-C2 fixed-scenario sweeps and projected full-footprint evaluation. C1 has two runs to cover both S02 seeds; query failures remain recorded.
- [camera-live/](camera-live/): stationary live checks, actual RGB and projected grids, TF/camera metadata, command graph, and sampled resources for C1-C4.
- [stopping-distance/run-20261102-full-mu050/](stopping-distance/run-20261102-full-mu050/): 24 measured trials at mu=1.2 and mu=0.5, CSV telemetry, and simulator logs.
- The localization allowance comes from [phase4-localization-slip-final4.json](../phase4-localization-slip-final4.json). It is a simulator slip observation, not a hardware-calibrated covariance.

The best measured camera corridor is 0.48 m. The conservative gate requires 0.656429 m and a 0.345 m corridor width; C2 also recorded 0% occupied recall. The gate stays BLOCKED and no RGB navigation movement was attempted.