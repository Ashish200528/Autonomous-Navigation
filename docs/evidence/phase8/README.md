# Phase 8 evidence

Phase 8 is accepted only as a **PASS WITH LIMITS stop-only baseline**. Autonomous movement is blocked by the evidence gate.

- `phase8-summary.json`: machine-readable aggregate for the nine final live stop runs, static tests, stale-to-fresh perception recovery, feasibility calculations, and the separately labeled Phase 6 comparison.
- `phase8-feasibility.json`: camera projection, timing, localization, braking-data gap, and motion gate.
- `phase8-static-validation.json` and `phase8-static-validation.log`: seven projection, UNKNOWN-preservation, confidence, and distortion checks.
- `phase8-build.log`: final full workspace build; seven packages finished.
- `phase7-fault-injection.json` and `phase7-fault-console.log`: missing RGB, fresh input, stale timestamp, and recovery after fresh RGB returns.
- `runs/<run-id>/result.json`, `telemetry.csv`, and `launch.log`: bounded live Gazebo stop trial, request/output samples, evaluator-only truth, contact and resource data, ROS graph checks, and launch teardown.
- `worlds/s02-seed202.sdf` and `worlds/s02-seed202-fixture.json`: distinct deterministic S02 seed fixture used only to stage the test world; runtime nodes did not receive the layout or manifest.
- Phase 6 S00 reference comparison: `../phase6/runs/phase8-final-reference-s00-20261102/result.json` and associated bag/launch evidence.

The live trials cover S00 camera absent, S00 fresh RGB, S00 missing filtered localization, S00 command timeout, S01, S02 seeds 101 and 202, S03, and S09. All actual `/cmd_vel` samples were zero and all evaluator pose displacements were below 0.02 m. Repeated caster-to-ground contact records are surface telemetry, not collision episodes; no non-ground contact was recorded.

Earlier diagnostic attempts under `phase8/` are retained. The accepted final run IDs above supersede failed harness synchronization attempts. Do not interpret stop tests as motion or goal-navigation trials.
