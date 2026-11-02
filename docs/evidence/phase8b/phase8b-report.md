# Phase 8B — RGB camera geometry and stopping feasibility

Date: 2026-11-02 (Asia/Calcutta)
Project: `/home/ashish/AutonomousNavigationCar`
Result: **Phase 8B assessment complete; autonomous motion gate BLOCKED.** The Phase 8 safe-stop behavior remains locked to zero velocity.

## Scope and implementation

Phase 8B compared camera mount height, downward pitch, ground-projection interval, and floor-seed row on fixed simulated scenarios. The Phase 7 dataset runner now stores each image's live TF transform, accepts bounded camera candidates and a separate evidence root, and supports both requested S02 seeds. The projection evaluator measures connected FREE space across the full robot footprint and both wheel tracks. Mapper optical-axis checks support pitched cameras.

A separate empty-world stopping test measured commanded speeds 0.02, 0.05, 0.10, and 0.18 m/s at friction coefficients 1.2 and 0.5, with three repeats per pair. It used Gazebo pose differences in simulation time to detect settling and evaluator pose only inside that test process. It is isolated from the realistic navigation runtime. No navigation controller, motion permission, or hardware integration was added.

The final seven-package ROS workspace build passed. Phase 6, Phase 7 fault/static, and Phase 8 static validation artifacts are retained. Phase 8 remained stop-only throughout all camera trials: the safe-stop supervisor was the sole `/cmd_vel` publisher and every recorded output was zero.

## Camera candidate results

All views used a 320 x 240 provisional zero-distortion camera with 60-degree horizontal FOV. Mount x/y remained (0.10, 0.00) m relative to `base_link`; the tested mount-height changes were in z. Pixel metrics are offline comparisons against evaluator-only geometry.

| Candidate | Camera height / pitch | Ground interval / seed row | Captured views and image metrics | Full-footprint corridor |
|---|---|---|---|---|
| C0, level baseline | 0.18 m / 0 rad | 0.50-0.60 m / 0.90 | 10 valid views; false-FREE 0; occupied recall 59.83%; known-floor FREE coverage 100%; UNKNOWN 68.23%; median 13.46 FPS; p95 latency 52.53 ms, max 641.04 ms | 0/10 connected; max length 0 m |
| C1, moderate pitch | 0.18 m / 0.45 rad | 0.10-0.75 m / 0.85 | Two 10-view runs, 20 valid views combined; false-FREE 0; occupied recall 0%; known-floor FREE coverage 71.86%; UNKNOWN 47.16%; per-run median 13.69-14.31 FPS and p95 latency 27.08-27.95 ms | 0/20 connected; max length 0 m |
| C2, high pitch | 0.24 m / 0.80 rad | 0.10-0.90 m / 0.75 | 12/12 views passed; false-FREE 0; occupied recall 0% (3,390 known occupied pixels); known-floor FREE coverage 76.29%; UNKNOWN 27.17%; median 14.99 FPS; p95 latency 18.07 ms, max 236.80 ms; timestamp age p95 45.9 ms, max 174 ms | 10/12 connected, both wheel tracks in 10/12; best length 0.48 m; worst 0 m |
| C3, exploratory | 0.18 m / 0.25 rad | 0.10-0.90 m / 0.85 | One live S00 check; 14.35 FPS; 28 ms camera age; 16.75 ms latency | 0 m connected |
| C4, exploratory low mount | 0.12 m / 0.35 rad | 0.05-0.90 m / 0.85 | One live S00 check; 11.35 FPS; 34 ms camera age; 19.86 ms latency | 0 m connected; 0.729 near-footprint FREE fraction still had a 0.105 m UNKNOWN gap |

C2 gave the best geometric corridor, but it classified none of its evaluator-known occupied pixels as OCCUPIED. Its 0.48 m best connected corridor is not a robust obstacle-clearance result. C0 retained higher occupied recall but had no connected full-footprint corridor. C3 and C4 were exploratory S00 checks only, not multi-scenario datasets.

One C0 P7C1 case and one S02 case in each C1 run failed because the separate Gazebo truth service reported "Requested entity not found"; the opposite fixed S02 seed succeeded on each C1 rerun, and all C2 cases passed. These evaluator lookup failures are preserved in the run results and excluded from image aggregates.

The C2 live stop-only run recorded 617 FREE and 5,008 UNKNOWN grid cells, zero OCCUPIED grid cells, 80/80 zero `/cmd_vel` samples, and no displacement above the stop tolerance. The sampled test process peaked at 89.2% CPU and 500,188 KiB RSS; available WSL memory was 2.34 GiB at the end of the live run. Dataset timing and memory are simulated/headless observations, not a hardware performance guarantee.

## Isolated stopping-distance measurements

Settling was detected from finite differences of successive Gazebo evaluator poses in simulation time: linear speed at or below 0.005 m/s and yaw rate at or below 0.01 rad/s held for 0.5 seconds. Gazebo's reported entity twist was stale at low speed, so it is retained as diagnostic data and is not the stopping criterion. Distance is evaluator pose displacement from the zero-command pose to the settled pose, projected along the initial heading.

| Target speed | Mean measured speed at zero, mu=1.2 | Max stop distance, mu=1.2 | Mean measured speed at zero, mu=0.5 | Max stop distance, mu=0.5 |
|---:|---:|---:|---:|---:|
| 0.02 m/s | 0.01776 m/s | 0.000080 m | 0.01659 m/s | 0.000097 m |
| 0.05 m/s | 0.04441 m/s | 0.000283 m | 0.04412 m/s | 0.000368 m |
| 0.10 m/s | 0.08441 m/s | 0.001412 m | 0.08948 m/s | 0.001165 m |
| 0.18 m/s | 0.15988 m/s | 0.001840 m | 0.17294 m/s | 0.002980 m |

All 24 moving trials (4 speeds x 2 friction settings x 3 repeats) were confirmed moving and settled. Maximum truth/odometry stop-distance difference was 0.00356 m. Both simulator groups exited cleanly after SIGINT; there were zero non-ground contact events. The longest zero-command-to-settled confirmation was 0.774 s at 0.18 m/s and mu=0.5; this includes the required 0.5 s hold. The accepted low-friction measurement covers mu=0.5 in this simulator. Earlier mu=0.001 and mu=0.2 pilots produced no traction and are not braking measurements. These figures do not bound hardware stopping distance.

## Motion gate calculation

The gate uses worst observed C2 camera timestamp age and perception latency, the 20 Hz supervisor period, the largest measured stop distance at mu=0.5, the Phase 4 slip-derived localization error, and an explicit 0.10 m safety margin. Forward distance is measured from the robot's front footprint boundary; the full-footprint test includes robot width and a 0.02 m lateral margin on each side.

```text
required forward length =
  v * (camera age + perception latency + supervisor period)
  + measured stop distance
  + localization uncertainty allowance
  + forward footprint margin from this reference plane
  + explicit safety margin

= 0.18 * (0.174 + 0.2368 + 0.050)
  + 0.00298
  + 0.4705050313
  + 0.000
  + 0.100
= 0.6564290313 m
```

The 0.470505 m allowance is the Phase 4 maximum wheel-odometry endpoint XY error measured under mu=0.001 while evaluator chassis displacement was zero. It is used instead of the Phase 8 live EKF's zero reported covariance, which is not a credible uncertainty bound. Forward footprint extent is already included by measuring from the robot front; required corridor width is 0.345 m (0.305 m footprint plus 0.02 m on each side).

The best observed candidate corridor is 0.480 m, 0.176429 m shorter than the calculated requirement; some views had no connected corridor. C2 also had zero occupied recall. **Gate result: BLOCKED.** No realistic controller, RGB Nav2 navigation, or autonomous movement was enabled or tested. The unconditional-zero safe-stop path remains the correct behavior.

## Evidence index

- `camera-datasets/`: fixed-seed candidate datasets, per-view RGB and masks, metrics, and projected-grid evaluation.
- `camera-live/`: candidate live stop-only checks, captured RGB/grid artifacts, graph checks, and resource samples.
- `stopping-distance/run-20261102-full-mu050/`: 24-trial results, telemetry, and both friction-specific Gazebo logs.
- `final-build.log`, `phase6-static-validation.log`, `phase7-static-validation.log`, `phase7-fault-injection.json`, and `phase8-static-validation.json`: final build and regression checks.
- `motion-gate.json`: machine-readable gate inputs and calculation.
- `../../phase4-localization-slip-final4.json`: source for the slip-derived localization allowance.

Phase 8B ends here. Phase 9, parking, hardware transfer, and the Arduino UNO Q application were not changed.
