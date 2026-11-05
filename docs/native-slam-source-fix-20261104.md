# Scan endpoint contract correction — 2026-11-04

## Finding and controlled correction

The S01 bias is reproducible in actual scan-stamped SLAM output. A passive baseline trace recorded raw/conditioned scans, wheel/fused odometry, TF, SLAM `/pose`, graph markers, IMU and independent Gazebo 3D truth. No diagnostic node publishes runtime data. Offline analysis interpolates timestamp brackets (maximum gap .15 s), not callback-time truth or latest TF.

Baseline: 40 accepted SLAM poses, maximum scan-stamped XY error .23446 m. Error first exceeded .10 m at scan time 29.8 s, after .466 m of physical travel; wheel error was .00034 m and fused error .00213 m at that instant. Individual scan corrections accumulated in roughly .01 m steps. Existing graph vertices were not revised. Scan receipt age was .006–.026 s; static transforms put LiDAR at (0,0,.16) m above base_footprint with zero yaw. Maximum physical pitch was .00359 rad. Raw ranges versus independently evaluated planar scenario geometry had median absolute error .00808 m, 95th percentile .02676 m, 99th .04323 m, and an edge/outlier maximum 4.72 m. These measurements rule against a uniform .23 m sensor offset or timestamp delay; they do not claim every rendered beam is geometrically exact.

The conditioner encoded 10,114 raw `+Inf` no-return beams as exactly `range_max=6.0` m in 389 paired scans. That is appropriate for the project's local ray-clearing contract, but not as finite hit endpoints for Karto. [Karto's point construction](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.8.5/lib/karto_sdk/include/karto_sdk/Karto.h) retains finite no-return/over-range endpoints in its unfiltered point set; [its matcher](https://raw.githubusercontent.com/SteveMacenski/slam_toolbox/2.8.5/lib/karto_sdk/src/Mapper.cpp) uses that set in `FindValidPoints`. Merely lowering `max_laser_range` does not remove those endpoints from that path. A compiled probe against this laptop's installed headers/library confirms 360 finite endpoints for one real hit plus 359 finite no-returns, even at a lower range threshold; NaN encoding leaves the one real endpoint. Installed binary/header hashes and the exact compile command are preserved.

The correction adds an **opt-in separate SLAM scan**: actual finite hit ranges strictly between min/max retain their values and beam indices; no-return and invalid endpoints become NaN. Header, timestamp, angular geometry and beam count are unchanged. `/realistic/scan`, its health, local ray clearing, RGB fusion, costmaps and the authoritative motion guard retain their original inputs. The original SLAM YAML remains the default. `slam_toolbox_hit_endpoints.yaml` differs only in scan topic; scan matching, loop closing, odometry integration and all matching weights remain enabled/unchanged. This is an input contract fix, not odometry-only localization.

## Gazebo verification

| Run | Actual outcome | Final localization error | Goal error / truth path |
|---|---|---:|---:|
| Instrumented baseline S01 observation leg | FAIL localization; action SUCCEEDED | .2340 m | intermediate goal only |
| Corrected S01 observation 0 | PASS, goal and stop | .00439 m | .1281 m / 1.3609 m |
| Corrected S01 observation 1 | PASS, goal and stop | .04980 m | .1657 m / 2.5579 m cumulative |
| Corrected S01 original goal, 300 s mission | FAIL / TIMEOUT, safe stop | .02289 m | 2.6459 m / 4.5322 m cumulative |
| Corrected direct S00 2 m | PASS / SUCCEEDED, safe stop | .00546 m | .1360 m / 1.8877 m |
| Corrected direct S00 3 m | PASS / SUCCEEDED, safe stop | .02189 m | .1461 m / 2.8814 m |

The corrected S01 trace had maximum scan-stamped SLAM error .02301 m over 112 aligned poses. During a turn, SLAM corrected about .05 m of wheel/EKF error, demonstrating active scan localization rather than simple odometry propagation. Continuous TF-based error can exceed the scan-stamped error between accepted scans; both measurements are saved. All 1,474 paired SLAM scan messages preserved hit values and metadata with zero encoding mismatches. S00 2/3 m maximum scan-stamped errors were .01187/.02945 m.

These completed runs recorded no non-ground collisions, no mixed guard outputs and confirmed final stops. Corrected S01 minimum conservative clearance was .50068 m. Original-goal completion did **not** pass. Production limits, unknown-space denials, fail-closed runtime timeouts, stop confirmation, localization/goal acceptance and the turn-envelope hash are unchanged. No predefined map or runtime truth is used.

Evidence: `docs/evidence/native-s01-source-baseline-20261104`, `native-s01-hit-encoding-20261104`, `native-s00-2m-hit-encoding-20261104`, `native-s00-3m-hit-encoding-20261104`. Each contains results, command/trajectory CSVs, exact config snapshots, passive source trace, source analysis and per-scan correction CSV. Reproduce analysis with `python3 scripts/analyze_slam_source.py <run-directory>`. Gzip traces are losslessly recompressed after shutdown to reduce repository size.

## Remaining execution problem and checks

Nav2's translation-only SimpleProgressChecker declares failure while a validated pure turn is progressing. Independent 12 s truth windows around failures showed .041 m translation with 1.077 rad yaw, .044 m with .927 rad yaw, and .033 m with .521 rad yaw. Other failure windows had genuinely limited motion. The next controlled S01 test should recognize rotational progress without changing runtime safety timeouts or goal acceptance; the original profile and failed route are preserved for comparison.

RTF was .993–.998; corrected headless matching-process peak RSS was 1843–2137 MiB, mean CPU 471–488% (about 4.7–4.9 logical cores), scan rate 5.00–5.01 Hz, RGB about 5 Hz and map about 1 Hz. RSS/CPU are sampled process-pattern aggregates, not exact host peaks. Raw traces are bounded to 64 MiB uncompressed and did not truncate. One shutdown-only wheel-adapter pybind exception occurred after the S01 result was collected; it is retained in launch.log. All owned simulation processes stopped.

The full offline discoverable suite passed 115 tests; separate Phase 6/7/8 static validation scripts passed. No installed ROS/Gazebo/driver changes or new dependencies were needed. Full build logs and the compiled Karto contract probe are in `docs/evidence/native-slam-source-investigation-20261104`.

The pre-existing user-generated GUI run `gui-new-20261104T111856` also reports PASS, final localization error .00319 m. Its twelve reviewed Gazebo/RViz screenshots and child console are preserved. The observer's final observer.json is absent, so no new observer-wide memory-peak claim is made for that run. Earlier complete GUI resource evidence and accepted baseline S00/observation runs remain intact.

## Reproduce corrected S00

```bash
cd ~/AutonomousNavigationCar
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/commission_lidar_s00.py \
  docs/evidence/native-turn-production-matrix-20261103/envelope.json \
  "docs/evidence/s00-corrected-$(date +%Y%m%dT%H%M%S)" \
  --distance-m 3 --timeout-s 180 --domain-id 60 --trace-slam \
  --slam-hits-only \
  --slam-config simulation/ros2_ws/src/anc_lidar/config/slam_toolbox_hit_endpoints.yaml
```

Use `--distance-m 2` for the accepted two-metre correction profile. Existing graphical S00 and baseline observation commands remain in [the preceding report](native-lidar-followup-20261104.md); they use the original default configuration.
