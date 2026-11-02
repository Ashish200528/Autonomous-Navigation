# Autonomous Navigation Car roadmap status

Updated: 2026-11-02

The supplied master roadmap remains reference material. This status records the implemented phase gates.

| Phase | Status | Scope |
|---|---|---|
| 0 | PASS | Repository and simulation planning baseline |
| 1 | PASS / PASS WITH LIMITS | ROS 2, Gazebo, RViz and reference goal setup |
| 2 | PASS WITH LIMITS | Custom differential-drive robot |
| 3 | PASS WITH LIMITS | Camera, IMU, odometry, TF and contact interfaces |
| 4 | PASS WITH LIMITS | Wheel/IMU EKF and known-start localization |
| 5 | PASS WITH LIMITS | Deterministic worlds and scenario harness |
| 6 | PASS WITH LIMITS | ORACLE_REFERENCE Nav2 baseline |
| 7 | PASS WITH LIMITS | RGB FREE/OCCUPIED/UNKNOWN perception |
| 8 | PASS WITH LIMITS: stop-only baseline | Local RGB traversability and safe-stop supervisor |
| 8B | COMPLETE WITH LIMITS | Camera candidate sweep and isolated simulated stopping-distance measurements |
| Phase 8 RGB-only motion gate | BLOCKED | Best connected full-footprint corridor 0.480 m vs 0.656429 m required; candidate occupied recall is inadequate |
| 8C Branch 1: RGB + 2D LiDAR | PASS WITH LIMITS | Sensor-derived local map, optional RGB fusion, fail-closed safety gate, bounded straight motion and fault stops |
| 8C Stage D: avoidance / goal navigation | BLOCKED / NOT RUN | Angular authority is 0 rad/s; SLAM/Nav2 global navigation was not established |

Phase 8 remains an independent RGB-only stop-only profile with its motion gate blocked. Phase 8C uses a separate LiDAR profile and passed a limited forward-only gate in S00; its fused map did not gain cells from RGB in that clear corridor. The short movement and fault-stop evidence does not establish obstacle avoidance or goal reaching. See [Phase 8C evidence](evidence/phase8c/lidar/README.md).

The 8C speed gate used the Phase 8B simulator stop-distance table, Phase 4 slip-derived localization allowance, current scan/map health, footprint clearance and safety margin. A 0.50 m clearance probe stayed stopped below the 0.583 m minimum tested requirement. No hardware safety claim is made.

The Phase 6 ORACLE_REFERENCE launch remains independent. ToF/RGB-only branches, parking, hardware integration and Phase 9 are outside this branch.


## Remaining LiDAR assignment update (2026-11-02)

| Stage | Status | Result |
|---|---|---|
| 8C Stage 1 turning | PASS WITH LIMITS | Six calibration pulses and swept-clearance denials; production yaw disabled. |
| 8C Stage 2 SLAM | PASS WITH LIMITS | Stationary map/map-to-odom only; moving localization and drift reduction unproven. |
| 8C Stage 3 realistic goals | BLOCKED / NOT RUN | Nav2 remains off pending production yaw and moving localization evidence. |
| 8C Stage 4 avoidance/replanning | BLOCKED / NOT RUN | No obstacle traversal or replanning run. |
| 8C Stage 5 parking | BLOCKED live | 7 unit tests pass for a non-authoritative proposal controller. |
| 8C Stage 6 evaluation | PARTIAL WITH LIMITS | Prior profile evidence aggregated; not a controlled head-to-head benchmark. |
| 8C Stage 7 hardware | PARTIAL DESIGN ONLY | UNO Q compatibility and physical safety require validation. |
| 8C Stage 8 demo | PASS WITH LIMITS | Headless video and run guide; no Nav2 path or desktop capture. |

Full evidence: docs/phase8c-stage-results.md. Older Stage D text predates calibration pulses; production yaw and downstream navigation remain blocked.


## 2026-11-02 readiness and moving-SLAM follow-up

The single-run bounded forward SLAM probe now measures in-motion scan, SLAM map, SLAM pose, map-to-odom TF, truth-relative pose error, and non-ground contacts. Its trial result cannot PASS without those observations and limits. The Nav2 launcher now validates JSON result contents: production turning must include 13 measured cases, moving SLAM must include three unique passing trials, and the guarded-chain evidence must show a successful S00 goal with active Nav2 lifecycle nodes, live costmap/plan updates, safe clearance, and the sole safety-guard cmd_vel publisher.

The prerequisite flags and evidence references remain unchanged; all three prerequisites are still false/unset. A post-build host-memory probe measured 926 MiB available on Windows and 3134 MiB in WSL. Windows remains below the launcher's 2 GiB headless requirement, so no live simulation was started. Offline build/tests passed; actual moving SLAM, production turning, Nav2 goals, S01 avoidance, replanning and parking remain BLOCKED or NOT TESTED. See docs/phase8c-stage9-report.md and docs/evidence/phase8c/lidar/moving-slam-readiness-offline-validation-final-20261102.log.
