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
