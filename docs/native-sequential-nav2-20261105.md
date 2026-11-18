# Sequential Nav2 commissioning — 2026-11-05

The scan-matching endpoint correction remains enabled only by explicit options. The accepted baseline GUI S00, original two-metre/observation workflows and Phase 6 reference configuration are preserved. This work adds an optional Nav2 controller for the already validated pure forward/turn primitives; it does not grant actuator authority or validate reverse/combined motion.

## Controller and independent qualification

`anc_nav2_sequential_controller::SequentialController` implements Nav2's controller interface. It returns proposals through the existing controller → smoother → collision monitor → authoritative guard chain. It never publishes `/cmd_vel`. Forward proposals are bounded to .08 m/s, pure turns to .05 rad/s. Primitive changes require 1.25 s of zero proposals plus .50 s of measured low odometry speed. Invalid/stale pose, invalid path, missing transforms, stale costmap, occupied/unknown predicted footprints fail closed. The original guard, swept-disk unknown checks, freshness rules, reverse/mixed denials and production envelope are unchanged.

The optional profile retains .15 m/.08 rad Nav2 goal tolerances, .10 m translation progress and the 12 s watchdog. It uses PoseProgressChecker with .40 rad angular progress: .05 rad/s requires 8 s plus bounded slew/settling to demonstrate that progress. This changes progress classification, not physical safety thresholds. The .04 rad turn stop band and .12 rad drive-heading band were checked with actual left/right goal turns before S01.

Before changing scan clearing, direct S00 2 m and 3 m passed with .01934/.02339 m final localization error and .11762/.11166 m goal error. Independent +pi/2 and -pi/2 goal tests passed. Normal-stop analysis retains TURN_STOP_V2 unchanged (.75 s physical stop, .05 rad stopping angle including uncertainty, .50 s confirmation, 1.40 s confirmation deadline, .15 s maximum relevant sample gap):

| Trial | Guard zero → physical stop upper | Guard-zero angle upper | Upstream-observed stop upper | Upstream-observed angle upper | Upstream-observed confirmation |
|---|---:|---:|---:|---:|---:|
| Left stop | .02838 s | .00233 rad | .28242 s | .01122 rad | .77651 s |
| Right stop | .06481 s | .00402 rad | .41984 s | .01641 rad | .89847 s |
| Clearing-profile left stop | .04535 s | .00288 rad | .34237 s | .01352 rad | .84157 s |

“Upstream observed” starts at the evaluator's raw command subscription receipt, not the publisher's source time. The original guard decision timestamp measures suppression separately from physical deceleration. Clock/pose receipt uncertainty is included, stationary intervals are fully retained and post-confirmation rebound is checked. These are additional normal-stop measurements, not a replacement or relabelling of the existing thirteen production fault/clearance categories. Gazebo truth enters only the evaluator. Synthetic stop-analysis fixtures test rejection of rebound, missing clock evidence and recovery velocities above this profile.

## Instrumented Nav2 clearing defect

The first new-controller S01 attempt aborted on its own conservative footprint check, despite accurate localization (.00218 m at the failed first observation). A bounded diagnostic captured cost **255 / unknown** at prediction .60 s, with robot pose approximately (-.00017, 0) in odom and a correctly centred local grid. The runtime guard independently remained authoritative. Nav2 recovery requested reverse motion, which the guard denied; no S01 success is claimed for these runs.

Adding the live SLAM map as a rolling local costmap layer did not fix the failure. That optional experimental profile and failed evidence are retained; it is not promoted. The hit-only SLAM scan cannot provide no-return clearing rays in open directions either.

The actual input-contract defect is that `/realistic/scan` normalizes measured no-return beams to exactly 6.0 m. [LaserProjection](https://raw.githubusercontent.com/ros-perception/laser_geometry/ros2/src/laser_geometry.cpp) accepts ranges strictly below `range_max`, so those beams disappear from Nav2's point cloud. [Nav2's supported infinity callback](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_costmap_2d/plugins/obstacle_layer.cpp) instead maps positive infinity to `range_max - .0001` when `inf_is_valid` is true. A compiled test against the installed projection library independently reproduces the exact-maximum exclusion, valid epsilon endpoint and NaN rejection.

The correction is a separate opt-in `/realistic/scan/nav2` stream: actual hits retain their values; normalized no-return beams become +Inf; invalid, missed and out-of-range beams become NaN. Header, beam index, angles and timestamp are preserved. The optional `nav2_sequential_clearing.yaml` changes only the local/global obstacle scan topic and enables `inf_is_valid`. Marking stops at 5.5 m, clearing at 6.0 m; no-return endpoints cannot become marked obstacles. The safety scan, local safety map and SLAM hit-only scan remain separate. Unknown cells remain unknown unless a measured ray clears them; neither the footprint check nor the authoritative guard ignores unknown space.

The clearing-profile S00 forward/+pi/2 goal passed in 49.60 s, with .1580 m true goal error, .05479 m final localization error, .01070 m maximum scan-stamped SLAM error and a confirmed stop. It requested/output no mixed or reverse motion. RTF was .9916. Observer process-tree peak RSS was 2091 MiB and minimum host available memory 1646 MiB; no memory abort occurred. This is a headless autonomous result, not GUI evidence.

## Evidence and limits

Evidence directories begin `docs/evidence/native-sequential-`. `navigation-result.json`, per-observation results, exact configuration snapshots, command/trajectory CSVs, sensor maps, live parameter audits, source analysis and bounded traces distinguish actions from mission outcomes. Full repeated range traces are archived under `~/.local/state/anc-evidence-raw/20261105-sequential/`; each bounded run has a hash/path manifest. The compact original costmap diagnostic retains its full trace. The preliminary 100 s timeout request for a short S00 turn was rejected before launch by the existing 60 s bound; its observer failure is retained separately from the accepted retry.

One successful right-turn run required Gazebo shutdown escalation through SIGTERM to SIGKILL after goal/stop evidence collection. Its launch log is preserved, and owned simulation processes were subsequently absent. This is a cleanup problem, not an accepted shutdown improvement.

Ten packages build sequentially, including the optional new controller; the original nine packages remain intact. Offline Python regression (210 pytest cases; overlapping 135 unittest cases), C++ tests (10 colcon result entries across two executables), separate Phase 6/7/8 checks and rosdep results are saved in `native-sequential-controller-20261104/` (the series name predates these Nov 5 runs). The first C++ fixture build failed because two abstract Layer methods were omitted; the corrected build and failure log are retained. No ROS/Gazebo/NVIDIA reinstall, administrator-security change or runtime safety-gate modification was made.

## First complete S01 mission

`native-sequential-clearing-s01-20261105` passed all three observation actions and the original **(+3, 0, 0)** destination from the original (-3, 0, 0) spawn. It used the live hit-only SLAM scan, corrected Nav2 clearing scan, sensor-built maps and the unchanged safety guard. Cumulative truth path was **7.0253 m**, mission time **249.78 s** within the unchanged 300 s bound. Original-goal true XY error was **.14907 m**, heading error **.05190 rad**, final SLAM localization error **.07319 m** and heading error **.01632 rad**. Minimum conservative evaluator clearance was **.57407 m**. Stop was confirmed; non-ground contacts and mixed guard outputs were zero, and the single authoritative publisher audit passed.

Maximum aligned scan-stamped SLAM error was .05492 m. Live input-contract auditing paired 1220 scans: 421364 hits retained, 14835 measured no-return clearing beams, 3001 invalid beams kept NaN and zero value/metadata mismatches.

All three observation actions passed in 38.11/51.11/59.61 s with final localization errors .01987/.01151/.01537 m. `/cmd_vel_nav_raw` and final `/cmd_vel` remained within .08 m/s and .05 rad/s. Sampled guard reasons included 27 confirmed-primitive-stop waits, four measured-gate clamps and 202 stale-command samples; these interventions remained active. Counts are sampled observations, not distinct physical incidents.

The final action's sampled matching-process peak RSS was 1977 MiB, mean CPU 483% (about 4.8 logical cores); whole-observer process-tree peak RSS was 2200 MiB and minimum host available RAM 1598 MiB. RTF was .99785 and scan rate 5.004 Hz. This first complete headless mission establishes one bounded simulated S01 success, not reliability across scenarios or hardware. A graphical repeat and the longer S00 regressions with the clearing profile remain separate required checks. After shutdown, available RAM was about 2.0 GiB, below the unchanged 2.5 GiB GUI preflight; unrelated applications are not terminated. Parking, exploration, mixed/reverse movement and hardware autonomy remain unvalidated.

## Reproduce bounded visual S00

```bash
cd ~/AutonomousNavigationCar
source /opt/ros/jazzy/setup.bash
source simulation/ros2_ws/install/setup.bash
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python3 scripts/observe_gui_commission.py \
  "docs/evidence/sequential-s00-visual-$(date +%Y%m%dT%H%M%S)" --domain-id 45 \
  --runner-args --distance-m 2 --timeout-s 180 --trace-slam --slam-hits-only \
  --nav2-clearing-scan \
  --slam-config simulation/ros2_ws/src/anc_lidar/config/slam_toolbox_hit_endpoints.yaml \
  --nav2-config simulation/ros2_ws/src/anc_lidar/config/nav2_sequential_clearing.yaml
```

This is a reproduction command, not a claim that this new profile has passed a graphical run. The already accepted default graphical S00 remains reproducible using the observer without `--runner-args`; its command and evidence are in the earlier reports. Add `--headless` before `--runner-args` for resource-observed headless tests. Never run two simulations concurrently on this laptop.
