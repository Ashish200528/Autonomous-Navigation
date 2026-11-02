# Phase 8C Branch 1 evidence index

Date: 2026-11-02. Workspace: /home/ashish/AutonomousNavigationCar. Branch: feature/phase8c-lidar.
World: S00 empty arena, except where the static-clearance probe world is named.

## Sensor and map
- final-post-motion-live.json/.log: 360-beam /realistic/scan, ±π, 0.12–6.0 m, 4.94 Hz, 358/360 usable. LiDAR map: 4,206 FREE, 267 OCCUPIED, 27,927 UNKNOWN at 0.10 m; RGB 617 FREE, 5,008 UNKNOWN, zero metric OCCUPIED; fused counts matched LiDAR-only.
- final-post-motion-camera-pair.json/.log: 320×240 CameraInfo pairing under C2 pose, no input failures.
- final-*graph-audit.log: safety, mapper, fuser, RGB inputs and sole /cmd_vel publisher; runtime has no oracle-map or Gazebo-truth input.
- lidar-build-infinity-camera-c2.log: final eight-package build.
- phase8c-common-tests-final.log / phase8c-lidar-model-tests-final.log / camera-sync-c2-unit-test.log: 8/8, 13/13 and 3/3 unit tests.
- final-resource-snapshot.txt: WSL resource and process sample; combined profile showed high CPU and an EKF update warning.

## Stage B stationary safety
- final-stage-b-invalid-scan-retry.json/.log: malformed scan becomes UNKNOWN and output is zero; valid scan recovery.
- final-stage-b-scan-process-loss.json, final-stage-b-rgb-process-loss.json, final-stage-b-localization-loss.json: stale inputs stop and recover.
- final-stage-b-clearance-probe.json/.log and s00_static_clearance_probe.sdf: 0.50 m corridor denied; 0.582701 m required at 0.02 m/s.
- Earlier diagnostic attempts remain beside final retries.

## Stage C bounded motion
- final-stage-c-command-timeout.json: 0.212 m total displacement, 0 after authoritative zero.
- final-stage-c-lidar-loss-motion.json: 0.046981 m total, 0.000082 after zero.
- final-stage-c-rgb-loss-motion.json: 0.056990 m total, 0 after zero.
- final-stage-c-localization-loss-motion.json: 0.049390 m total, 0 after zero.
- Trial requests were 0.05 m/s; safety cap allowed max 0.18 m/s forward and 0 rad/s angular. These are straight/fault-stop checks, not avoidance.

## Cross-phase regressions
- Phase 3: final-phase3-regression-console.log and ../../phase3-sensor-smoke-phase8c-branch1-regression.json, ../../phase3-camera-frame-phase8c-branch1-regression.png, ../../phase3-sensor-telemetry-phase8c-branch1-regression.csv; 20/20 pass.
- Phase 4: final-phase4-regression-console.log (one threshold failure) and final-phase4-regression-retry-console.log (pass with limits); JSON results ../../phase8c-branch1-regression-normal.json and ../../phase8c-branch1-regression-normal-retry.json.
- Phase 5 static: final-regression-phase5-static.log, 47/47.
- Phase 6 profile: final-regression-phase6-static.log, 19/19. Earlier live Nav2 results remain in ../phase6; Nav2 was not rerun here.
- Phase 7 static: final-regression-phase7-static.log, 13/13.
- Phase 8 static: final-regression-phase8-static.log, 7/7; live phase8-profile-regression/phase8c-branch1-stop-only-regression/ shows 125/125 zero commands, sole Phase 8 publisher, no Nav2 and no displacement.
- Phase 8B: final-phase8b-stop-regression-console.log; one low-friction sample 0.002976 m at 0.1588 m/s. Full 24-trial run remains in ../phase8b.

## Commands and limits
From WSL: cd /home/ashish/AutonomousNavigationCar; source scripts/source_ros_jazzy.sh; run the common, lidar-model and camera-sync Phase 8C test scripts. Live evaluators are phase8c_live_probe.py, phase8c_invalid_scan_test.py, phase8c_stationary_loss_test.py and phase8c_motion_trial.py; captured command output is in per-stage logs. Truth queries are evaluator-only.
Evidence supports a planar sensor, one-scan map, optional RGB fusion, forward gate and guarded straight movement. It does not establish persistent mapping, reliable general obstacle recall, scan-matching drift correction, yaw safety, obstacle avoidance, Nav2 goals, 3D sensing or hardware safety. No other branch, Phase 9 or UNO Q change is included.


## Remaining LiDAR stages (2026-11-02)

- stage1-rotation-trials-retry.json/.log: six calibration pulses passed. Production yaw stays disabled.
- stage1-rotation-trials.json/.log: original failed evaluation retained.
- stage1-s00-near-wall-gate.json, stage1-s01-near-box-gate.json, stage1-s03-constriction-gate.json: no-motion denial probes; box/constriction also include invalid RGB floor support.
- stage2-s00-stationary-slam-tuned.json/.log: bounded S00 result; launch log has TF-cache and diagnostic QoS warnings.
- stage2-s00-stationary-slam-map.pgm/.yaml and .png: saved from live /map; PNG is a preview, stationary observations only.
- stage5-parking-controller-tests.log: proposal controller, 7/7 tests; no runtime wiring or live parking.
- stage6-comparative-summary.json, stage6-comparison.csv, stage6-comparison.png, stage6-resource-observations.png, stage6-idle-resource-observation.json: profile-separated evidence and plots. Generator is scripts/phase8c_stage6_report.py.
- live-demo-2026-11-02/phase8c-live-demo-report.md and local MP4: headless sensor composite, not desktop capture.
- docs/phase8c-stage-results.md: full stage report.
- docs/phase8c-lidar-run-guide.md: safe sensor and map commands.
- docs/phase8c-hardware-transfer.md: UNO Q architecture and checklist.

No queue-full or EKF missed-rate warning appeared in the 25-second tuned log, but the short observation does not prove earlier issues solved. A TF-cache drop and diagnostic QoS warnings remain. No moving SLAM pose, drift improvement, Nav2 goal, avoidance, replanning or live parking is claimed.

- Final verification logs: final-common-tests-2026-11-02.log (9/9), final-lidar-model-tests-2026-11-02.log (15/15), final-parking-tests-2026-11-02.log (7/7), final-compileall-2026-11-02.log, final-schema-parse-2026-11-02.log, final-colcon-build-retry-2026-11-02.log (8 packages) and final-stage6-report-2026-11-02.log.
- final-colcon-build-shell-error.log preserves the first wrapper failure caused by set -u before colcon; the retry passed.
- stage2-s00-stationary-slam-map.png is a small visual preview of the PGM map.

The first final-build wrapper stopped before colcon because the sourced ROS setup referenced an unset shell variable under set -u; that output is in final-colcon-build-shell-error.log. A separate first colcon log ended while anc_lidar was starting; the complete bounded retry in final-colcon-build-retry-2026-11-02.log passed all 8 packages in 38 seconds.
