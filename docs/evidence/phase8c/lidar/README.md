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
