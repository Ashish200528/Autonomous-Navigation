# Phase 1 through Phase 8C evidence index

Evidence below was captured from the E:-hosted Ubuntu-24.04 WSL distro. Phase 1 navigation used the packaged REFERENCE sensor profile. Phase 2/3/4 entity-state data appears only in test evaluators and is not an input to localization, navigation, or perception.

## Phase 1 environment checks

- post-install-verification.log: package versions, memory, disk, GPU, and package audit after stack setup.
- ros-talker.log, ros-listener.log, and ros-talker-listener-result.txt: ROS demo message exchange.
- gazebo-headless.log: Gazebo Harmonic empty world initialized and advertised services.
- rviz2.log: RViz2 WSLg startup and OpenGL output. The screenshot was inspected in the setup conversation but is not stored as an image.
- nav2-headless.log, nav2-diagnostic-launch.log, and nav2-diagnostic-graph.log: packaged TurtleBot3/Nav2 launch and live topic smoke checks.

## Phase 1 reference goal

- nav2-reference-trial-preflight.log and nav2-reference-trial-preflight2.log: preflight graph, lifecycle, action, and topic checks.
- nav2-reference-trial-launch.log and nav2-reference-trial-retry-launch.log: Nav2 launch diagnostics, including the initial-pose timing issue and the corrected run.
- nav2-reference-trial-after-initialpose.log and nav2-reference-trial-postflight.log: AMCL pose, lifecycle/costmap state, and final pose snapshots.
- nav2-reference-trial-map-check.json and nav2-reference-trial-map-check.log: known-free corridor sampling.
- nav2-reference-trial-goal.log: full NavigateToPose feedback and SUCCEEDED result.
- nav2-reference-trial-summary.json and nav2-reference-trial-metrics.log: checks, final pose error, command counts, and partial-recording caveat.
- nav2-reference-trial-telemetry.csv: recorded /clock, /odom, /cmd_vel, and /cmd_vel_nav samples.
- nav2-reference-trial-bag/: MCAP capture. It ends before the last action feedback and is not used alone to claim completion.
- nav2-reference-trial-rosbag.log: recording/reindex output.

The final JSON summary is the compact result. The detailed logs remain the source for individual observations.

## Phase 2 custom robot

- phase2-build-initial.log, phase2-build-final.log, phase2-build-caster-contact-fix.log, and phase2-final-validation.log: colcon output for the three packages and final Xacro/URDF/SDF/YAML checks. The final package build reported 3 packages finished.
- phase2-model-validation.log, phase2-model-validation-final.log, phase2-caster-geometry-fix.log: Xacro, URDF, and world validation from development iterations.
- phase2-corrected-baseline.log, phase2-live-preflight.log, phase2-final-preflight.log: entity state, odometry, resource, service, and topic preflight output.
- phase2-simulation-launch-final.log: final headless motion-test launch.
- phase2-tf-check.log and phase2-simulation-tf-launch.log: publisher graph and successful tf2_echo odom-to-base_link lookup.
- phase2-motion-results.json, phase2-motion-test.log, and phase2-motion-telemetry.csv: final motion result, full command output, and sampled telemetry. All 10 required checks passed.
- phase2-motion-first-attempt.*, phase2-motion-second-attempt.*, phase2-motion-third-attempt.*, and phase2-motion-fourth-attempt.*: preserved failed iterations, including initial caster/contact diagnostics. These files are retained so the model corrections and remaining sensor gap can be reviewed.
- phase2-simulation-launch.log, phase2-simulation-launch-corrected.log, phase2-simulation-launch-caster-contact-attempt.log, phase2-simulation-launch-track-corrected-attempt.log, and phase2-simulation-launch-front-caster-attempt.log: earlier launch logs from development iterations.

The final Phase 2 motion test recorded no contact messages. Phase 3 diagnosed this as a Gazebo system-order issue for a dynamically spawned model and a bridge topic mismatch. The Phase 3 run produced verified wall-contact messages; see the Phase 3 evidence below.

No Phase 2 screenshot or video was produced because these tests were intentionally headless. Gazebo was stopped after the checks.


## Phase 3 sensor simulation and ROS interfaces

- phase3-baseline-build.log: three-package Phase 2 baseline rebuild before Phase 3 changes.
- phase3-sensor-build-initial.log, phase3-sensor-build-fix.log, phase3-sensor-build-final-attempt.log, phase3-sensor-build-attempt2.log, and phase3-sensor-build-final-verified.log: sequential ROS workspace builds; the final verified build completed all three packages after parameter updates.
- phase3-model-validation-initial.log: initial Xacro/URDF/SDF inspection; records the ignored IMU orientation tag that was removed.
- phase3-contact-baseline-launch.log, phase3-simulation-launch-initial.log, phase3-simulation-launch-fix-attempt1.log, phase3-simulation-launch-attempt2.log, and phase3-simulation-launch-attempt3.log: preserved contact/sensor diagnostic launches.
- phase3-simulation-launch-final.log: final fresh-world headless smoke-test launch. phase3-validation-final.log: final Xacro/URDF/SDF, Python, YAML/JSON, bag metadata, resource, and diff checks. phase3-sensor-smoke-final.log and phase3-sensor-smoke-final.json: accepted test output and machine-readable results; all checks passed.
- phase3-sensor-telemetry-final.csv: timestamped camera, CameraInfo, IMU, odometry, joint, clock, and contact telemetry, including the reproducible wall event and odometry/evaluator comparison.
- phase3-camera-frame-final.png: actual Gazebo-rendered RGB camera frame captured before the wall approach. The image contains the red wall and gray ground.
- phase3-sensors-bag-final/: finalized 4.15 s MCAP capture (14.1 MiB) of /clock, RGB image/CameraInfo, sanitized /imu/data, /odom, /tf, and /tf_static. High-rate contact and joint-state data remain in CSV/JSON.
- phase3-rosbag-record-final.log: recorder output for the bounded MCAP capture.
- phase3-rviz-view.jpg, phase3-rviz.log, and phase3-rviz-simulation-launch.log: WSLg screenshot and logs from RViz2 displaying RGB and robot/TF while Gazebo ran headlessly.
- phase3-sensor-smoke-attempt1.* and phase3-sensor-smoke-verification1.*: preserved early test attempts. The first exposed clock QoS, orientation handling, configured odometry rate, contact registration, and pre-contact image selection issues; later fixes are covered by the accepted final run.
- phase3-rosbag-record-short.log records the oversized high-rate capture attempt. Large interrupted/overflow MCAP attempts are kept locally under ignored evidence directories; they are not part of the accepted capture.

The Gazebo contact test counts repeated contact records while the robot remains in contact; they are not unique collision episodes. The final smoke result reports wall-specific contact messages separately from other contacts.

## Phase 4 localization

- phase4-baseline-environment.log, phase4-baseline-build.log, phase4-baseline-launch.log, and phase4-baseline-phase3-smoke.log: baseline environment/build and accepted Phase 3 behavior before Phase 4 edits.
- phase4-build-final.log and phase4-validation-final.log: final four-package build and static validation, including Python/YAML, covariance dimensions/types, Xacro/URDF/SDF, wheel friction, world, and launch arguments.
- phase4-localization-normal-final7.json, phase4-localization-normal-final7.csv, phase4-localization-normal-final7.png, phase4-normal-launch-final7.log, and phase4-normal-test-final7.log: accepted normal trajectory, stationary IMU, dropout/recovery, wall stall, and rectangle-return measurements.
- phase4-localization-slip-final4.json, phase4-localization-slip-final4.csv, phase4-localization-slip-final4.png, phase4-slip-launch-final4.log, and phase4-slip-test-final4.log: accepted low-friction wheel-slip trial.
- phase3-sensor-smoke-phase4-final-model.json, phase3-sensor-telemetry-phase4-final-model.csv, phase3-camera-frame-phase4-final-model.png, and phase4-final-phase3-smoke.log: Phase 3 regression on the corrected Phase 4 model; 20/20 checks passed.
- phase4-tf-publisher-info-final4.log and phase4-ekf-node-info-final4.log: topic-level TF publisher and EKF graph/subscription inspection. Graph logs show anc_ekf_filter and robot_state_publisher on /tf; EKF inputs are /odometry/wheel and /imu/data. Jazzy rclpy MessageInfo did not expose publisher_gid per TF message, so no per-edge owner is claimed.
- phase4-normal-launch-attempt*.log, phase4-localization-normal-attempt*.json/.csv/.png, phase4-slip-launch-attempt*.log, phase4-localization-slip-attempt*.json/.csv/.png, and earlier final*.log/JSON files: preserved iterations, including an EKF YAML numeric-type startup failure, initial wall assertion mismatch, subscriber-count/GID instrumentation corrections, and friction conversion diagnostics. The final7 normal and final4 slip artifacts above are authoritative. phase4-normal-test-final6.log preserves a failed summary-formatting attempt; no result JSON was written by that attempt.

Only the Phase 4 evaluator calls /gzserver/get_entity_state. The EKF and wheel adapter consume only /odometry/wheel and /imu/data. Ground truth is retained as evaluator evidence, not published to ROS. Test JSON source_commit records baseline Git HEAD at measurement time; Phase 4 changes were uncommitted working-tree content then.

## Phase 5 reproducible worlds and scenario harness

- phase5-baseline-build.log, phase5-baseline-launch.log, phase5-baseline-phase3-test.log, phase5-baseline-phase4-launch.log, and phase5-baseline-phase4-test.log: pre-change rebuild and fresh Phase 3/Phase 4 baseline checks. The Phase 3 JSON/CSV/camera frame and Phase 4 JSON/CSV/plot are also retained at the evidence root.
- phase5-build.log: four-package workspace build after launch parameterization.
- phase5/static-validation.json and .log: 47 checks for registry IDs, geometry, start/goal clearance, corridors, routes/no-route, same-seed exact layout/SDF replay, different valid seeded layouts, and all six gz sdf -k checks.
- phase5/phase5-summary.json: phase decision and cross-trial seed/hash/shutdown comparison, including the default-world regression.
- phase5/runs/s02-seed101-repeat1/ and phase5/runs/s02-seed101-repeat2/: independent complete S02 trials with identical layout/world hashes; each contains world.sdf, manifest.json, layout.png, launch.log, and result.json.
- phase5/runs/s04-seed202-known-start/: nonzero position/yaw known-start, spawn truth, map-to-odom, composed map-to-base TF, sensors, and shutdown.
- phase5/runs/s09-seed101-blocked-route/: blocked geometry and expected no-route validation with live sensor/TF checks.
- phase5-run-s02-seed101-repeat1-summary.log, phase5-run-s02-seed101-repeat2-summary.log, phase5-run-s04-seed202-summary.log, and phase5-run-s09-seed101-summary.log: concise command output for each runner.
- Canonical worlds: simulation/scenarios/worlds/s00_empty_open_arena.sdf, s01_single_box_detour.sdf, s02_random_simple_objects.sdf, s03_constricted_occlusion.sdf, s04_seeded_known_start.sdf, and s09_no_route.sdf.
- phase5/default-world-regression/: controlled launch without overrides; phase2_wall, zero spawn/map-to-odom, sensor startup, evaluator truth agreement, and clean shutdown passed. The earlier coreutils-timeout diagnostic is preserved as phase5/default-world-timeout-diagnostic.log because its second interrupt caused forced escalation; it is not counted as an accepted run.
- Every accepted live run captured evaluator truth in its local result.json; the runner did not publish truth or entity layouts to ROS, command the robot, or attempt navigation. Each accepted process group exited cleanly after SIGINT.


## Phase 6 reference Nav2 navigation

- phase6/build-final.log: final colcon build of the five packages up to anc_navigation.
- phase6/static-validation.log: Python syntax and 19 Phase 6 static acceptance checks.
- phase6/xacro-camera-toggle.log: XML-based Xacro check for camera-off/on modes and IMU preservation.
- phase6/phase6-summary.json and phase6/README.md: accepted scenario table, aggregate measurements, evidence paths, and limits.
- phase6/runs/s00-reference-final-stop-check/: straight open-arena NavigateToPose gate.
- phase6/runs/s01-reference-final-stop-check/: box detour.
- phase6/runs/s02-reference-final-stop-check/: seeded simple-object obstacle course.
- phase6/runs/s03-reference-final-stop-check/: constricted route and final clean-shutdown verification.
- phase6/runs/s04-reference-final-stop-check/: randomized known-start trial.
- phase6/runs/s09-reference-seed101-pose-odom-stop/: unreachable static goal, bounded Nav2 abort, settled pose/odometry/zero-command check.
- Each accepted run contains result.json, world/map files, truth trajectory, logs, costmaps/trace images where available, and an MCAP rosbag. Earlier failed diagnostic attempts are retained under the same runs/ directory and are not counted as accepted results.

The ORACLE_REFERENCE map is generated from the exact scenario geometry and the known spawn pose is supplied to the localization baseline. This verifies custom-car Nav2 planning against a privileged reference map; it does not validate RGB perception, unknown-map navigation, unknown-start localization, parking, or sim-to-real behavior. S03's minimum conservative clearance was about 0.0155 m. S09's Gazebo truth twist disagreed with stable pose and near-zero odometry while command topics were zero; see its result.json. No GUI screenshot was captured because these trials were headless; the run-specific map/layout and navigation trace PNGs are saved.

## Phase 7 RGB-only perception

- phase7/build-full.log: final six-package ROS workspace build.
- phase7/static-validation.log: 13/13 classifier, mask, ambiguity, calibration, and fail-closed checks.
- phase7/fault-injection.log and phase7/fault-injection.json: live missing-RGB, fresh-RGB, and stale-sim-timestamp behavior; invalid cases publish all UNKNOWN with zero unknown confidence, and the stale case retains the source image timestamp.
- phase7/floor-reference-calibration.json: fixed Lab floor reference, ROI, calibration scope, support requirements, and valid 0.50-0.60 m band.
- phase7/accepted-phase7-final-verified-console.log and phase7/dataset/accepted-phase7-final-verified/dataset-results.json: authoritative six-case, 12-view dataset run, quantitative metrics, topic contracts, frame timing, memory samples, and process outcomes.
- phase7/dataset/accepted-phase7-final-verified/{s00-seed101-rgb-only,s01-seed101-rgb-only,s02-seed101-rgb-only,s03-seed101-rgb-only,s04-seed202-rgb-only,p7c1-seed101-rgb-only}/: generated world/manifest, actual RGB, predicted mask, confidence/overlay, offline evaluator truth, and per-view metrics.
- phase7/baseline/: accepted Phase 3 camera/sensor and Phase 6 reference-navigation baseline logs; associated Phase 3 frame/telemetry and the isolated Phase 6 S00 result are linked in the Phase 7 report.
- phase7/diagnostic-findings.md: concise record of issues found and corrected during calibration and runner iteration. Other exploratory console logs remain local and are not presented as accepted evidence.
- phase7-perception-architecture.md, scripts/phase7_ground_truth.py, scripts/phase7_rgb_dataset.py, tests/phase7_perception_validation.py, and tests/phase7_sensor_fault_test.py: architecture, evaluator, dataset runner, and reproducible validations.

The aggregate result has zero false-FREE pixels among 465,242 evaluator-known occupied pixels, 88.01% useful floor coverage, 26.67% occupied recall, and 80.98% UNKNOWN coverage. P7C1 returned all UNKNOWN because calibrated floor support was not observable. This is a limited conservative perception result, not a navigation or collision-avoidance result. See docs/PHASE_REPORT.md for the full metric table and scope limits.


## Phase 8 RGB-only local traversability and stop-only baseline

- phase8/phase8-summary.json and phase8/README.md: nine accepted stop-only runs, safety and resource aggregates, separate Phase 6 comparison, and evidence index. Autonomous motion is explicitly blocked.
- phase8/phase8-feasibility.json: camera projection, connected-free-space, reaction-distance, and unmeasured braking-distance gate.
- phase8/phase8-static-validation.json and phase8-static-validation.log: 7/7 FREE projection, UNKNOWN preservation, confidence, and calibration checks.
- phase8/phase8-build.log: final seven-package ROS workspace build.
- phase8/runs/: bounded S00/S01/S02/S03/S09 live stop runs, including missing camera/localization, command timeout, and S02 seeds 101/202. All 1,052 observed output commands were zero.
- phase8/phase7-fault-injection.json and phase7-fault-console.log: missing/stale RGB fail-closed behavior and stale-to-fresh recovery.
- phase6/runs/phase8-final-reference-s00-20261102/: isolated ORACLE_REFERENCE regression recorded during Phase 8; it is not RGB-only navigation evidence.


## Phase 8B camera geometry and stopping feasibility

See [phase8b/README.md](phase8b/README.md) for the evidence index and [phase8b/phase8b-report.md](phase8b/phase8b-report.md) for candidate metrics and the BLOCKED motion-gate decision. The isolated stop-test details and calculation are recorded in [phase8b/motion-gate.json](phase8b/motion-gate.json). The Phase 8 safe-stop supervisor stayed active; no autonomous RGB navigation was run.

## Phase 8C Branch 1 RGB + 2D LiDAR

See [phase8c/lidar/README.md](phase8c/lidar/README.md) for S00 sensor/map measurements, graph audits, safety gate, Stage B/C faults, regressions and interpretation. Stage D obstacle avoidance and Nav2 goals were not run; angular speed remains zero.

Phase 8C includes the final build, 24 unit tests, Phase 5/6/7/8 regressions, fresh Phase 3/4 records, Phase 8 zero-command run, Phase 8B stopping repeat, and sensor/fault/motion JSON and logs. Development failures and final retries are both retained; only named final results count as accepted.


## Phase 8C remaining LiDAR development

Stage report/status matrix: docs/phase8c-stage-results.md. Hardware plan: docs/phase8c-hardware-transfer.md. Operator guide: docs/phase8c-lidar-run-guide.md.

- Stage 1 calibration and no-motion clearance probes: phase8c/lidar/stage1-rotation-trials-retry.json and stage1-s00-near-wall-gate.json / stage1-s01-near-box-gate.json / stage1-s03-constriction-gate.json.
- Stage 2 stationary map: phase8c/lidar/stage2-s00-stationary-slam-tuned.json and stage2-s00-stationary-slam-map.pgm/.yaml/.png.
- Stage 5 proposal tests: phase8c/lidar/stage5-parking-controller-tests.log.
- Stage 6 aggregate JSON/CSV and plots: phase8c/lidar/stage6-comparative-summary.json, stage6-comparison.csv, stage6-comparison.png and stage6-resource-observations.png.
- Stage 8 headless report: live-demo-2026-11-02/phase8c-live-demo-report.md. MP4 remains local under existing ignore rule.

- Final verification: phase8c/lidar/final-common-tests-2026-11-02.log, final-lidar-model-tests-2026-11-02.log, final-parking-tests-2026-11-02.log and final-colcon-build-retry-2026-11-02.log.
