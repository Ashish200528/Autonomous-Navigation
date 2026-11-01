# Phase 1 through Phase 3 evidence index

Evidence below was captured from the E:-hosted Ubuntu-24.04 WSL distro. Phase 1 navigation used the packaged REFERENCE sensor profile. Phase 2/3 entity-state data appears only in test evaluators and is not an input to navigation or perception.

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
