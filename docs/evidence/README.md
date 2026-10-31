# Phase 1 and Phase 2 evidence index

Evidence below was captured from the E:-hosted Ubuntu-24.04 WSL distro. Phase 1 navigation used the packaged REFERENCE sensor profile. Phase 2 entity-state data appears only in the motion evaluator and is not an input to the robot controller or Nav2.

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

The final motion test recorded no contact messages even though the contact bridge publisher and Gazebo topic were present. The wall stop is validated by the withheld evaluator pose matching the geometry-derived contact plane and near-zero forward velocity. Do not interpret the missing contact messages as evidence that there was no collision.

No Phase 2 screenshot or video was produced because these tests were intentionally headless. Gazebo was stopped after the checks.
