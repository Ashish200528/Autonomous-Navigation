# Open issues and next phase

## Resolved in Phase 0/1

- WSL2 Ubuntu 24.04 and the ROS 2 Jazzy/Gazebo Harmonic toolchain were verified.
- E: storage, WSL memory/swap, WSLg, and GPU visibility were checked.
- ROS talker/listener, headless Gazebo, RViz2, packaged Nav2 sample, and a REFERENCE map-frame goal were verified.
- The independent Phase 2 robot packages build and the first motion/wall-stop tests pass within their stated limits.

## P0 before hardware-faithful claims or transfer

- Measure chassis footprint, height, mass, wheel radius, wheel separation, and center of mass. Current simulation values are placeholders.
- Measure camera mount pose and calibration, including resolution, FOV, intrinsics, distortion, and image timing.
- Define IMU noise/bias and odometry covariance from real measurements or clearly labeled assumptions.
- Define hardware watchdog and independent electrical emergency stop before any motor integration.

## P1 for Phase 3 and 4

- Add RGB image and CameraInfo, IMU, and timestamped ROS bridges; verify rates and frame IDs.
- Verify full wheel-link TF and joint-state visualization; the current TF test covers odom to base_link.
- Diagnose why the configured Gazebo contact sensor advertised a topic but emitted zero messages.
- Characterize odometry during wall contact and wheel slip; the final trial's odometry advanced while world pose was stopped.
- Add noise/covariance and compare odometry/state estimates to evaluator-only truth offline.
- Test whether Gazebo GUI and RViz2 can run together within the WSL memory cap; the current automated tests are headless.

## P2 for later navigation experiments

- Add unknown-pose acquisition separately from the known-initial-pose baseline.
- Create seeded open-field, obstacle, constriction, occlusion, dynamic-obstacle, safe-stop, and parking scenarios.
- Implement class-agnostic RGB FREE/OCCUPIED/UNKNOWN perception; never treat UNKNOWN as CLEAR.
- Implement the realistic sensor-constrained local costmap and replanning path.
- Define parking tolerance and dwell using measured footprint, then test stationary pose error.
- Plan UNO Q adapters only after simulation interfaces and regression checks are stable.
