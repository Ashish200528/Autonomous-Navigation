# Open issues and next phase

## Resolved through Phase 3

- WSL2 Ubuntu 24.04 and the ROS 2 Jazzy/Gazebo Harmonic toolchain were verified.
- E: storage, WSL memory/swap, WSLg, and GPU visibility were checked.
- ROS talker/listener, headless Gazebo, RViz2, packaged Nav2 sample, and a REFERENCE map-frame goal were verified.
- The custom differential-drive model builds and passes forward, reverse, rotation, wall-stop, odometry, and TF checks within the limits recorded in the phase report.
- Phase 3 RGB camera and CameraInfo, IMU, clock, odometry, wheel joint states, and the full robot TF chain publish at verified rates.
- The Phase 2 contact gap was diagnosed. Gazebo's Contact system was loaded after UserCommands, so sensors on the runtime-spawned robot were not registered. Loading UserCommands before Contact and bridging the actual scoped contact topic restored wall-contact telemetry.
- RViz2 displayed the camera image and robot frames while headless Gazebo ran.

## P0 before hardware-faithful claims or transfer

- Measure chassis footprint, height, mass, wheel radius, wheel separation, and center of mass. Current simulation values are placeholders.
- Measure camera mount pose and calibration, including resolution, FOV, intrinsics, distortion, noise, and image timing.
- Measure IMU noise/bias and derive estimator covariance from hardware data.
- Define hardware watchdog and independent electrical emergency stop before any motor integration.

## P1 before navigation/perception experiments

- Reduce wheel-odometry divergence under wall stall/slip; Phase 3 measured +0.46115 m at the wall. Do not replace odometry with simulator pose.
- Investigate and model rotation error; the Phase 2 trial differed by about 0.10057 rad.
- Validate sensor behavior over multiple seeds, longer runs, and camera viewpoints.
- Measure or tune contact report semantics before treating repeated contact points as collision episodes.
- Test Gazebo's GUI and RViz2 running together; Phase 3 used headless Gazebo with RViz2.
- Decide whether to add calibrated camera noise/distortion only after choosing hardware and measuring it.

## P2 for later navigation experiments

- Add unknown-pose acquisition separately from the known-initial-pose baseline.
- Create seeded open-field, obstacle, constriction, occlusion, dynamic-obstacle, safe-stop, and parking scenarios.
- Implement class-agnostic RGB FREE/OCCUPIED/UNKNOWN perception; never treat UNKNOWN as CLEAR.
- Implement a realistic sensor-constrained local costmap and replanning path.
- Define parking tolerance and dwell using measured footprint, then test stationary pose error.
- Plan UNO Q adapters only after simulation interfaces and regression checks are stable.

Phase 4 and later work remain out of scope until Phase 3 has been reviewed.
