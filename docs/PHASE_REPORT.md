# Autonomous Navigation Car simulation phase report

Date: 2026-11-02 (Asia/Calcutta)
Project: /home/ashish/AutonomousNavigationCar on Ubuntu-24.04 at E:/WSL/Ubuntu-24.04
Branch: feature/phase8c-lidar (based on main commit 61c59790ad36194c86e5db86f5a56be170dd6e42)
Scope: Phase 0 through Phase 8C Branch 1 (RGB + 2D LiDAR), with earlier phase limits preserved.

## Outcome

- Phase 0 planning baseline: PASS. The independent repository remains tagged v0.1-phase0. Physical hardware values remain unresolved.
- Phase 1 environment gate: PASS. WSL2 Ubuntu 24.04.5, ROS 2 Jazzy, Gazebo Harmonic, ros_gz, RViz2, Nav2, the packaged TurtleBot3 simulator, and available resources were checked.
- Phase 1 Task A reference navigation: PASS. Nav2 accepted a map-frame goal and returned SUCCEEDED with error code 0. Localization, known-free corridor, nonzero controller output, odometry motion, and final AMCL pose were checked.
- Phase 2 Task B custom robot: PASS WITH LIMITS. The custom differential-drive robot built and spawned headlessly. Forward, reverse, rotation, odometry, base TF, and a physical wall stop passed. The configured contact sensor and ROS bridge produced zero contact messages; odometry also diverged under the wall stall. These limits are recorded below.
- Phase 3 sensor simulation and ROS interfaces: PASS WITH LIMITS. RGB/CameraInfo, sanitized IMU, clock, odometry, joint states, TF, wall contact, RViz, and a bounded bag were verified. Sensor parameters remain provisional.
- Phase 4 odometry, state estimation, and known-start localization: PASS WITH LIMITS. The wheel/gyro EKF reduced motion RMSE and wall-stall x error in the accepted run, but did not correct wheel slip and was worse than raw odometry at the non-closing rectangle endpoint.
- Phase 5 reproducible simulation worlds and scenario harness: PASS WITH LIMITS. Six bounded worlds, deterministic seeded layouts, geometry route/no-route checks, four headless runs, known-start TF, sensor streams, evaluator-only truth, and clean shutdown were verified. No navigation or perception was attempted.
- Phase 6 REFERENCE Nav2 navigation: PASS WITH LIMITS. The selected S00/S01/S02/S03/S04/S09 trials passed using a per-run geometry-derived oracle map, known-start EKF TF, headless Gazebo, collision/clearance/truth evaluation, rosbag evidence, and bounded teardown. This is not RGB or unknown-map navigation.

- Phase 8C Branch 1 RGB + 2D LiDAR: PASS WITH LIMITS for sensors, local mapping, the forward gate and bounded straight/fault-stop tests. Angular motion remains disabled. Stage D avoidance/goal navigation are BLOCKED / NOT RUN. SLAM/Nav2 default off; drift improvement was not demonstrated.

## Scope and integrity

The supplied roadmap and task text governed the phase gates. Gazebo entity state is consumed only by test evaluators. Phase 6 explicitly supplies Nav2 a per-run ORACLE_REFERENCE occupancy grid rasterized from deterministic scenario geometry and a known spawn pose; Gazebo truth pose and obstacle geometry are not published to the navigation graph. This is a reference planning baseline, not RGB perception, unknown-map navigation, or hardware-faithful localization. All custom vehicle dimensions are design placeholders, not measured hardware values.

No package installation, system-wide upgrade, Windows/WSL restart, or Arduino UNO Q application change was made during these validation tasks. All final simulation process groups exited and no processes remained; the final S01 run required launch-managed SIGTERM/SIGKILL escalation for its Gazebo child, as recorded in its result.json. No Phase 2 screenshot was saved; tests were headless.

## Environment and resources

| Item | Verified |
|---|---|
| Windows distribution | Windows 11 |
| WSL | Default version 2; Ubuntu-24.04 is running as WSL 2 |
| Ubuntu | 24.04.5 LTS, Noble; Linux user ashish |
| Project | /home/ashish/AutonomousNavigationCar |
| ROS | ROS_DISTRO=jazzy; ros-jazzy-ros-base 0.11.0 |
| Gazebo | Harmonic / Gazebo Sim 8.15.0 |
| ros_gz | ros-jazzy-ros-gz 1.0.24 |
| Nav2 | 1.3.13 |
| RViz2 | 14.1.24 |
| GPU visible to WSL | NVIDIA GeForce GTX 1650, 4096 MiB, driver 592.82 |
| WSL resources after final test | 3.5 GiB total RAM, about 3.0 GiB available; 1.0 GiB swap free |
| E: storage after tests | About 53 GiB free |

The separate Phase 1 environment report records the earlier installation transactions and WSLg visual check. No new apt packages were needed for Phase 2.

## Phase 1 Task A: REFERENCE Nav2 goal

The packaged TurtleBot3 Waffle/Nav2 sample was run headlessly. It used its static occupancy map and simulated laser scan. The straight corridor was checked against the live /map before sending the goal.

| Measure | Result |
|---|---|
| Initial map pose | (-2.0, -0.5, 0.0 rad) |
| Initial AMCL estimate | (-1.9750, -0.5351) m |
| Goal | (2.0, -0.5, 0.0 rad) in map |
| Corridor occupancy | 161 of 161 samples were known free; 0.025 m sample spacing; map resolution 0.05 m |
| NavigateToPose | SUCCEEDED; error code 0 |
| Action feedback | 919 poses; 9.18 simulated seconds; 3.7296 m map-frame displacement |
| Final AMCL estimate | (1.7728, -0.6526) m |
| Final AMCL XY error | 0.2737 m; within this trial's declared 0.30 m acceptance check |
| Controller output | 78 nonzero /cmd_vel samples and 80 nonzero /cmd_vel_nav samples |
| Recorded topics | /clock, /odom, /amcl_pose, /cmd_vel, and /cmd_vel_nav |

Nav2 lifecycle, map-to-odom TF, and costmap availability were captured after the initial pose was applied. An earlier diagnostic launch without an early initial pose timed out during planner lifecycle startup. The passing run publishes the initial pose early enough for localization to initialize.

The MCAP bag is partial: it ended at simulated time 113.76 s, before the last action feedback at 118.944 s. The full goal result and movement evidence therefore come from the action feedback log and live postflight pose snapshots, not from the bag alone. The 0.30 m check belongs to this reference goal test; it does not replace the provisional 0.20 m parking target or validate parking.

Evidence: docs/evidence/nav2-reference-trial-summary.json, nav2-reference-trial-goal.log, nav2-reference-trial-map-check.json, nav2-reference-trial-postflight.log, nav2-reference-trial-telemetry.csv, and nav2-reference-trial-bag/.

## Phase 2 Task B: custom differential-drive robot

Three ROS 2 packages were added under simulation/ros2_ws/src:

- anc_description: Xacro model and installed link to the root robot-parameter YAML.
- anc_gazebo: a small headless world with ground and a static wall.
- anc_bringup: server launch, robot spawn, robot_state_publisher, and ros_gz topic bridges.

At Phase 2 acceptance the robot had a box chassis, two driven wheels, a passive spherical caster, a camera frame placeholder, Gazebo DiffDrive, wheel odometry, and a base TF chain. The Phase 2 test recorded no contact messages. Phase 3 later diagnosed and corrected the sensor registration and topic bridge; the historical Phase 2 results remain unchanged.

Simulation-only values in config/robot_parameters.yaml remain explicitly labeled placeholders: chassis 0.32 x 0.24 x 0.08 m and 2.0 kg; wheel radius 0.04 m, width 0.025 m, mass 0.15 kg, separation 0.28 m, and axle x -0.04 m; caster x 0.19 m, radius 0.025 m, mass 0.05 kg; speed limits 0.25 m/s and 1.0 rad/s. The conservative footprint rectangle includes the simulated wheel and caster envelope. These are not measurements or sim-to-real specifications.

### Build and model checks

- colcon build --symlink-install --packages-up-to anc_bringup: 3 packages finished.
- python3 -m py_compile scripts/phase2_motion_test.py: passed.
- xacro expansion and check_urdf: parsed successfully; root link and child links were reported.
- gz sdf -k simulation/ros2_ws/src/anc_gazebo/worlds/phase2_test_world.sdf: Valid.
- ROS graph included /clock, /cmd_vel, /contacts, /odom, /tf, and /tf_static; /gzserver/get_entity_state was available for evaluator-only measurements.
- tf2_echo odom base_link resolved through the dynamic and static transforms. /tf had publishers from Gazebo bridge and robot_state_publisher; /tf_static had robot_state_publisher.

### Motion and collision results

The final run passed all 10 required checks in docs/evidence/phase2-motion-results.json:

| Check | Measured result |
|---|---|
| Forward command | Truth delta x +0.30035 m; odometry delta x +0.30030 m |
| Reverse command | Truth delta x -0.29951 m; odometry delta x -0.30030 m |
| In-place rotation | Truth yaw +1.10063 rad; odometry yaw +1.20120 rad |
| Wall approach | Expected robot center x 0.735 m from the configured robot envelope and world wall geometry |
| Wall stop | Evaluator x 0.735 m; position error 0.000 m; final forward velocity about -0.00000056 m/s |
| Maximum sampled x during approach | 0.735025 m; no tunneling past the wall |
| Maximum absolute roll/pitch | 0.00401 rad |
| Odometry messages | 564 |

The chassis stayed upright and the physics constraint held the robot at the calculated wall contact plane while forward commands continued. The ROS contact topic had a publisher, but the test recorded 0 contact messages and 0 wall contact events. Therefore the wall stop is supported by the geometry-derived evaluator pose and near-zero velocity, not by contact-sensor telemetry. A zero sensor count is not reported as a zero-collision count.

During the wall stall, wheel odometry advanced to x=1.40039 m while evaluator pose remained at x=0.735 m. The wheels continued rotating against the wall and odometry followed that motion. Odometry matches forward and reverse travel away from the obstacle, but stall/slip accuracy is not validated. Rotation odometry also differs from evaluator yaw by about 0.10 rad. These results are a basic motion check, not a calibrated dynamics result.

## Phase 3 Task C: sensor simulation and ROS interfaces

**Outcome: PASS WITH LIMITS.** Phase 4 changes were followed by a Phase 3 regression on the corrected model; no Phase 5 work was started.

### RGB camera

The provisional forward-facing camera publishes /camera/image_raw and /camera/camera_info. The accepted smoke run received 320x240 rgb8 frames at 15.15 Hz with frame_id=camera_optical_frame. CameraInfo used plumb_bob, zero distortion, and focal lengths fx=fy=277.128 px, matching the provisional 60 deg horizontal FOV. The saved PNG is a rendered Gazebo image showing the red wall and gray ground; it is not fabricated or a depth channel. The configuration uses no camera noise and no physical calibration.

### IMU

The simulated IMU publishes at 100 Hz in imu_link. During the stationary sample window the mean acceleration was x/y/z = 0.000/0.003/9.803 m/s^2. Provisional SDF noise standard deviations are 0.002 rad/s for angular velocity and 0.02 m/s^2 for linear acceleration; the bridge covariance diagonals were 4e-6 and 4e-4 respectively. These parameters are not hardware measurements.

Gazebo Harmonic's raw IMU also contains a pose-derived orientation. The earlier orientation_enabled tag was ignored by Gazebo and did not suppress that value. The final launch routes raw data to /anc_internal/imu_raw, then the anc_bringup adapter drops the quaternion and publishes /imu/data with a zero quaternion and orientation_covariance[0] = -1, the ROS convention for unavailable orientation. The raw topic is excluded from the accepted rosbag and must not feed navigation/perception.

### Odometry, clock, joints, and TF

/odom used frame odom and child base_footprint, publishing at 29.41 Hz against the configured 30 Hz. /clock advanced monotonically. /joint_states published the left/right wheel joint names at 499.45 Hz. The smoke test verified the edges odom -> base_footprint -> base_link -> camera_link -> camera_optical_frame, base_link -> imu_link, and both wheel links. It found no conflicting child frames; the latest dynamic TF was within 0.002 s of /clock.

### Contact diagnosis and wall test

The Phase 2 contact gap had two causes. First, Gazebo Harmonic's Contact system was loaded after UserCommands, while the robot was created after world startup; loading UserCommands before Contact lets the contact system see the new model. Second, Gazebo published the sensor on its actual scoped topic /world/anc_phase2_test/model/anc_robot/link/base_footprint/sensor/chassis_contact/contact, while the old ROS bridge subscribed to /model/anc_robot/contacts. The world system order and bridge mapping were corrected. This ordering matches the documented Gazebo Sim runtime-spawn behavior described in [Gazebo Sim issue #2223](https://github.com/gazebosim/gz-sim/issues/2223).

The final reproducible forward wall approach passed at the expected robot center x=0.735 m. It recorded 6,374 contact messages and 8,703 contact records during the test window; 2,330 messages/records included phase2_wall. Maximum observed wall-contact depth was approximately 0.000232 m. Counts include repeated contacts while touching; they are not unique collision episodes.

Wheel odometry reached x=1.19615 m while evaluator-only pose remained x=0.735 m, a +0.46115 m divergence during wall stall. The evaluator value was used only by the smoke-test script. The Phase 2 rotation result remains a separate observation: odometry yaw +1.20120 rad versus evaluator yaw +1.10063 rad, about 0.10057 rad difference. These limits are not hidden by substituting simulator pose into /odom.

### Visualization, bag, and evidence

RViz2 displayed the RGB camera and robot/TF frames while headless Gazebo ran. Screenshot: docs/evidence/phase3-rviz-view.jpg. The accepted MCAP bag is 4.15 s and 14.1 MiB with 2,911 messages across camera/CameraInfo, sanitized IMU, odometry, clock, and TF. High-rate contact and joint-state telemetry remains in the CSV/JSON rather than the bag.

The final automated test passed every check in docs/evidence/phase3-sensor-smoke-final.json. The CSV, actual rendered PNG, final build and launch logs, RViz screenshot/log, and bag are listed in docs/evidence/README.md and tests/test_case_registry.yaml. During the headless run, about 2.5 GiB WSL memory remained available; the 1 GiB swap was unused. RViz opened under WSLg. No apt install, system-wide upgrade, WSL restart, or Arduino UNO Q change was made.

### Phase 3 acceptance and limits

- RGB image/CameraInfo, IMU, odometry, clock, TF, wheel joint states, contact telemetry, reproducible smoke test, RViz view, and bounded bag: **PASS**.
- Overall Phase 3: **PASS WITH LIMITS** because camera calibration/noise and IMU assumptions are provisional, raw Gazebo orientation is excluded rather than sensor-disabled, odometry diverges under stall, and repeated contact records are not unique collision episodes.
- The project retains separate REFERENCE and REALISTIC profile labels; Phase 3 does not add perception or navigation behavior.

## Phase 4: odometry, state estimation, and known-start localization

**Outcome: PASS WITH LIMITS.** A new anc_localization package provides the wheel covariance adapter, robot_localization EKF configuration, and known-start launch. The Phase 4 test is headless and uses Gazebo truth only from the evaluator script.

### Estimator and transform ownership

- /odom is preserved as raw Gazebo DiffDrive output. anc_wheel_odometry_covariance republishes its twist on /odometry/wheel with provisional velocity variances: vx=0.0004 m^2/s^2 and vy=0.01 m^2/s^2. Pose covariance is marked unused with a large variance.
- robot_localization runs at 30 Hz with a 0.25 s sensor timeout and two_d_mode. It consumes /odometry/wheel vx/vy plus sanitized /imu/data angular_velocity.z. It does not consume wheel pose, raw Gazebo pose, IMU orientation, or ground truth.
- The sanitized IMU marks orientation unavailable. Known start initializes map -> odom at [0, 0, 0]. The EKF publishes odom -> base_footprint; robot_state_publisher owns robot link transforms. The Gazebo /tf bridge for DiffDrive was removed to prevent a second configured writer of the odom edge.
- The evaluator checks required edges, static map-to-odom identity, and parent conflicts. Jazzy rclpy MessageInfo did not provide publisher_gid, so per-edge publisher attribution is unavailable in this evidence. Topic-level graph logs show /tf publishers anc_ekf_filter and robot_state_publisher; launch/config assigns odom -> base_footprint to the EKF.

The filter is not given /gzserver/get_entity_state. Only scripts/phase4_localization_test.py calls that service. Truth stays in local CSV/JSON evaluation output.

### Measurements

Accepted normal evidence: phase4-localization-normal-final7.json/.csv/.png and phase4-normal-launch-final7.log and phase4-normal-test-final7.log. Accepted slip evidence: phase4-localization-slip-final4.json/.csv/.png and phase4-slip-launch-final4.log and phase4-slip-test-final4.log.

| Measure | Result |
|---|---:|
| Raw / wheel / filtered / IMU rates | 29.41 / 29.41 / 29.98 / 97.26 Hz |
| Motion XY RMSE, raw -> filtered | 0.15010 m -> 0.04809 m |
| Motion yaw RMSE, raw -> filtered | 0.28447 rad -> 0.01530 rad |
| Mean filtered sigma x / y / yaw | 0.0704 m / 0.1091 m / 0.02908 rad |
| Stationary gyro z mean / standard deviation | -0.000013 / 0.001921 rad/s; configured variance is 0.000004 rad^2/s^2 |
| IMU dropout | 0 IMU messages during dropout; 74 filtered outputs during dropout; 105 IMU messages after recovery |
| Physical wall stall | Truth speed about zero; raw x error +0.3898 m; filtered x error +0.0204 m |
| Rectangle return endpoint | Truth itself ended 0.134 m and 0.980 rad from start; raw error 0.0765 m / 0.501 rad, filtered error 0.1596 m / 0.976 rad |
| Low-friction slip, mu=0.001 | Truth chassis delta 0.000 m; raw odometry +0.4705 m; filtered +0.4681 m |

The estimator improved aggregate motion RMSE and wall-stall x error in this run. It performed worse than raw odometry at the rectangle endpoint, where the test truth trajectory itself did not close. During slip, the fused estimate followed biased wheel speed because the available gyro only measures yaw rate and does not independently observe forward motion. This demonstrates the limitation instead of correcting it with simulator truth.

The test verified monotonic clock/sensor stamps, approximately shared /odom and /odometry/wheel stamps, stable filtered output, finite covariance, no large one-sample pose jumps, required TF chain presence, and IMU dropout/recovery. The timestamp comparison matches shared observed stamps rather than assuming separate subscribers receive equal message counts. Wheel-input dropout/staleness, multi-seed confidence, and statistical covariance calibration remain untested. Process noise, initial covariance, and wheel variances are provisional simulation tuning, not hardware calibration.

### Build, regression, and environment

- colcon build --symlink-install --packages-up-to anc_bringup: 4 packages finished.
- Static validation passed for Python compilation, YAML, EKF covariance dimensions/types, Xacro and check_urdf, default SDF wheel friction, Gazebo world SDF, and launch arguments. Full output: docs/evidence/phase4-build-final.log and phase4-validation-final.log.
- Phase 3 regression on the corrected model passed 20/20 checks. See phase3-sensor-smoke-phase4-final-model.json, phase3-sensor-telemetry-phase4-final-model.csv, phase3-camera-frame-phase4-final-model.png, and phase4-final-phase3-smoke.log.
- Final test environment reported 3.5 GiB WSL RAM total, about 3.0 GiB available, and unused 1 GiB swap. Gazebo remained headless; no system-wide install/upgrade or WSL restart was done.
- The Phase 4 trajectories were kept headless; no Phase 4-specific RViz screenshot was captured. The quantitative plots are saved with the normal and slip evidence; the earlier Phase 3 RViz screenshot remains indexed.

The Phase 4 test JSON records source HEAD 0e019ac38e0747b3856d474c93ad00c124631cb3, the accepted Phase 3 base commit. Phase 4 files were working-tree changes during measurements and are committed separately after validation.

## Phase 5: reproducible simulation worlds and scenario harness

**Outcome: PASS WITH LIMITS.** This phase implements reproducible, bounded test scenes and verifies the simulation harness. It does not implement or claim autonomous navigation.

### Scope and scenario registry

The registry retains roadmap IDs S00-S09. Phase 5 activates S00-S04 and S09; S05-S08 remain planned for later work. The six canonical worlds are stored under simulation/scenarios/worlds/:

- S00: empty open arena.
- S01: one box blocks the direct segment; a footprint-clear detour remains.
- S02: six seeded box, cylinder, sphere, and chair-proxy geometries with generic obstacle_NN names. No semantic class is assigned.
- S03: two offset barriers create alternating lower and upper constrictions.
- S04: seeded nonzero known-start pose and four seeded simple geometries.
- S09: a static divider and room boundary leave no footprint-clear route.

The arena outer bounds are x=[-4, 4] m and y=[-3, 3] m. The inside faces of the perimeter walls are x=[-3.85, 3.85] m and y=[-2.85, 2.85] m. Clearance validation uses the configured 0.264 m robot circumscribed radius plus 0.05 m margin. The generator checks physical obstacle footprints, start/goal clearance, configured corridor centerlines, and start-to-goal connectivity on a 0.10 m grid. Room geometry is composed from simple SDF models; the Phase 2 source world remains unchanged.

### Generator, runner, and truth boundary

scripts/scenario_generator.py reads the YAML registry, samples placements from the requested seed with deterministic Python PRNG state, rejects invalid placements, and builds SDF from the existing Phase 2 world skeleton. A manifest records scenario ID, seed, REFERENCE/REALISTIC profile label, start, goal, shape/pose/static fields, corridors, validation checks, and layout/world hashes. A Matplotlib plot records the generated layout and geometry route.

scripts/scenario_runner.py writes a run-specific world, manifest, and plot; launches anc_bringup sim.launch.py with that world and the same start_x/start_y/start_yaw values for Gazebo spawn and known-start map-to-odom; subscribes to camera, IMU, odometry, clock, and TF; and records run metadata and evaluator evidence. Only this evaluator calls /gzserver/get_entity_state and /gzserver/get_entities. It does not publish truth or obstacle layouts, send velocity commands, request a navigation goal, or report a goal as reached. Shutdown sends SIGINT to the runner's process group and records the exit code and whether that group is gone.

The REFERENCE and REALISTIC labels remain separate in the registry and evidence. Phase 5 does not introduce a sensor-profile switch or hardware-calibrated sensor/noise values; both labels currently use the existing common simulated robot sensor configuration.

### Validation and run results

Before changing Phase 5 code, the four-package workspace rebuilt successfully; the Phase 3 sensor regression passed 20/20 checks, and a fresh Phase 4 normal run passed its 13 checks with the recorded PASS WITH LIMITS decision. Baseline artifacts are phase5-baseline-build.log, phase5-baseline-phase3-test.log with its JSON/CSV/camera frame, and phase5-baseline-phase4-test.log with its JSON/CSV/plot and launch log.

- colcon build --symlink-install --packages-up-to anc_bringup: 4/4 packages built.
- python3 scripts/phase5_scenario_test.py: 47/47 checks passed. It checked all active scenario geometries, deterministic same-seed layout and SDF bytes, different valid S02/S04 layouts for seeds 101 and 202, the blocked S01 direct line, S09 no-route status, all six canonical worlds, and gz sdf -k for each world.
- S02 seed 101 repeat 1 and repeat 2: both passed all 13 runner checks. They produced identical layout SHA-256 bf4f06eaf451774aa9155aa163f805b43e993d69f770bab94c3aeefa8b8504b5 and SDF SHA-256 94e028f3cec36ea7c18ea798fe05d1e345ee0a930ab0e2d73c7804c5ed4a0c99. Each observed 59 camera frames, 385/387 IMU messages, 113/114 odometry messages, the requested entity names, and map-base TF within 0.00016 m of evaluator pose.
- S04 seed 202: all 13 checks passed. The requested known start was (-2.570240, -0.399685, -2.390039) and Gazebo truth was (-2.570230, -0.399678, -2.390040). The static map-to-odom matched the requested pose; composed map-to-base TF was within 0.00016 m and 0.00008 rad of evaluator truth. Camera/IMU/odometry counts were 61/396/117.
- S09 seed 101: all 13 checks passed with geometry route false, expected route false, sensor streams present, and TF consistent at the start. Camera/IMU/odometry counts were 60/390/115.
- All four scenario runs shut down with SIGINT, exit code 0, no forced escalation, and no remaining process group. A separate default-world regression also loaded the Phase 2 wall, spawned at the origin, checked zero map-to-odom, and shut down cleanly. The independent run directories preserve each SDF, manifest, plot, launch log, and result JSON.
- A preliminary standalone check using coreutils timeout did spawn the default robot, but the timeout wrapper sent a second interrupt and escalated Gazebo shutdown. That diagnostic is retained as phase5/default-world-timeout-diagnostic.log; the controlled process-group regression above is the accepted shutdown result.

The comparison summary is docs/evidence/phase5/phase5-summary.json. The default-world regression is docs/evidence/phase5/default-world-regression/result.json. Every live scenario run includes a generated layout.png visualization; the runner remained headless and did not capture a Gazebo GUI screenshot. Static checks are in docs/evidence/phase5/static-validation.json and .log; each live trial is under docs/evidence/phase5/runs/. docs/evidence/README.md indexes the build, checks, summaries, and run artifacts.

### Limits and Phase 6 readiness

The 10 cm grid uses a conservative circumscribed-circle footprint and a fixed safety margin; it is a scene-feasibility check, not a Nav2 planner, costmap, or calibrated collision model. Randomly generated obstacles are static in this phase. The static field is recorded and SDF honors it, but no dynamic-obstacle behavior was tested. Sensor-profile names are registry metadata only; this phase does not switch profiles or calibrate sensors.

No path-following command, Nav2 goal, RGB perception, class-agnostic occupancy layer, safe-stop behavior, unknown-pose acquisition, or parking test was run. S09 proves only that the geometry validator recognizes a blocked route; it does not prove that a robot detects the condition or stops safely. The infrastructure is ready for a separately scoped next phase to implement and test one of those behaviors. Hardware geometry, camera calibration, IMU bias/noise, wheel slip, and watchdog/e-stop requirements remain open.

## Reproduction commands

In the WSL project directory:

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/ashish/AutonomousNavigationCar
cd simulation/ros2_ws
colcon build --symlink-install --packages-up-to anc_bringup
source install/setup.bash
ros2 launch anc_bringup sim.launch.py
~~~

In a second WSL terminal while the launch is active:

~~~bash
source /opt/ros/jazzy/setup.bash
cd /home/ashish/AutonomousNavigationCar
source simulation/ros2_ws/install/setup.bash
python3 scripts/phase2_motion_test.py
~~~

Phase 4 normal replay, as executed for the accepted final7 evidence:

~~~bash
# Terminal 1
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
ros2 launch anc_bringup sim.launch.py
~~~

~~~bash
# Terminal 2
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
python3 scripts/phase4_localization_test.py --mode normal --run-id phase4-localization-normal-final7
~~~

The accepted slip replay used a fresh Gazebo process:

~~~bash
ros2 launch anc_bringup sim.launch.py wheel_surface_mu:=0.001
python3 scripts/phase4_localization_test.py --mode slip --run-id phase4-localization-slip-final4 --slip-mu 0.001 --slip-speed 0.25 --slip-duration 2
~~~

At evidence capture, git rev-parse HEAD returned 0e019ac38e0747b3856d474c93ad00c124631cb3. The Phase 4 changes were present as working-tree edits during those tests. The final commit hash is recorded by the repository commit and in the completion message.

The Phase 1 sample launch used headless mode. The map-corridor check and bag summarizer are scripts/check_reference_map_corridor.py and scripts/summarize_nav2_reference_bag.py. Full output and telemetry are indexed in docs/evidence/README.md.

### Phase 5 static validation and scenario replay

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install --packages-up-to anc_bringup
cd ../..
python3 scripts/phase5_scenario_test.py
~~~

For a fresh headless run:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
python3 scripts/scenario_runner.py --scenario S02 --seed 101 --run-id s02-seed101-replay
~~~

The run ID must be new because the runner preserves each evidence directory. Change the scenario and seed to replay another layout. The default-world regression is:

~~~bash
python3 scripts/phase5_default_world_smoke.py
~~~

## Official references

- ROS 2 Jazzy installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- ROS 2 Jazzy and Gazebo Harmonic pairing: https://gazebosim.org/docs/harmonic/ros_installation/
- Nav2 Jazzy quickstart: https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/
- Gazebo Harmonic sensor and contact systems: https://gazebosim.org/docs/harmonic/sensors/
- Gazebo Sim contact system runtime-spawn ordering diagnosis: https://github.com/gazebosim/gz-sim/issues/2223
- ros_gz supported bridge types: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md
- robot_localization sensor configuration: https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst
- SDFormat shape geometry and collision/visual elements: https://github.com/gazebosim/sdf_tutorials/blob/master/docs/spec_shapes/tutorial.md
- SDFormat world and model structure: https://github.com/gazebosim/sdf_tutorials/blob/master/docs/spec_world/tutorial1.md
- ros_gz_sim world and entity creation: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_sim/README.md
- robot_localization state-estimation nodes: https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/state_estimation_nodes.rst
- SDFormat URDF extensions and wheel friction conversion: https://sdformat.org/tutorials/specification/sdformat_urdf_extensions/1.6/
- Gazebo Harmonic camera frame and CameraInfo migration notes: https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/

## Remaining work after Phase 7

Phase 7 provides a camera-only visual label for a narrow, calibrated portion of simulated floor; it does not establish general obstacle perception, metric depth, online mapping, safe navigation, or sim-to-real readiness. The next navigation phase has not started. Remaining work includes measuring the real chassis and camera, improving wheel-slip behavior, expanding camera calibration and appearance coverage, and raising occupied recall without introducing false-FREE predictions. The Phase 6 ORACLE_REFERENCE baseline and its S09 truth-twist/odometry discrepancy remain separately documented.

## Phase 6: REFERENCE Nav2 navigation

### Result

Status: PASS WITH LIMITS for the selected scenario set. The custom car used Nav2 NavigateToPose with NavFn planning, Regulated Pure Pursuit, a bounded command guard, wheel/IMU EKF odometry, and a known-start map-to-odom transform. The camera is disabled only in this headless Phase 6 launch to reduce rendering load; existing simulator defaults and Phase 3/5 camera tests remain unchanged. Nav2 received a per-run 5 cm occupancy map generated from scenario geometry. It did not receive Gazebo entity-state truth or obstacle geometry as ROS inputs.

| Scenario | Run ID | Action outcome | Truth path | Min clearance | Contacts | Localization drift |
|---|---|---|---:|---:|---:|---:|
| S00 | s00-reference-final-stop-check | SUCCEEDED | 1.853 m | 1.7326 m | 0 | 0.001 m |
| S01 | s01-reference-final-stop-check | SUCCEEDED | 6.141 m | 0.1129 m | 0 | 0.016 m |
| S02 | s02-reference-final-stop-check | SUCCEEDED | 6.644 m | 0.1190 m | 0 | 0.029 m |
| S03 | s03-reference-final-stop-check | SUCCEEDED | 9.086 m | 0.0155 m | 0 | 0.060 m |
| S04 | s04-reference-final-stop-check | SUCCEEDED | 6.110 m | 0.1065 m | 0 | 0.064 m |
| S09 | s09-reference-seed101-pose-odom-stop | ABORTED (expected no-route; error 208, 15 recoveries) | 0.456 m | 0.5128 m | 0 | 0.080 m |

All six final-code runs have individual result.json and MCAP evidence under docs/evidence/phase6/runs/. S00-S04 returned SUCCEEDED with error code 0; S09 returned ABORTED with error code 208, not SUCCEEDED, and did not time out. Every run's simulation process group exited and no simulation processes remained. The final S01 run records launch-managed SIGTERM/SIGKILL escalation of its Gazebo child; see its result limitations. The complete summary is docs/evidence/phase6/phase6-summary.json.

### Validation

- colcon build --symlink-install --packages-up-to anc_navigation: 5 packages finished; see docs/evidence/phase6/build-final.log.
- Python syntax compilation and 19 Phase 6 static acceptance checks passed; see docs/evidence/phase6/static-validation.log.
- Both Xacro camera modes parsed: zero RGB camera sensors with camera disabled, one with it enabled, and one IMU in both modes; see docs/evidence/phase6/xacro-camera-toggle.log.
- MCAP inspection confirmed one known-start map-to-odom edge on /tf_static and the recorded odom-to-base_footprint edge on /tf. The live /tf publisher graph contained anc_ekf_filter and robot_state_publisher only; the EKF is the sole configured publisher for odom-to-base_footprint.
- The command guard stayed within 0.25 m/s and 1.0 rad/s. No non-ground contact collisions were recorded.

### Limits and remaining work

S03 had the smallest conservative clearance at 0.0155 m, and should be treated as a very tight path in this placeholder geometry. S03 and S04 final localization drift was about 0.060 m and 0.064 m, respectively. S09 required 15 Nav2 recoveries and moved about 0.456 m along its bounded failed attempt before stopping; it is evidence of a static no-route abort, not zero movement or a general emergency-stop test. Its final Gazebo entity-state twist reported about 0.102 rad/s angular velocity while the truth pose was unchanged over the final 0.986 s, raw odometry was zero, filtered odometry was near zero, and command topics were zero. The discrepancy is preserved in the run result and must be resolved before using entity-state twist as a stop signal.

This phase does not implement RGB FREE/OCCUPIED/UNKNOWN perception, unknown-start localization, dynamic obstacles, sensor-loss recovery, parking, hardware watchdog, or an electrical emergency stop. Chassis/wheel geometry, camera calibration, IMU noise/bias, and wheel slip remain open sim-to-real limits.

## Phase 7: class-agnostic RGB FREE / OCCUPIED / UNKNOWN perception

### Result and scope

**Assessment: PASS WITH LIMITS.** Phase 7 adds the anc_perception ROS 2 package and separate REALISTIC_RGB_ONLY launch/configuration. Its launch uses the Phase 3 RGB camera, sanitized IMU, and existing EKF odometry, but the perception node subscribes only to /camera/image_raw, /camera/camera_info, and /clock. Clock is used only for timestamp age. The node publishes /perception/image_mask (mono8: FREE=0, OCCUPIED=100, UNKNOWN=255), /perception/confidence, /perception/overlay, and /perception/health.

The node does not use odometry, IMU, map, scan, depth, Gazebo entity state, scenario geometry, manifests, or the Phase 6 ORACLE_REFERENCE map. Ground truth is generated by a separate offline evaluator and is not imported by the runtime package. The Phase 6 reference launch/costmap remains a separate runnable baseline.

### Method and geometry boundary

The CPU-only OpenCV classifier uses generic color/edge cues and a fixed Lab floor appearance calibration from the saved Phase 3 RGB image. A ground-plane projection assumes the provisional 320x240 pinhole camera, CameraInfo intrinsics (fx/fy about 277.13 px, center 160/120), zero distortion, level camera (0 rad pitch), camera origin about 0.18 m above the floor, and a planar floor. FREE is emitted only in a camera-centered forward band of 0.50-0.60 m. A calibrated-looking seed and a connected floor region of at least 4,000 pixels are required. Distortion, stale/missing RGB, invalid camera calibration, or insufficient floor support fails closed to UNKNOWN.

The calibration is specific to simulated floor material, lighting, camera response, and provisional transform. It is not a general floor detector. The narrow valid band and high UNKNOWN rate avoid treating visually ambiguous same-color obstacle surfaces as traversable; no arbitrary metric obstacle distance is claimed.

### Build and tests actually executed

- Full colcon build --symlink-install in simulation/ros2_ws: six packages finished (docs/evidence/phase7/build-full.log).
- Static classifier/profile validation: 13/13 checks passed (docs/evidence/phase7/static-validation.log).
- Live sensor fault test: PASS. Missing RGB emitted INVALID_UNKNOWN/all-255 mask/confidence 0; fresh RGB returned a valid 5,120-pixel FREE band; a 0.5 s stale simulated timestamp emitted INVALID_UNKNOWN/all-255 and preserved the source image stamp (docs/evidence/phase7/fault-injection.json).
- Full headless Gazebo dataset: all six cases launched and shut down cleanly, with two views each (12 RGB/mask/truth samples); graph inspection confirmed exactly the three expected input topics per case. Cases: S00 seed 101, S01 seed 101, S02 seed 101, S03 seed 101, S04 seed 202, P7C1 derived from S01 seed 101.
- The Phase 3 camera/sensor smoke check and Phase 6 S00 ORACLE_REFERENCE navigation were rerun separately as pre-change baselines and retained. The Phase 7 RGB dataset did not modify the reference navigation implementation or switch its profile.

### Accepted aggregate measurements

Metrics aggregate 12 images at 320x240 (921,600 pixels); evaluator truth excludes unknown/out-of-view pixels from class-specific denominators.

| Measure | Result |
|---|---:|
| Predicted FREE / OCCUPIED / UNKNOWN pixels | 51,200 / 124,082 / 746,318 |
| Evaluator-known FREE / OCCUPIED pixels | 58,173 / 465,242 |
| False-FREE pixels / rate on known occupied | 0 / 0.0% |
| OCCUPIED precision / recall | 100.0% / 26.67% |
| Useful FREE coverage on known floor | 88.01% |
| UNKNOWN coverage of all predictions | 80.98% |
| Processing latency p50 / p95 / max | 12.827 / 22.902 / 230.119 ms |
| Measured frame rate median / p10 | 14.093 / 12.932 FPS |
| Camera simulated timestamp age p50 / p95 / max | 0.022 / 0.054 / 0.128 s (0.35 s limit) |

These are not a single accuracy score: low occupied recall and large UNKNOWN share are material limitations. S03/S04 low-contrast obstacle pixels were often UNKNOWN. In P7C1, both views were INVALID_UNKNOWN because calibrated floor support was too small or unobservable; this is a conservative appearance failure, not a successful obstacle classification. False-positive and uncertain examples, RGB, predicted mask, confidence, overlay, and offline truth for each view are kept under docs/evidence/phase7/dataset/accepted-phase7-final-verified/.

### Resource and shutdown observations

The accepted run recorded about 2.8 GiB available memory under the WSL 3.5 GiB memory limit after the trial. Sampled maximum RSS was about 537 MiB for Gazebo and about 186 MiB for the Python perception node. The dataset added about 1.9 MB; no model downloads or large neural-network dependencies were used. Every accepted process group exited after SIGINT without forced escalation, and no simulation or perception processes remained afterward.

### Limits and next boundary

The classifier is color- and lighting-specific, uses only a narrow ground band, and misses many low-contrast occupied pixels by marking them UNKNOWN. Its output is not a 2D costmap, depth map, obstacle-distance estimate, or robot command. Online mapping, planning, autonomous obstacle avoidance, unknown-start localization, sensor-loss driving policy, parking, and hardware safety are not implemented. Phase 8 subsequently added a stop-only baseline. Its motion gate remains BLOCKED because FREE evidence is disconnected from the footprint and hard-stop distance is unmeasured. Recalibrate the actual camera/floor, improve occupied recall while retaining the zero false-FREE result, and broaden validation before any later navigation claim.

Evidence and reproducible commands are indexed in docs/evidence/README.md and scripts/README.md; dataflow and projection assumptions are in docs/phase7-perception-architecture.md.

## Phase 8: REALISTIC_RGB_ONLY local traversability and safe stopping

### Result

**Classification: PASS WITH LIMITS for the stop-only baseline. Autonomous motion is BLOCKED by the feasibility gate.** The system reports STOP when a continuous footprint-clear corridor and stopping distance cannot be justified. No Phase 8 goal or robot motion was attempted.

The projected S00 FREE patch spans about 0.090 m longitudinally at base_link x=0.604-0.694 m. The robot front envelope is x=0.215 m, leaving about 0.389 m of UNKNOWN. Phase 7's accepted challenge aggregate has 80.98% UNKNOWN and 26.67% occupied recall. Camera/processing/supervisor p95 reaction distance at 0.18 m/s is about 0.024 m; adding a 0.20 m margin gives about 0.224 m before braking, whose distance remains unmeasured. The FREE patch is shorter than this lower bound and disconnected from the footprint.

### Implementation and safeguards

The new anc_realistic_navigation package publishes a fresh, robot-relative OccupancyGrid in base_link. The 75 x 75 map uses 0.02 m cells with FREE=0, OCCUPIED=100, UNKNOWN=-1. Only high-confidence FREE pixels are projected with live CameraInfo and a TF-verified camera transform. Invalid/stale image, confidence, health, CameraInfo, simulation clock, filtered odometry, covariance, frame, or TF invalidates the snapshot to UNKNOWN. No UNKNOWN is promoted to FREE; obstacle-like RGB pixels without metric range remain UNKNOWN and block.

The separate phase8_realistic.launch.py starts no Nav2 or reference map and passes publish_known_start_tf:=false. It keeps wheel/IMU EKF odometry local, without the Phase 6 known-start map-to-odom transform. The existing launch default remains true for the separate Phase 6 ORACLE_REFERENCE profile. The safe-stop supervisor is the only /cmd_vel publisher and emits zero Twist at 20 Hz unconditionally. Requested commands are isolated on /cmd_vel/requested. Gazebo truth and scenario geometry are accessed only by the live test evaluator.

### Validation

- Full ROS workspace build: seven packages finished.
- Projection/fail-closed static suite: 7/7 passed.
- Nine final live Phase 8 stop runs passed across S00, S01, S02, S03, and S09; S02 was repeated with seeds 101 and 202. Checks covered missing camera, missing localization, fresh but insufficient RGB, unlocalized obstacle evidence, command timeout, sole command authority, no Nav2, no known-start map-to-odom TF, zero command output, and evaluator-only pose.
- All 1,052 actual /cmd_vel samples were zero; maximum evaluator displacement was 0.00001055 m; all runs recorded zero non-ground contact events. Repeated caster-to-ground surface messages are not counted as obstacle collisions.
- The Phase 7 fault harness passed missing-RGB, stale timestamp, fresh-input, and stale-to-fresh recovery checks.
- A separate Phase 6 S00 ORACLE_REFERENCE rerun returned NavigateToPose SUCCEEDED: path 1.8546 m, final goal error 0.1454 m, conservative clearance 1.7314 m, localization drift 0.0019 m. It is a reference comparison only and does not demonstrate RGB-only navigation.

Per-run JSON/CSV/log evidence is under docs/evidence/phase8/runs/; aggregate values are in docs/evidence/phase8/phase8-summary.json. Full feasibility and topic/TF details are in docs/phase8-feasibility.md.

### Limits and remaining work

No movement, local route traversal, goal-reaching, or moving sensor-loss stop was demonstrated. Emergency braking distance is not measured. Camera geometry, chassis footprint, visual floor model, and estimator uncertainty remain provisional simulation assumptions. Dynamic camera loss during motion, local map recovery under a live stream interruption, and an end-to-end controller/planner are deferred until a continuous safe corridor and full stopping-distance bound can be demonstrated. Phase 8 did not touch the Arduino UNO Q application or start parking or hardware phases.


## Phase 8B: RGB camera geometry and stopping-feasibility assessment

**Assessment complete with limits; autonomous motion remains BLOCKED.** Camera z/pitch, ground-projection interval, and floor-seed candidates were evaluated on fixed scenarios. The runner recorded actual camera TF for projection evaluation. C0-C2 completed multi-view sweeps (C1 was repeated to cover both fixed S02 seeds); C3/C4 were exploratory S00 live checks.

C2 (camera height 0.24 m, downward pitch 0.80 rad, 0.10-0.90 m ground interval) produced the longest connected full-footprint corridor: 0.48 m in 10/12 evaluated views; two views had zero corridor. It recorded zero false-FREE pixels, but also 0% occupied recall across 3,390 evaluator-known occupied pixels. The corridor remains short of the 0.656429 m gate requirement. The baseline and moderate-pitch candidates had no connected corridor.

A separate empty-world test completed 24/24 moving trials at 0.02, 0.05, 0.10, and 0.18 m/s, under simulated friction mu=1.2 and 0.5, with three repeats per condition. The largest settled stop distance was 0.00298 m at 0.18 m/s, mu=0.5. It was measured by evaluator-pose finite differences, because the Gazebo entity-state twist was stale at low speed. This measurement is limited to the simulator and tested friction.

The conservative gate calculation is:

```text
0.18 * (0.174 s camera age + 0.2368 s perception latency + 0.050 s supervisor period)
+ 0.00298 m stop distance
+ 0.470505 m Phase 4 slip-derived localization allowance
+ 0.000 m forward footprint addition (measurement starts at the front footprint boundary)
+ 0.100 m explicit safety margin
= 0.656429 m required connected corridor
```

The full-footprint width requirement is 0.345 m, including 0.02 m lateral clearance on either side. The 0.470505 m allowance comes from the Phase 4 low-friction endpoint error with no evaluator chassis displacement; it replaces the Phase 8 EKF's zero covariance as the conservative allowance. Since the best corridor is only 0.48 m and C2 missed every known occupied pixel, the motion gate stays BLOCKED. No motion controller, autonomous driving scenario, or RGB Nav2 run was enabled. The existing stop-only lock remains intact.

The seven-package ROS build and Phase 6/7/8 regression checks passed. Candidate, per-view, stopping-distance, and resource evidence is indexed in [docs/evidence/phase8b/README.md](evidence/phase8b/README.md). Full details: [docs/evidence/phase8b/phase8b-report.md](evidence/phase8b/phase8b-report.md).


## Phase 8C Branch 1: RGB + 2D LiDAR

### Assessment

| Milestone | Assessment | Evidence |
|---|---|---|
| Sensor integration | PASS WITH LIMITS | 360-beam lidar_link scan at 4.94 Hz; 358/360 beams usable (99.44%). RGB/CameraInfo, IMU, filtered odometry and TF were present. |
| Local map and fusion | PASS WITH LIMITS | Single-scan 180×180, 0.10 m map: 4,206 FREE, 267 OCCUPIED, 27,927 UNKNOWN. RGB yielded 617 FREE, 5,008 UNKNOWN, no metric OCCUPIED; fused S00 grid equaled LiDAR-only. |
| Safety gate | PASS WITH LIMITS | S00 corridor 3.20 m; 0.656429 m required at 0.18 m/s. A 0.50 m obstacle clearance stayed stopped; 0.582701 m is required at 0.02 m/s. |
| Actual movement | PASS WITH LIMITS | Bounded straight requests at 0.05 m/s; watchdog trial moved 0.212 m, then settled. Fault trials settled after 0.047–0.057 m total travel with post-zero movement 0–0.000082 m. |
| Avoidance / goal navigation | BLOCKED / NOT RUN | Angular limit is 0 rad/s. No obstacle-course avoidance or Nav2 goal success is claimed. |

### Implementation and runtime boundary

anc_lidar adds REALISTIC_RGB_LIDAR: parameterized Gazebo planar scanner and bridge, scan conditioner, single-scan ray-traced map, footprint inflation, optional conservative RGB fusion, measured speed gate, and command guard. Provisional config uses 360 samples over ±π, 0.12–6.0 m, 5 Hz, 0.01 m distance-noise sigma, and configurable missed/invalid readings. This generic sensor is not a hardware specification. Its scan plane is 0.16 m above ground and cannot guarantee detection below it or overhead.

Observed Gazebo unobstructed rays arrived as +Inf; the conditioner maps that observed representation to a finite max-range no-return. NaN, negative infinity and invalid/out-of-range samples remain UNKNOWN. Valid return endpoints are OCCUPIED; only measured space before them is FREE; behind hits and unobserved space remains UNKNOWN. The map is a fresh single-scan snapshot. RGB cannot erase LiDAR OCCUPIED cells or fabricate metric obstacle locations.

Graph audits confirmed runtime nodes did not consume the Phase 6 reference map or Gazebo truth. Truth was queried only by evaluators, never published to runtime. anc_safety_command_guard was the sole /cmd_vel publisher; requests were isolated on /cmd_vel_nav. Shared launch keeps LiDAR disabled by default. Phase 8 remains a separate unconditional-zero profile.

### Safety and controlled trials

The gate uses scan validity, age/rate, paired map health, filtered-localization freshness/covariance, processing/supervisor delay, Phase 8B stopping data, Phase 4 slip allowance, full-footprint clearance and margin. The clearance probe yielded 0.50 m and was denied motion (0.582701 m required at 0.02 m/s). Invalid scan, paused scan conditioner, stale localization, paused RGB perception and stale command tests produced zero authoritative output; recovery followed restored inputs.

Moving fault trials at 0.05 m/s request speed:
- LiDAR loss: 0.046981 m total displacement, 0.000082 m after zero.
- RGB-perception loss: 0.056990 m total, 0 m after zero.
- Localization loss: 0.049390 m total, 0 m after zero.
- Command timeout: 0.212 m total forward displacement, 0 m after zero.

Each trial settled within the 0.50 m displacement and 0.10298 m stopping limits. The gate’s clear-corridor cap was 0.18 m/s; no yaw command was permitted. One Phase 8B low-friction sample measured 0.002976 m at 0.1588 m/s observed speed (target 0.18, μ=0.5). This is a simulator sample, not a hardware stopping bound.

### Resource and localization limits

WSL has about 3.5 GiB RAM. The LiDAR profile reduces camera rate to 5 Hz. Under the combined profile Gazebo used about 106% CPU and an EKF update-rate warning occurred; other ROS processes also used substantial CPU. Optional SLAM generated /map and stationary map→odom but had a full scan queue and EKF timing warnings. No drift improvement was established, so SLAM/Nav2 remain off.

A fresh Phase 4 normal replay had one initial wall-stationarity assertion failure at 0.01255 m/s; a retry passed. Retry filtered motion RMSE was 0.0568 m vs 0.1731 m raw, but filtered return-to-start error (0.1014 m, 0.533 rad) was worse than raw (0.0145 m, 0.014 rad). This wheel/gyro regression does not show LiDAR scan-matching drift correction.

### Regression record

The final build completed eight ROS packages. Phase 8C tests passed common 8/8, scan/map 13/13, CameraInfo pairing 3/3. Fresh Phase 3 sensor smoke passed 20/20 (RGB/CameraInfo 15.07 Hz, IMU 99.69 Hz, odometry 29.41 Hz, TF/contact passed). Phase 4 normal retry passed with the stated return-error limits. Static regressions passed Phase 5 47/47, Phase 6 profile 19/19, Phase 7 13/13, Phase 8 projection 7/7. Fresh Phase 8 S00 stop-only regression had 125/125 zero /cmd_vel samples, sole Phase 8 publisher, no Nav2 and no displacement. Phase 8B one-sample repeat passed at 0.002976 m. Existing full Phase 6 Nav2 trials and Phase 8B 24-run sweep were retained but not repeated on this low-memory branch. See docs/evidence/phase8c/lidar/README.md.

### Remaining work

The 2D plane misses low obstacles and overhead structure; reflectivity, dynamic obstacles and varied surfaces were not evaluated. RGB added no cells on the tested corridor and supplies no metric obstacle range. Wheel/gyro return error remains significant, and scan matching has not been shown to improve it. No obstacle course, goal reaching, parking, hardware integration or UNO Q change is included. Stage D remains blocked pending yaw stopping, turning clearance, localization/map quality and resource validation.


## Phase 8C remaining LiDAR stages: supplemental report (2026-11-02)

This additive continuation preserves earlier report history and failed attempts. Full matrix is in docs/phase8c-stage-results.md.

### Stage 1: calibration-only rotation and swept clearance

Six S00 1.2-second pure-yaw pulses at +/-0.02, +/-0.05 and +/-0.10 rad/s passed. Command-to-zero output latency was 0.00386-0.04985 s; first slow pose-derived rate followed 0.019-0.172 s after zero; settling took 0.579-0.722 s. Maximum center drift was 0.004337 m; residual yaw was zero in saved samples and there were no non-ground contacts. Swept disk radius was 0.523593 m from provisional footprint circumradius, 0.14 m drift allowance, 0.02 m sensor uncertainty and 0.10 m margin.

The near-wall no-motion gate denied rotation with 18 occupied and 4 unknown cells. Near-box/constriction probes also found occupied/unknown cells, but both additionally failed RGB floor-support validation and are not isolated clearance-only tests. The sole /cmd_vel publisher remained the guard. Production angular speed stays 0.0 rad/s: short clear-arena pulses do not establish general yaw stopping distance or combined-motion safety. The initial failed evaluation remains at stage1-rotation-trials.json/.log.

### Stage 2: tuned stationary SLAM map

SLAM now uses a 0.25 s minimum scan interval (4 Hz processing), queue size 3, 0.05 s transform publishing and 3 s map updates. In 25.001 wall seconds / 24.53 simulation seconds, stationary S00 received 128 scans, 8 map messages and 508 map-to-odom TF messages. It saved a 155 x 115 map at 0.05 m resolution: 1,447 free, 6 occupied, 16,372 unknown. All 520 /cmd_vel samples were zero; there were no non-ground contacts.

No SLAM pose messages arrived. Stationary truth drift is not localization accuracy or drift improvement. The tuned log had no queue-full or EKF missed-update-rate warning during this 25-second interval, but did include a startup TF-cache drop and diagnostic-subscriber QoS warnings. This does not prove earlier timing issues resolved. The first failed monitor QoS attempt remains preserved.

### Stages 3-5: navigation and parking

Nav2 goals, obstacle traversal, dynamic-obstacle response, replanning and live parking were not attempted. They require production yaw stopping and moving localization evidence not established. The non-authoritative parking proposal passed 7/7 unit tests, blocks in calibration mode, and is not wired to runtime velocity.

### Stage 6: profile-separated evaluation and resources

Generated JSON, CSV and plots aggregate saved evidence. Phase 6 ORACLE_REFERENCE succeeded on reachable S00-S04 (5/5), with mean final XY error 0.14382 m and zero collisions; S09 aborted for the expected no-route case. Phase 7 RGB had 12 views, 0 false-FREE rate on known occupied pixels, 88.01% useful floor FREE coverage, 100% occupied precision, 26.67% occupied recall and 80.98% UNKNOWN. Phase 8 RGB-only stop profile passed 9/9 with 1,052/1,052 zero command samples and maximum displacement 0.00001055 m. Phase 8C had 358/360 usable beams at 4.94 Hz; RGB added no cells over LiDAR in tested clear S00.

These are different profiles, not a head-to-head benchmark. Phase 6 used oracle map; Phase 7 evaluated RGB labels; Phase 8 stayed stopped; Phase 8C measured LiDAR and bounded straight motion.

WSL has about 3.5 GiB RAM. Phase 8 recorded max sampled process RSS 852,672 KiB and minimum available memory 1,956,248 KiB. One summarized process sample during stationary SLAM reported about 2.2 GiB available, Gazebo RSS about 508,616 KiB and instantaneous process CPU near 112%; this is not a whole-system peak or time series. After simulation shutdown idle available memory was about 3.0 GiB. These are capacity observations, not a controlled benchmark ranking.

### Stage 7: UNO Q transfer preparation

docs/phase8c-hardware-transfer.md documents Linux MPU / STM32 responsibilities, Bridge contract, watchdog/E-stop requirements, LiDAR comparison and checklist. UNO Q Debian Trixie compatibility with ROS Jazzy remains unverified; Jazzy Tier 1 is Ubuntu Noble arm64. No UNO Q software, Arduino app, wiring or hardware changed. Hardware readiness is PARTIAL; physical validation PENDING HARDWARE.

### Stage 8: saved demo and run guide

A headless sensor composite is saved at docs/evidence/live-demo-2026-11-02/phase8c-live-demo.mp4 and described by its report. It shows RGB, scan, safety state and /cmd_vel; it is not a Gazebo desktop video or Nav2 path. MP4 remains local under the existing *.mp4 ignore rule. The operation guide and config/phase8c_lidar.rviz provide sensor/stationary-map workflows. RViz was not opened in this pass.

### Final capability classifications

| Capability | Result |
|---|---|
| LiDAR sensing | PASS WITH LIMITS |
| Local mapping | PASS WITH LIMITS |
| Safe rotation | PASS WITH LIMITS - calibration only; production yaw disabled |
| SLAM and moving localization | BLOCKED |
| Autonomous goal navigation | BLOCKED |
| Obstacle avoidance | BLOCKED |
| Replanning | BLOCKED |
| Autonomous parking | BLOCKED live; 7 unit tests pass |
| Fault handling | PASS WITH LIMITS for tested forward-motion and stop-only faults |
| Full simulation evaluation | BLOCKED / INCOMPLETE |
| Hardware deployment readiness | PARTIAL |
| Physical hardware validation | PENDING HARDWARE |

GitHub issue listing returned no existing issues; none was created. Branch remains feature/phase8c-lidar; no merge to main was made.


## Phase 8C Stage 9 update (2026-11-02)

Nav2 launch/configuration, a guarded command chain, an evaluator, a GUI/RViz path and a memory/readiness-gated demo launcher were added. A follow-up strengthened moving-SLAM trial measurements and replaced the launcher file-existence check with content-based evidence validation. The single-package anc_lidar build and offline checks pass. A post-build host-memory probe measured 926 MiB available on Windows and 3134 MiB in WSL; Windows remains below the headless launcher's 2 GiB threshold, so no new Gazebo, moving-SLAM or Nav2 trial was started. Production yaw remains zero and the readiness gate is blocked with evidence references unset. Full results and the current capability matrix are in docs/phase8c-stage9-report.md.
