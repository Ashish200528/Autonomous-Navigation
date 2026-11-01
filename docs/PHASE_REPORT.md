# Autonomous Navigation Car simulation phase report

Date: 2026-11-02 (Asia/Calcutta)
Project: /home/ashish/AutonomousNavigationCar on Ubuntu-24.04 at E:/WSL/Ubuntu-24.04
Branch: main
Scope: Phase 0 baseline through Phase 6 reference-only Nav2 navigation.

## Outcome

- Phase 0 planning baseline: PASS. The independent repository remains tagged v0.1-phase0. Physical hardware values remain unresolved.
- Phase 1 environment gate: PASS. WSL2 Ubuntu 24.04.5, ROS 2 Jazzy, Gazebo Harmonic, ros_gz, RViz2, Nav2, the packaged TurtleBot3 simulator, and available resources were checked.
- Phase 1 Task A reference navigation: PASS. Nav2 accepted a map-frame goal and returned SUCCEEDED with error code 0. Localization, known-free corridor, nonzero controller output, odometry motion, and final AMCL pose were checked.
- Phase 2 Task B custom robot: PASS WITH LIMITS. The custom differential-drive robot built and spawned headlessly. Forward, reverse, rotation, odometry, base TF, and a physical wall stop passed. The configured contact sensor and ROS bridge produced zero contact messages; odometry also diverged under the wall stall. These limits are recorded below.
- Phase 3 sensor simulation and ROS interfaces: PASS WITH LIMITS. RGB/CameraInfo, sanitized IMU, clock, odometry, joint states, TF, wall contact, RViz, and a bounded bag were verified. Sensor parameters remain provisional.
- Phase 4 odometry, state estimation, and known-start localization: PASS WITH LIMITS. The wheel/gyro EKF reduced motion RMSE and wall-stall x error in the accepted run, but did not correct wheel slip and was worse than raw odometry at the non-closing rectangle endpoint.
- Phase 5 reproducible simulation worlds and scenario harness: PASS WITH LIMITS. Six bounded worlds, deterministic seeded layouts, geometry route/no-route checks, four headless runs, known-start TF, sensor streams, evaluator-only truth, and clean shutdown were verified. No navigation or perception was attempted.
- Phase 6 REFERENCE Nav2 navigation: PASS WITH LIMITS. The selected S00/S01/S02/S03/S04/S09 trials passed using a per-run geometry-derived oracle map, known-start EKF TF, headless Gazebo, collision/clearance/truth evaluation, rosbag evidence, and bounded teardown. This is not RGB or unknown-map navigation.

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

## Next recommended task

Phase 5 geometry generation and harness checks are complete with limits. The Phase 6 ORACLE_REFERENCE Nav2 baseline passes its selected cases with limits; it does not establish RGB-only navigation or sim-to-real readiness. Next, resolve the S09 truth-twist/odometry discrepancy, improve clearance and drift margins, and measure chassis, wheel, encoder, camera, and IMU properties before hardware-faithful work. Unknown-start localization, RGB perception, dynamic obstacles, sensor-loss recovery, parking, and hardware emergency stop remain future phases.

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
