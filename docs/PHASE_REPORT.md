# Phase 0 through Phase 3 report

Date: 2026-11-01 (Asia/Calcutta)
Project: /home/ashish/AutonomousNavigationCar on Ubuntu-24.04 at E:/WSL/Ubuntu-24.04
Branch: main
Scope: Phase 0 planning baseline, Phase 1 reference navigation validation, Phase 2 custom differential-drive robot, and Phase 3 sensor simulation/ROS interfaces.

## Outcome

- Phase 0 planning baseline: PASS. The independent repository remains tagged v0.1-phase0. Physical hardware values remain unresolved.
- Phase 1 environment gate: PASS. WSL2 Ubuntu 24.04.5, ROS 2 Jazzy, Gazebo Harmonic, ros_gz, RViz2, Nav2, the packaged TurtleBot3 simulator, and available resources were checked.
- Phase 1 Task A reference navigation: PASS. Nav2 accepted a map-frame goal and returned SUCCEEDED with error code 0. Localization, known-free corridor, nonzero controller output, odometry motion, and final AMCL pose were checked.
- Phase 2 Task B custom robot: PASS WITH LIMITS. The custom differential-drive robot built and spawned headlessly. Forward, reverse, rotation, odometry, base TF, and a physical wall stop passed. The configured contact sensor and ROS bridge produced zero contact messages; odometry also diverged under the wall stall. These limits are recorded below.
- Phase 3 sensor simulation and ROS interfaces: PASS WITH LIMITS. RGB/CameraInfo, sanitized IMU, clock, odometry, joint states, TF, wall contact, RViz, and a bounded bag were verified. Sensor parameters remain provisional and wall-stall odometry still diverges.

## Scope and integrity

The supplied roadmap and task text governed the phase gates. Gazebo entity state was used only by the Phase 2/3 test evaluators. Neither Nav2 nor a perception/controller node received Gazebo ground-truth pose or obstacle geometry. The Nav2 run is labeled REFERENCE because it used the packaged map and simulated laser scan; it is not evidence of RGB-only navigation. All custom vehicle dimensions are design placeholders, not measured hardware values.

No package installation, system-wide upgrade, Windows/WSL restart, or Arduino UNO Q application change was made during these validation tasks. Gazebo was stopped cleanly after tests. No Phase 2 screenshot was saved; tests were headless.

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

**Outcome: PASS WITH LIMITS.** No Phase 4 or later work was started.

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

The Phase 1 sample launch used headless mode. The map-corridor check and bag summarizer are scripts/check_reference_map_corridor.py and scripts/summarize_nav2_reference_bag.py. Full output and telemetry are indexed in docs/evidence/README.md.

## Official references

- ROS 2 Jazzy installation: https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html
- ROS 2 Jazzy and Gazebo Harmonic pairing: https://gazebosim.org/docs/harmonic/ros_installation/
- Nav2 Jazzy quickstart: https://docs.nav2.org/jazzy/getting_started/quickstart/quickstart/
- Gazebo Harmonic sensor and contact systems: https://gazebosim.org/docs/harmonic/sensors/
- Gazebo Sim contact system runtime-spawn ordering diagnosis: https://github.com/gazebosim/gz-sim/issues/2223
- ros_gz supported bridge types: https://github.com/gazebosim/ros_gz/blob/ros2/ros_gz_bridge/README.md
- Gazebo Harmonic camera frame and CameraInfo migration notes: https://gazebosim.org/docs/harmonic/migrating_gazebo_classic_ros2_packages/

## Next recommended task

Review the Phase 3 implementation and evidence before starting Phase 4. Preserve REFERENCE/REALISTIC separation and Gazebo truth isolation. Before hardware-faithful or sim-to-real claims, measure the physical chassis, camera calibration, and IMU noise; improve wheel odometry under slip/stall; and define contact-event semantics. Phase 4 remains unstarted.
