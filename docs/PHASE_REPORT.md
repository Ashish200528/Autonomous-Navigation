# Phase 0, Phase 1, and Phase 2 report

Date: 2026-11-01 (Asia/Calcutta)
Project: /home/ashish/AutonomousNavigationCar on Ubuntu-24.04 at E:/WSL/Ubuntu-24.04
Branch: main
Scope: Phase 0 baseline, complete Phase 1 reference navigation validation, and first Phase 2 differential-drive robot test.

## Outcome

- Phase 0 planning baseline: PASS. The independent repository remains tagged v0.1-phase0. Physical hardware values remain unresolved.
- Phase 1 environment gate: PASS. WSL2 Ubuntu 24.04.5, ROS 2 Jazzy, Gazebo Harmonic, ros_gz, RViz2, Nav2, the packaged TurtleBot3 simulator, and available resources were checked.
- Phase 1 Task A reference navigation: PASS. Nav2 accepted a map-frame goal and returned SUCCEEDED with error code 0. Localization, known-free corridor, nonzero controller output, odometry motion, and final AMCL pose were checked.
- Phase 2 Task B custom robot: PASS WITH LIMITS. The custom differential-drive robot built and spawned headlessly. Forward, reverse, rotation, odometry, base TF, and a physical wall stop passed. The configured contact sensor and ROS bridge produced zero contact messages; odometry also diverged under the wall stall. These limits are recorded below.
- Phase 3 has not started. The camera link is only a placeholder frame; no camera or IMU sensor is simulated.

## Scope and integrity

The supplied roadmap and task text governed the phase gates. Gazebo entity state was used only by the Phase 2 test evaluator. Neither Nav2 nor a perception/controller node received Gazebo ground-truth pose or obstacle geometry. The Nav2 run is labeled REFERENCE because it used the packaged map and simulated laser scan; it is not evidence of RGB-only navigation. All custom vehicle dimensions are design placeholders, not measured hardware values.

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

The robot has a box chassis, two driven wheels, a passive spherical caster, a camera frame placeholder, Gazebo DiffDrive, wheel odometry, and a single base TF chain. The test world loads Gazebo Harmonic's Contact system. The contact endpoint is advertised, but no contact messages were observed in the trial.

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
- Gazebo Harmonic contact sensor and system plugin: https://gazebosim.org/docs/harmonic/sensors/

## Next recommended task

Proceed to Phase 3 sensor simulation: add RGB image and CameraInfo, IMU, bridge configuration, and timestamp/TF frequency checks. Keep the profiles named REFERENCE and REALISTIC, keep Gazebo truth evaluator-only, and retain FREE/OCCUPIED/UNKNOWN semantics. Before using contact events as a collision metric, diagnose why the configured contact sensor emitted no messages. Before claiming odometry accuracy under contact or slip, define and test a wheel-slip model.
