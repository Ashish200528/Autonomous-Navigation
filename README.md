# Autonomous Navigation Car Simulation

Independent simulation project for the Autonomous Navigation Car. The existing Arduino UNO Q application is out of scope and was not changed.

## Mission and constraints

Navigate an open indoor arena with arbitrary obstacles from a start pose to a map-frame goal, replan when needed, stop safely when required inputs are stale or uncertain, and park within a measured tolerance. The realistic profile must not use simulator truth, perfect depth, hidden obstacle locations, or an ideal range feed as controller inputs.

All hardware measurements remain unresolved. The Phase 2 robot geometry and Phase 3 sensor parameters are explicitly documented placeholders. A provisional RGB camera and IMU now publish ROS interfaces; camera calibration/noise and IMU noise/bias are not hardware-measured. REFERENCE and REALISTIC profiles must stay separate, and FREE, OCCUPIED, and UNKNOWN must remain distinct.

## Repository layout

- simulation/ros2_ws/src/ contains the custom robot description, Gazebo world, ROS bringup packages, and the Phase 4 anc_localization package.
- simulation/scenarios/ contains the scenario registry and six generated Phase 5 SDF worlds.
- simulation/data/ is for future logs and calibration artifacts; review large raw files before committing.
- hardware/uno_q/ is reserved for future adapters; keep the existing UNO Q application intact.
- scripts/ contains project-scoped ROS setup, Phase 2-4 checks, the seeded Phase 5 harness, and the Phase 6 Nav2 reference runner.
- config/robot_parameters.yaml is the source of truth for measured and provisional model parameters.

## Start a project shell

From WSL:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
~~~

The script sources ROS 2 Jazzy only in the current terminal and does not alter the user's global shell startup file.

## Phase 1 REFERENCE navigation test

The packaged TurtleBot3/Nav2 sample was run headlessly with an early AMCL initial pose and a map-frame goal. The saved result is in docs/evidence/nav2-reference-trial-summary.json. This uses the packaged static map and simulated scan; it is not an RGB-only result. The action returned SUCCEEDED with error code 0 and final AMCL XY error 0.2737 m against the trial's 0.30 m acceptance check. The bag is partial; the action log and live postflight snapshot support completion.

## Phase 2 custom robot

Build the ROS packages needed by anc_bringup (four packages after Phase 4):

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install --packages-up-to anc_bringup
source install/setup.bash
~~~

Launch the headless Gazebo world in one WSL terminal:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
ros2 launch anc_bringup sim.launch.py
~~~

Run the motion and wall-stop checks from another terminal while the launch remains active:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
python3 scripts/phase2_motion_test.py
~~~

The final run passed forward, reverse, in-place rotation, odometry, stability, TF, and geometry-based wall-stop checks. The robot center stopped at x=0.735 m, the expected contact plane for the current placeholder footprint. Gazebo's contact topic emitted zero messages, so collision evidence comes from the evaluator-only pose and near-zero velocity. Odometry also advanced while the robot was stalled at the wall. See docs/PHASE_REPORT.md and docs/evidence/README.md for values and limitations.

## Phase 3 sensor interface test

Build the ROS workspace as shown in the Phase 2 section, then launch Gazebo headlessly. In another WSL terminal run:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
python3 scripts/phase3_sensor_smoke_test.py
~~~

The repeatable test checks rendered RGB content, CameraInfo calibration, IMU frame/rate/covariance and unavailable orientation, odometry, /clock, TF and wheel joints. It commands the robot toward the fixed test wall, verifies wall-specific contact messages, and records evaluator-only pose solely inside the test. It writes JSON and CSV evidence under docs/evidence/. A project RViz view is available with rviz2 -d config/phase3_sensor_view.rviz.

The accepted run measured a 15.15 Hz camera, 100 Hz IMU, 29.41 Hz odometry against the configured 30 Hz, and 2,330 wall-contact messages/records. At wall stall, wheel odometry exceeded evaluator pose by 0.46115 m. See docs/PHASE_REPORT.md and docs/evidence/README.md.

## Phase 4 localization

The known-start baseline uses the robot_localization EKF to publish /odometry/filtered. A covariance adapter relays Gazebo DiffDrive /odom to /odometry/wheel; the EKF fuses wheel vx/vy and sanitized /imu/data angular velocity z. The sanitized IMU has no absolute orientation. The filter never subscribes to raw Gazebo pose or evaluator truth.

Gazebo no longer publishes odom -> base_footprint on /tf; the EKF owns that dynamic transform. A known-start static map -> odom transform completes map -> odom -> base_footprint -> base_link. Unknown-pose localization is a later experiment.

Normal run, using default wheel friction:

~~~bash
# Terminal 1
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
ros2 launch anc_bringup sim.launch.py

# Terminal 2
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
python3 scripts/phase4_localization_test.py --mode normal --run-id phase4-localization-normal
~~~

Low-friction slip run (restart Gazebo with test friction):

~~~bash
ros2 launch anc_bringup sim.launch.py wheel_surface_mu:=0.001
python3 scripts/phase4_localization_test.py --mode slip --run-id phase4-localization-slip --slip-mu 0.001 --slip-speed 0.25 --slip-duration 2
~~~

The accepted headless run reduced motion XY RMSE from 0.1501 m raw to 0.0481 m filtered and yaw RMSE from 0.2845 rad to 0.0153 rad. At the wall stall, filtered longitudinal error was 0.020 m versus 0.390 m raw. Fusion did not improve the rectangle return endpoint and did not correct wheel slip: at mu=0.001, evaluator chassis displacement was 0 m while wheel and filtered odometry each advanced about 0.47 m. Covariances and model parameters remain provisional. See docs/PHASE_REPORT.md and docs/evidence/README.md.

## Phase 5 reproducible worlds and scenario harness

Phase 5 adds six light, bounded indoor SDF worlds for roadmap IDs S00, S01, S02, S03, S04, and S09. The generator reads simulation/scenarios/scenario_registry.yaml, accepts an integer seed, validates the known start and goal against the robot footprint and arena, checks geometry and a 10 cm footprint-inflated grid route, then writes a fresh world, manifest, and layout plot.

Build the project packages and run static geometry/SDF checks:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install --packages-up-to anc_bringup
cd ../..
python3 scripts/phase5_scenario_test.py
~~~

Run a headless scenario trial; the runner launches Gazebo, spawns the robot at the seeded known start, checks camera/IMU/odometry/clock and TF, records evaluator-only Gazebo truth and entity names locally, then sends SIGINT and waits for its process group to exit:

~~~bash
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
python3 scripts/scenario_runner.py --scenario S02 --seed 101 --run-id s02-seed101-repeat1
~~~

Each run writes world.sdf, manifest.json, layout.png, launch.log, and result.json under docs/evidence/phase5/runs/<run-id>/. The runner performs no navigation and publishes no truth or obstacle layout to ROS. The two S02 seed-101 trials, a nonzero-start S04 trial, and S09 blocked-route trial are indexed in docs/evidence/README.md. This validates scenario construction and sensor/TF startup; it does not validate path planning, perception, obstacle avoidance, or safe stopping.

## Phase 7 RGB-only perception

Phase 7 adds a separate REALISTIC_RGB_ONLY profile and anc_perception package. The runtime node reads only /camera/image_raw, /camera/camera_info, and /clock (timestamp age only), and publishes a mono8 mask (FREE=0, OCCUPIED=100, UNKNOWN=255), confidence image, overlay, and health JSON. It does not read odometry, IMU, maps, depth, Gazebo entity state, or scenario manifests. The Phase 6 ORACLE_REFERENCE launch remains separate.

Build and run the bounded checks from Ubuntu 24.04 WSL:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install
cd ../..
python3 tests/phase7_perception_validation.py
python3 tests/phase7_sensor_fault_test.py
python3 scripts/phase7_rgb_dataset.py --run-id phase7-replay-unique --scenarios S00,S01,S02,S03,S04,P7C1
~~~

Use a new run ID for each dataset replay. Evidence is indexed in docs/evidence/README.md; projection assumptions and the runtime/evaluator boundary are in docs/phase7-perception-architecture.md.

The accepted 12-view run recorded zero false-FREE pixels in evaluator-known occupied regions and 88.0% useful free coverage on known floor, with 100% occupied precision but 26.7% occupied recall. UNKNOWN covered 81.0% of all pixels. P7C1 failed closed to all UNKNOWN when calibrated floor support was not observable. Assessment: PASS WITH LIMITS. The method is tied to simulated floor appearance and a 0.50-0.60 m ground band; it provides no arbitrary obstacle depth, navigation, or obstacle avoidance.

## Phase 8 REALISTIC_RGB_ONLY stop-only baseline

Phase 8 adds a conservative base-relative traversability grid and a single safe-stop supervisor. The feasibility gate is BLOCKED for motion: the 0.50-0.60 m ground band projects to a roughly 0.09 m free patch behind a 0.389 m UNKNOWN gap, and hard-stop distance is not measured. The accepted behavior is STOP. Nine bounded live tests passed: all 1,052 actual /cmd_vel samples were zero and evaluator displacement stayed below 0.000011 m. This is not goal-reaching navigation.

The Phase 8 launch starts without a known-start map-to-odom transform, Nav2, or a reference map. See docs/phase8-feasibility.md, docs/roadmap-status.md, and docs/evidence/phase8/README.md.

Build and launch in WSL:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch anc_realistic_navigation phase8_realistic.launch.py
~~~

Reproduce static checks with python3 tests/phase8_safety_validation.py. The live harness is tests/phase8_live_safety_test.py; each run uses a unique --run-id and records JSON, CSV, launch logs, resource samples, and evaluator-only truth under docs/evidence/phase8/runs/.

## Current status

- Phase 0 baseline: complete and tagged v0.1-phase0.
- Phase 1 environment and REFERENCE Nav2 goal: PASS.
- Phase 2 custom differential-drive motion and wall stop: PASS WITH LIMITS.
- Phase 3 sensor simulation and ROS interfaces: PASS WITH LIMITS; see the phase report for provisional sensor assumptions and odometry/contact constraints.
- Phase 4 known-start wheel/IMU localization: PASS WITH LIMITS.
- Phase 5 reproducible worlds and scenario harness: PASS WITH LIMITS; see docs/PHASE_REPORT.md and docs/evidence/README.md.
- Phase 6 selected ORACLE_REFERENCE Nav2 scenarios S00-S04 and S09: PASS WITH LIMITS; see docs/PHASE_REPORT.md and docs/evidence/phase6/README.md.
- Phase 7 RGB-only FREE/OCCUPIED/UNKNOWN perception: PASS WITH LIMITS; see docs/PHASE_REPORT.md and docs/phase7-perception-architecture.md.
- Phase 8 local RGB map and zero-command safe-stop baseline: PASS WITH LIMITS. Autonomous motion and goal-reaching are BLOCKED by the feasibility gate; see docs/phase8-feasibility.md.
