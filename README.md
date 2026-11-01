# Autonomous Navigation Car Simulation

Independent simulation project for the Autonomous Navigation Car. The existing Arduino UNO Q application is out of scope and was not changed.

## Mission and constraints

Navigate an open indoor arena with arbitrary obstacles from a start pose to a map-frame goal, replan when needed, stop safely when required inputs are stale or uncertain, and park within a measured tolerance. The realistic profile must not use simulator truth, perfect depth, hidden obstacle locations, or an ideal range feed as controller inputs.

All hardware measurements remain unresolved. The Phase 2 robot geometry and Phase 3 sensor parameters are explicitly documented placeholders. A provisional RGB camera and IMU now publish ROS interfaces; camera calibration/noise and IMU noise/bias are not hardware-measured. REFERENCE and REALISTIC profiles must stay separate, and FREE, OCCUPIED, and UNKNOWN must remain distinct.

## Repository layout

- simulation/ros2_ws/src/ contains the custom robot description, Gazebo world, and ROS bringup packages, including the Phase 3 camera, IMU, joint-state and contact interfaces.
- simulation/scenarios/ contains the scenario registry and future seeded worlds.
- simulation/data/ is for future logs and calibration artifacts; review large raw files before committing.
- hardware/uno_q/ is reserved for future adapters; keep the existing UNO Q application intact.
- scripts/ contains project-scoped ROS setup and repeatable evaluation scripts.
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

Build the three implemented packages:

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

## Current status

- Phase 0 baseline: complete and tagged v0.1-phase0.
- Phase 1 environment and REFERENCE Nav2 goal: PASS.
- Phase 2 custom differential-drive motion and wall stop: PASS WITH LIMITS.
- Phase 3 sensor simulation and ROS interfaces: PASS WITH LIMITS; see the phase report for provisional sensor assumptions and odometry/contact constraints.
- Phase 4 perception/navigation, unknown-pose acquisition, and parking state machine have not been started.
