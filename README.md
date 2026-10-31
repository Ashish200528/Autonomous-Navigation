# Autonomous Navigation Car Simulation

Independent simulation project for the Autonomous Navigation Car. The existing Arduino UNO Q application is out of scope and was not changed.

## Mission and constraints

Navigate an open indoor arena with arbitrary obstacles from a start pose to a map-frame goal, replan when needed, stop safely when required inputs are stale or uncertain, and park within a measured tolerance. The realistic profile must not use simulator truth, perfect depth, hidden obstacle locations, or an ideal range feed as controller inputs.

All hardware measurements remain unresolved. The Phase 2 model uses explicitly documented placeholders only. The camera is currently a TF frame placeholder with no simulated image; no IMU is implemented yet. REFERENCE and REALISTIC profiles must stay separate, and FREE, OCCUPIED, and UNKNOWN must remain distinct.

## Repository layout

- simulation/ros2_ws/src/ contains the implemented Phase 2 description, Gazebo, and bringup packages, plus future package slots.
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

## Current status

- Phase 0 baseline: complete and tagged v0.1-phase0.
- Phase 1 environment and REFERENCE Nav2 goal: PASS.
- Phase 2 custom differential-drive motion and wall stop: PASS WITH LIMITS.
- No Phase 3 sensors, realistic perception, custom Nav2 configuration, unknown-pose acquisition, or parking state machine has been implemented.
- No remote push was made. Current branch is main.
