# Scripts

source_ros_jazzy.sh is a project-scoped environment helper. Source it from the project root in each WSL shell; it does not change ~/.bashrc or settings for the separate Arduino UNO Q application.

Add future reproducible setup, environment sanity, and scenario runner scripts in the matching roadmap phase. Do not put unreviewed bulk system-install scripts here.

## Phase 6 REFERENCE Nav2 trials

The Phase 6 runner launches a bounded headless world, generates its per-run ORACLE_REFERENCE map from scenario geometry, records MCAP and evaluator evidence, then pauses the simulator and shuts down its ROS process group. The RGB camera is disabled only for this run profile. Run static checks first:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
source simulation/ros2_ws/install/setup.bash
python3 -m py_compile scripts/phase6_navigation_runner.py scripts/scenario_runner.py
python3 tests/phase6_navigation_validation.py
~~~

Use a new run ID for each trial. The direct Phase 6 command is:

~~~bash
python3 scripts/phase6_navigation_runner.py --scenario S01 --seed 101 --run-id s01-reference-detour-seed101
~~~

Or delegate through the scenario runner with --mode navigation; its default remains stationary:

~~~bash
python3 scripts/scenario_runner.py --scenario S01 --seed 101 --run-id s01-reference-detour-seed101 --mode navigation
~~~

Accepted Phase 6 run IDs and limitations are indexed in docs/evidence/phase6/README.md and docs/evidence/phase6/phase6-summary.json. This profile is not RGB perception or hardware validation.

## Phase 7 REALISTIC_RGB_ONLY perception

Build and run the checks from the repository root:

~~~bash
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install
cd ../..
python3 tests/phase7_perception_validation.py
python3 tests/phase7_sensor_fault_test.py
~~~

For a fixed-suite replay, use a new run ID to preserve previous evidence:

~~~bash
source simulation/ros2_ws/install/setup.bash
python3 scripts/phase7_rgb_dataset.py --run-id phase7-replay-unique --scenarios S00,S01,S02,S03,S04,P7C1
~~~

The runner starts bounded headless Gazebo cases, captures RGB and classifier outputs, queries simulator geometry only in the offline evaluator, records per-view metrics and the live ROS graph contract, then shuts down each process group. anc_perception/realistic_perception.launch.py is separate from Phase 6 ORACLE_REFERENCE; do not interpret its output as a costmap or autonomous navigation input. Accepted run and topic contract are indexed in docs/evidence/README.md.

## Phase 8 REALISTIC_RGB_ONLY stop-only tests

Build and run static geometry checks:

~~~bash
cd /home/ashish/AutonomousNavigationCar
source scripts/source_ros_jazzy.sh
cd simulation/ros2_ws
colcon build --symlink-install
cd ../..
python3 tests/phase8_safety_validation.py
~~~

The live harness launches the separate anc_realistic_navigation/phase8_realistic.launch.py, submits a nonzero request only to /cmd_vel/requested, then verifies the sole /cmd_vel publisher remains zero and checks evaluator-only pose. Use unique run IDs and scenario worlds; examples are indexed in docs/evidence/phase8/README.md. Phase 8 code cannot unlock movement. The correct result is STOP/BLOCKED; do not count these stationary checks as navigation.
