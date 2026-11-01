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
