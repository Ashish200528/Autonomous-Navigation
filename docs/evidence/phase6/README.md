# Phase 6 evidence: REFERENCE Nav2 navigation

## Result

Overall status: PASS WITH LIMITS. Six selected deterministic scenarios passed their declared gates. S00-S04 returned NavigateToPose SUCCEEDED with error code 0. S09 returned ABORTED with error code 208, no timeout, and the expected no-route classification.

| Scenario | Seed | Status | Run ID | Path | Min clearance | Collisions |
|---|---:|---|---|---:|---:|---:|
| S00 | 101 | PASS | s00-reference-final-stop-check | 1.853 m | 1.7326 m | 0 |
| S01 | 101 | PASS | s01-reference-final-stop-check | 6.141 m | 0.1129 m | 0 |
| S02 | 101 | PASS | s02-reference-final-stop-check | 6.644 m | 0.1190 m | 0 |
| S03 | 101 | PASS | s03-reference-final-stop-check | 9.086 m | 0.0155 m | 0 |
| S04 | 202 | PASS | s04-reference-final-stop-check | 6.110 m | 0.1065 m | 0 |
| S09 | 101 | PASS_EXPECTED_NO_ROUTE | s09-reference-seed101-pose-odom-stop | 0.456 m | 0.5128 m | 0 |

## Evidence and checks

The final-code run directories preserve result.json, per-run SDF/manifest/map, truth trajectory, navigation trace, costmaps when available, rosbag logs, MCAP, and shutdown state. Earlier failed diagnostic attempts remain under runs/ and are not counted above.

- build-final.log: 5 ROS packages built successfully.
- static-validation.log: Python compile and 19 static acceptance checks passed.
- xacro-camera-toggle.log: camera-off has zero RGB sensor elements; camera-on has one; the IMU remains present in both.
- phase6-summary.json: machine-readable results and limitations.
- TF validation uses closed-bag transforms plus the live /tf publisher graph. It verifies one known-start map->odom edge and one configured EKF endpoint for odom->base_footprint; robot_state_publisher is the only other /tf publisher and publishes robot-link frames.
- The final S01 run required launch-managed SIGTERM/SIGKILL escalation for its Gazebo child, although the runner process group exited and the process scan was clear.

## Limits

The occupancy map is generated from exact scenario geometry and supplied to Nav2. This does not test RGB perception, unknown maps, unknown starts, dynamic obstacles, sensor-loss recovery, parking, or hardware emergency stop. S03 passed without contacts but had only 0.0155 m conservative clearance. S09 traveled about 0.456 m before Nav2 aborted the unreachable goal after 15 recoveries; truth pose was stable across 0.986 s, odometry was near zero, and command topics were zero. Gazebo's reported truth angular velocity disagreed with those signals and is flagged in the final run result. These are reference-simulation findings, not hardware-safe margins.
