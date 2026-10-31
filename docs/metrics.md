# Metrics and measurement rules

Targets below are starting rules, not measured hardware achievements. Results are single-run observations and do not establish statistical performance.

| Metric | Definition | Initial rule |
|---|---|---|
| Goal success | Reached / attempted per scenario group | Report numerator, denominator, and seeds |
| Collision count/rate | Gazebo contact events per run | Zero for accepted demo runs; retain failures |
| Minimum clearance | Minimum evaluator distance from robot footprint to obstacle | At least configured footprint margin |
| Path inefficiency | Actual path length / feasible reference path length | Report per run and sensor profile |
| Position error | Estimated pose versus evaluator-only truth | Truth available to evaluator only |
| Parking error | Final position and yaw error in map frame | Provisional 0.20 m and 10 degrees; stationary at least 5 s |
| False-clear rate | Blocked ground classified as free | Report prominently; no hidden failures |
| False-blocked rate | Traversable ground classified as blocked | Report separately |
| Reaction delay | Input fault/frame time to stop or replan | Record distribution and fault condition |
| FPS/real-time factor | Camera frequency and Gazebo simulation factor | Report measured values; do not assume real-time performance |
| Safe-stop outcome | Correctly stops for UNKNOWN/stale/lost required input | Explicit pass/fail per scenario |

## Observed baseline results

### Phase 1 REFERENCE navigation

One headless TurtleBot3/Nav2 trial used a known map-frame initial pose and goal. The action returned SUCCEEDED with error code 0. The known-free corridor had 161/161 samples at 0.025 m spacing. Final AMCL XY error was 0.27373 m against the trial's 0.30 m check. The trial used the static map and simulated laser scan; it is not RGB-only evidence. Its MCAP ended before the last goal feedback.

### Phase 2 custom robot

One headless motion test passed all 10 required checks. Truth displacement was +0.30035 m forward, -0.29951 m reverse, and +1.10063 rad rotation. Corresponding odometry deltas were +0.30030 m, -0.30030 m, and +1.20120 rad. Maximum absolute roll/pitch was 0.00401 rad.

The robot stopped at the geometry-derived wall contact plane: expected center x=0.735 m, measured x=0.735 m, maximum sampled x=0.735025 m, and final evaluator forward velocity about -0.00000056 m/s. This demonstrates that the physics constraint stopped the robot without tunneling in this test. The ROS contact topic emitted zero messages, so contact-event count is unobserved rather than a verified zero-collision count.

Under the wall stall, odometry advanced to x=1.40039 m while evaluator pose remained at x=0.735 m. Wheel motion and world pose therefore diverged under contact/slip. Odometry accuracy under stall is not validated.

Experiment record schema: run_id, seed, scenario_id, software_commit, sensor_profile, map_mode, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, success, collision_count, elapsed_s, path_length_m, min_clearance_m, replans, final_position_error_m, final_yaw_error_deg, perception_fps, false_clear_count, false_blocked_count, outcome_reason, rosbag_path, video_path.
