# Metrics and measurement rules

No results have been measured yet. Targets below are starting thresholds from the roadmap and are not achieved results.

| Metric | Definition | Initial rule |
|---|---|---|
| Goal success | Reached / attempted per scenario group | Report numerator, denominator and seeds |
| Collision count/rate | Gazebo contact events per run | Zero for accepted demo runs; retain failures |
| Minimum clearance | Minimum evaluator distance from robot footprint to obstacle | At least configured footprint margin |
| Path inefficiency | Actual path length / feasible reference path length | Report per run and sensor profile |
| Position error | Estimated pose versus evaluator-only truth | Truth available to evaluator only |
| Parking error | Final position and yaw error in map frame | Provisional ≤0.20 m and ≤10°; stationary ≥5 s |
| False-clear rate | Blocked ground classified as free | Report prominently; no hidden failures |
| False-blocked rate | Traversable ground classified as blocked | Report separately |
| Reaction delay | Input fault/frame time to stop or replan | Record distribution and fault condition |
| FPS/real-time factor | Camera frequency and Gazebo simulation factor | Report measured values; no assumed real-time performance |
| Safe-stop outcome | Correctly stops for UNKNOWN/stale/lost required input | Explicit pass/fail per scenario |

Experiment record schema: `run_id, seed, scenario_id, software_commit, sensor_profile, map_mode, start_x, start_y, start_yaw, goal_x, goal_y, goal_yaw, success, collision_count, elapsed_s, path_length_m, min_clearance_m, replans, final_position_error_m, final_yaw_error_deg, perception_fps, false_clear_count, false_blocked_count, outcome_reason, rosbag_path, video_path`.
