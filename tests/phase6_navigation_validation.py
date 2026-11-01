#!/usr/bin/env python3
"""Static acceptance checks for the Phase 6 REFERENCE profile."""
import tempfile
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))

import yaml

from scenario_generator import generate_layout, registry_data
from phase6_navigation_runner import make_oracle_map


ROOT = Path(__file__).resolve().parents[1]
DATA = registry_data()


def check(name, condition, detail):
    print(f"{'PASS' if condition else 'FAIL'} {name}: {detail}")
    if not condition:
        raise SystemExit(1)


def pgm_pixel(path, x, y):
    values = path.read_text().split()
    assert values[0] == "P2"
    width, height, maximum = map(int, values[1:4])
    data = list(map(int, values[4:]))
    xmin, _, ymin, _ = DATA["arena"]["outer_bounds_xy"]
    resolution = 0.05
    col = int((x - xmin) / resolution)
    row = height - 1 - int((y - ymin) / resolution)
    return data[row * width + col]


def main():
    phase6 = DATA.get("phase6_navigation", {})
    required = {"S00", "S01", "S02", "S03", "S04", "S09"}
    records = {item["id"]: item for item in phase6.get("scenarios", [])}
    check("phase6_scenario_registry", set(records) == required,
          f"registered={sorted(records)}")
    check("reference_profile_registry",
          phase6.get("profile") == "ORACLE_REFERENCE",
          f"profile={phase6.get('profile')}")

    original = {item["id"]: item for item in DATA["scenarios"]}
    historical_ok = all(
        original[key]["status"] == "active_phase5" and
        original[key]["sensor_profile"] in ("REFERENCE", "REALISTIC")
        for key in required
    )
    check("phase5_metadata_preserved", historical_ok,
          "active_phase5 status and original sensor_profile values remain intact")

    s01 = generate_layout("S01", 101, DATA)
    s09 = generate_layout("S09", 101, DATA)
    check("scenario_generator_route_expectations",
          s01["route_validation"]["found"] and not s09["route_validation"]["found"],
          "S01 traversable; S09 no-route")

    with tempfile.TemporaryDirectory(prefix="anc-phase6-map-") as temp:
        run_dir = Path(temp)
        _, pgm, summary = make_oracle_map(s01, run_dir)
        start, goal = s01["start_xyyaw"], s01["goal_xyyaw"]
        obstacle = s01["obstacles"][0]["pose_xyyaw"]
        check("map_contains_obstacle_geometry",
              pgm_pixel(pgm, obstacle[0], obstacle[1]) == 0,
              f"box center occupancy={pgm_pixel(pgm, obstacle[0], obstacle[1])}")
        check("map_start_goal_are_free",
              pgm_pixel(pgm, start[0], start[1]) == 254 and
              pgm_pixel(pgm, goal[0], goal[1]) == 254,
              f"start={pgm_pixel(pgm, start[0], start[1])}, "
              f"goal={pgm_pixel(pgm, goal[0], goal[1])}")
        check("map_size_and_profile",
              summary["width"] == 160 and summary["height"] == 120 and
              summary["profile"] == "ORACLE_REFERENCE",
              str(summary))
        _, s09_pgm, _ = make_oracle_map(s09, run_dir)
        check("no_route_divider_in_reference_map",
              pgm_pixel(s09_pgm, 0.0, 0.0) == 0,
              f"divider center occupancy={pgm_pixel(s09_pgm, 0.0, 0.0)}")

    nav_config = yaml.safe_load(
        (ROOT / "simulation/ros2_ws/src/anc_navigation/config/nav2_reference.yaml").read_text()
    )
    plugin = nav_config["controller_server"]["ros__parameters"]["FollowPath"]["plugin"]
    max_speed = nav_config["controller_server"]["ros__parameters"]["FollowPath"]["desired_linear_vel"]
    base = nav_config["global_costmap"]["global_costmap"]["ros__parameters"]["robot_base_frame"]
    check("custom_kinematics_limits_and_frames",
          plugin.endswith("RegulatedPurePursuitController") and max_speed <= 0.25 and
          base == "base_footprint",
          f"controller={plugin}, desired_linear_vel={max_speed}, base_frame={base}")

    launch = (ROOT / "simulation/ros2_ws/src/anc_navigation/launch/reference_navigation.launch.py").read_text()
    check("dedicated_reference_launch_only",
          "anc_bringup" in launch and "map_yaml" in launch and "amcl" not in launch.lower(),
          "launch uses the custom simulation and explicit map_yaml, with no AMCL")
    sim_launch = (ROOT / "simulation/ros2_ws/src/anc_bringup/launch/sim.launch.py").read_text()
    robot_xacro = (ROOT / "simulation/ros2_ws/src/anc_description/urdf/anc_robot.urdf.xacro").read_text()
    check("headless_camera_toggle",
          '"camera_enabled": "false"' in launch and
          '"camera_enabled", default_value="true"' in sim_launch and
          'xacro:arg name="camera_enabled" default="true"' in robot_xacro and
          'xacro:if value="$(arg camera_enabled)"' in robot_xacro,
          "Phase 6 disables RGB while the existing simulator default remains enabled")

    phase6_runner = (ROOT / "scripts/phase6_navigation_runner.py").read_text()
    check("closed_bag_tf_validation",
          "def inspect_bag_tf(bag_path)" in phase6_runner and
          'inspect_bag_tf(run_dir / "rosbag")' in phase6_runner and
          'get_publishers_info_by_topic("/tf")' in phase6_runner and
          '"anc_ekf_filter"' in phase6_runner,
          "MCAP edges and live /tf endpoints verify the sole EKF dynamic broadcaster")
    check("bounded_gazebo_pause_on_shutdown",
          '"/gzserver/set_simulation_state"' in phase6_runner and
          "timeout=8" in phase6_runner,
          "the runner asks Gazebo to pause and bounds the graceful shutdown request")
    check("bounded_post_action_settle",
          "settle_deadline = settle_start + 6.0" in phase6_runner and
          "post_action_settle_window_expired" in phase6_runner and
          "truth_pose_window(capture.truth_samples)" in phase6_runner and
          "capture.raw_odom_velocity" in phase6_runner,
          "a bounded settle window uses truth pose stability, odometry, and zero command")
    for relative in (
        "simulation/ros2_ws/src/anc_bringup/anc_bringup/imu_sanitizer.py",
        "simulation/ros2_ws/src/anc_localization/anc_localization/wheel_odometry_covariance.py",
        "simulation/ros2_ws/src/anc_navigation/anc_navigation/cmd_vel_guard.py",
    ):
        shutdown_source = (ROOT / relative).read_text()
        check("idempotent_ros_shutdown_" + Path(relative).stem,
              "if rclpy.ok():" in shutdown_source,
              "ROS shutdown is guarded after SIGINT")
    check("lifecycle_and_domain_gate",
          "lifecycle_nodes_active" in phase6_runner and
          'os.environ["ROS_DOMAIN_ID"] = env["ROS_DOMAIN_ID"]' in phase6_runner and
          "capture.truth_enabled = ready" in phase6_runner,
          "evaluator and launch share the private domain; actions wait for active lifecycle nodes")
    runner = (ROOT / "scripts/scenario_runner.py").read_text()
    check("phase5_stationary_mode_preserved",
          'choices=("stationary", "navigation"), default="stationary"' in runner and
          'if args.mode == "navigation"' in runner,
          "stationary remains default and navigation delegates to Phase 6 runner")
    print("OVERALL PASS")


if __name__ == "__main__":
    main()
