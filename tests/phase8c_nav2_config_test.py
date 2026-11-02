#!/usr/bin/env python3
"""Static integration tests for live SLAM map and guarded Nav2 topics."""
from pathlib import Path
import unittest
import yaml

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "simulation/ros2_ws/src/anc_lidar"
NAV2 = yaml.safe_load((SRC / "config/nav2_lidar.yaml").read_text())
NAV2_LAUNCH = (SRC / "launch/nav2_lidar.launch.py").read_text()
PROFILE_LAUNCH = (SRC / "launch/realistic_rgb_lidar.launch.py").read_text()


def assert_unique_yaml_keys(node, path="$"):
    if isinstance(node, yaml.MappingNode):
        names = [key.value for key, _ in node.value if isinstance(key, yaml.ScalarNode)]
        if len(names) != len(set(names)):
            raise AssertionError(f"duplicate YAML mapping key at {path}")
        for key, value in node.value:
            assert_unique_yaml_keys(value, f"{path}.{key.value}")
    elif isinstance(node, yaml.SequenceNode):
        for index, value in enumerate(node.value):
            assert_unique_yaml_keys(value, f"{path}[{index}]")


class Nav2LidarConfigTests(unittest.TestCase):
    def test_profile_keeps_simulation_headless_and_navigation_opt_in(self):
        self.assertIn('DeclareLaunchArgument("enable_slam", default_value="false"', PROFILE_LAUNCH)
        self.assertIn('DeclareLaunchArgument("enable_nav2", default_value="false"', PROFILE_LAUNCH)
        self.assertIn('"rotation_calibration_max_radps", default_value="0.0"', PROFILE_LAUNCH)
        self.assertIn('"gui": LaunchConfiguration("gui")', PROFILE_LAUNCH)
    def test_nav2_yaml_has_no_duplicate_mapping_keys(self):
        assert_unique_yaml_keys(yaml.compose((SRC / "config/nav2_lidar.yaml").read_text()))
    def test_global_costmap_uses_live_sensor_map_and_unknown_is_blocked(self):
        params = NAV2["global_costmap"]["global_costmap"]["ros__parameters"]
        self.assertFalse(params["rolling_window"])
        self.assertIn("static_layer", params["plugins"])
        self.assertEqual(params["static_layer"]["map_topic"], "/map")
        self.assertTrue(params["static_layer"]["subscribe_to_updates"])
        self.assertTrue(params["track_unknown_space"])
        self.assertFalse(NAV2["planner_server"]["ros__parameters"]
                         ["GridBased"]["allow_unknown"])

    def test_nav2_footprint_matches_robot_geometry(self):
        polygon = NAV2["local_costmap"]["local_costmap"]["ros__parameters"]["footprint"]
        self.assertIn("0.215", polygon)
        self.assertIn("0.1525", polygon)
        self.assertIn("-0.16", polygon)
        self.assertIn("0.1525", NAV2["global_costmap"]["global_costmap"]
                      ["ros__parameters"]["footprint"])

    def test_motion_topics_form_one_guarded_chain(self):
        smoother = NAV2["velocity_smoother"]["ros__parameters"]
        collision = NAV2["collision_monitor"]["ros__parameters"]
        self.assertIn('("cmd_vel", "/cmd_vel_nav_raw")', NAV2_LAUNCH)
        self.assertEqual(smoother["cmd_vel_in_topic"], "/cmd_vel_nav_raw")
        self.assertEqual(smoother["cmd_vel_out_topic"], "/cmd_vel_smoothed")
        self.assertEqual(collision["cmd_vel_in_topic"], "/cmd_vel_smoothed")
        self.assertEqual(collision["cmd_vel_out_topic"], "/cmd_vel_nav")
        self.assertIn('"requested_topic": "/cmd_vel_nav"', PROFILE_LAUNCH)
        self.assertIn('"output_topic": "/cmd_vel"', PROFILE_LAUNCH)
        self.assertNotEqual(collision["cmd_vel_out_topic"], "/cmd_vel")

    def test_low_speed_and_turn_outputs_remain_bounded(self):
        velocity = NAV2["velocity_smoother"]["ros__parameters"]["max_velocity"]
        controller = NAV2["controller_server"]["ros__parameters"]["FollowPath"]
        self.assertLessEqual(velocity[0], 0.12)
        self.assertLessEqual(velocity[2], 0.10)
        self.assertLessEqual(controller["max_angular_vel"], 0.10)
        self.assertTrue(controller["use_collision_detection"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
