#!/usr/bin/env python3
"""Offline tests for runtime publisher-audit parsing."""
import sys
import unittest
import math
from unittest.mock import patch
from types import SimpleNamespace
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from phase8c_navigation_runner import Eval

class PublisherAuditTests(unittest.TestCase):
    def test_incremental_goals_remain_anchored_at_original_known_start(self):
        node=Eval.__new__(Eval);node.fixed_goal_map_reference=(0,0,0)
        self.assertEqual(node.goal_map_pose((1,0,0),(0,0,0),(2,0,0)),[2,0,0])

    def test_rotated_known_start_mapping_preserves_destination(self):
        node=Eval.__new__(Eval);node.fixed_goal_map_reference=(10,20,math.pi/2)
        p=node.goal_map_pose((11,21,0),(-3,0,0),(3,0,0))
        self.assertAlmostEqual(p[0],10);self.assertAlmostEqual(p[1],26)

    def test_live_audit_uses_discovered_graph_without_blocking_cli(self):
        node=Eval.__new__(Eval)
        with patch.object(Eval,'get_publishers_info_by_topic',return_value=[SimpleNamespace(node_name='anc_safety_command_guard')]), \
                patch('phase8c_navigation_runner.subprocess.run',side_effect=AssertionError('must not block callbacks')):
            result=node.publisher_count('/cmd_vel')
        self.assertTrue(Eval.is_sole_guard_publisher(result))
        self.assertEqual(result['audit_method'],'rclpy_graph')

    def test_accepts_only_guard_as_single_cmd_vel_publisher(self):
        text = ("Publisher count: 1\nNode name: anc_safety_command_guard\n"
                "Node namespace: /\nSubscription count: 1\nNode name: another_node\n")
        info = Eval.parse_publisher_info(text)
        self.assertEqual(info["count"], 1)
        self.assertEqual(info["publisher_names"], ["anc_safety_command_guard"])
        self.assertTrue(Eval.is_sole_guard_publisher(info))

    def test_rejects_competing_or_unrecognized_publishers(self):
        multi = "Publisher count: 2\nNode name: anc_safety_command_guard\nNode name: bypass\nSubscription count: 0\n"
        wrong = "Publisher count: 1\nNode name: nav2_controller\nSubscription count: 0\n"
        empty = "Publisher count: 0\nSubscription count: 0\n"
        self.assertFalse(Eval.is_sole_guard_publisher(Eval.parse_publisher_info(multi)))
        self.assertFalse(Eval.is_sole_guard_publisher(Eval.parse_publisher_info(wrong)))
        self.assertFalse(Eval.is_sole_guard_publisher(Eval.parse_publisher_info(empty)))

if __name__ == "__main__":
    unittest.main(verbosity=2)
