#!/usr/bin/env python3
"""Offline tests for runtime publisher-audit parsing."""
import sys
import unittest
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from phase8c_navigation_runner import Eval

class PublisherAuditTests(unittest.TestCase):
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
