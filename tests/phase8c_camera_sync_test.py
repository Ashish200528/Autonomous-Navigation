#!/usr/bin/env python3
"""CameraInfo pairing tests for the shared RGB local map."""
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"simulation/ros2_ws/src/anc_realistic_navigation"))
from anc_realistic_navigation.mapper_node import nearest_camera_info_record


class CameraInfoPairingTests(unittest.TestCase):
    def test_selects_matching_older_info_when_newer_info_arrives_first(self):
        records = [
            {"stamp": 20.0, "wall": 1.0, "message": "matching-info"},
            {"stamp": 20.2, "wall": 1.1, "message": "next-frame-info"},
        ]
        self.assertEqual(nearest_camera_info_record(records, 20.0)["message"],
                         "matching-info")

    def test_uses_latest_info_when_no_mask_frame_exists_yet(self):
        records = [
            {"stamp": 20.0, "wall": 1.0, "message": "old"},
            {"stamp": 20.2, "wall": 1.1, "message": "new"},
        ]
        self.assertEqual(nearest_camera_info_record(records, None)["message"], "new")

    def test_empty_history_is_missing(self):
        self.assertIsNone(nearest_camera_info_record([], 20.0))


if __name__ == "__main__":
    unittest.main(verbosity=2)
