#!/usr/bin/env python3
"""Unit tests for the proposal-only simulation parking controller."""
import math
import unittest
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation/ros2_ws/src/anc_navigation_common"))
from anc_navigation_common.parking import parking_step


class ParkingControllerTests(unittest.TestCase):
    def setUp(self):
        self.safety = {
            "valid": True, "authorized": True, "motion_mode": "forward_only",
            "max_linear_speed_mps": 0.06, "max_angular_speed_radps": 0.0,
        }

    def test_forward_approach_respects_live_gate_speed(self):
        result = parking_step(
            current_pose=(0.0, 0.0, 0.0), target_pose=(1.0, 0.0, 0.0),
            safety_state=self.safety, localization_valid=True,
            measured_stationary=False, stationary_duration_s=0.0)
        self.assertEqual(result["phase"], "APPROACH")
        self.assertEqual(result["linear_x_mps"], 0.06)
        self.assertEqual(result["angular_z_radps"], 0.0)
        self.assertFalse(result["authoritative_output"])

    def test_no_angular_authority_blocks_turn_instead_of_bypassing_guard(self):
        result = parking_step(
            current_pose=(0.0, 0.0, 0.0), target_pose=(0.0, 0.0, math.pi/2),
            safety_state=self.safety, localization_valid=True,
            measured_stationary=True, stationary_duration_s=0.0)
        self.assertEqual(result["phase"], "BLOCKED")
        self.assertEqual(result["reason"], "angular_authority_missing")
        self.assertEqual((result["linear_x_mps"], result["angular_z_radps"]), (0.0, 0.0))

    def test_calibration_only_gate_cannot_be_used_for_parking(self):
        result = parking_step(
            current_pose=(0.0, 0.0, 0.0), target_pose=(0.0, 0.0, math.pi/2),
            safety_state=self.safety | {
                "motion_mode": "rotation_calibration_only",
                "max_angular_speed_radps": 0.10,
            }, localization_valid=True, measured_stationary=True,
            stationary_duration_s=0.0)
        self.assertEqual(result["reason"], "parking_requires_production_motion_mode")
        self.assertEqual((result["linear_x_mps"], result["angular_z_radps"]), (0.0, 0.0))

    def test_sequential_controller_rotates_then_approaches_without_combined_motion(self):
        rotate = parking_step(
            current_pose=(0.0, 0.0, math.pi/2), target_pose=(1.0, 0.0, 0.0),
            safety_state=self.safety | {"max_angular_speed_radps": 0.05},
            localization_valid=True, measured_stationary=True, stationary_duration_s=0.0)
        self.assertEqual(rotate["phase"], "ALIGN_APPROACH")
        self.assertEqual(rotate["linear_x_mps"], 0.0)
        self.assertLessEqual(abs(rotate["angular_z_radps"]), 0.05)

        drive = parking_step(
            current_pose=(0.0, 0.0, 0.0), target_pose=(1.0, 0.0, 0.0),
            safety_state=self.safety | {"max_angular_speed_radps": 0.05},
            localization_valid=True, measured_stationary=False, stationary_duration_s=0.0)
        self.assertEqual(drive["phase"], "APPROACH")
        self.assertEqual(drive["angular_z_radps"], 0.0)

    def test_stationary_hold_must_meet_position_heading_and_five_seconds(self):
        kwargs = dict(
            current_pose=(0.10, 0.0, math.radians(5)),
            target_pose=(0.0, 0.0, 0.0), safety_state=self.safety,
            localization_valid=True, measured_stationary=True)
        before = parking_step(**kwargs, stationary_duration_s=4.99)
        self.assertEqual(before["phase"], "HOLD")
        self.assertFalse(before["complete"])
        done = parking_step(**kwargs, stationary_duration_s=5.0)
        self.assertEqual(done["phase"], "COMPLETE")
        self.assertTrue(done["complete"])

    def test_nonstationary_pose_never_completes_dwell(self):
        result = parking_step(
            current_pose=(0.0, 0.0, 0.0), target_pose=(0.0, 0.0, 0.0),
            safety_state=self.safety, localization_valid=True,
            measured_stationary=False, stationary_duration_s=8.0)
        self.assertEqual(result["phase"], "HOLD")
        self.assertFalse(result["complete"])

    def test_invalid_safety_or_localization_fails_closed(self):
        for kwargs in (
            {"localization_valid": False, "safety_state": self.safety},
            {"localization_valid": True, "safety_state": self.safety | {"valid": False}},
            {"localization_valid": True, "safety_state": self.safety | {"authorized": False}},
        ):
            result = parking_step(
                current_pose=(0.0, 0.0, 0.0), target_pose=(1.0, 0.0, 0.0),
                measured_stationary=True, stationary_duration_s=0.0, **kwargs)
            self.assertEqual((result["linear_x_mps"], result["angular_z_radps"]), (0.0, 0.0))
            self.assertFalse(result["complete"])


if __name__ == "__main__":
    unittest.main(verbosity=2)