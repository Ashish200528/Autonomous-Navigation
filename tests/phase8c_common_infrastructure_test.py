#!/usr/bin/env python3
"""Dependency-light tests for reusable contracts and conservative safety gate."""
import unittest
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "simulation/ros2_ws/src/anc_navigation_common"))
from anc_navigation_common.contracts import (
    FREE, OCCUPIED, UNKNOWN, GridGeometry, classify_range,
    fuse_occupancy, input_age_valid, validate_grid,
)
from anc_navigation_common.safety import guarded_command, speed_gate


class CommonContractTests(unittest.TestCase):
    def test_ros_occupancy_values_and_dimensions(self):
        geometry = GridGeometry(2, 2, 0.05, -0.1, -0.1, "base_link")
        self.assertEqual(validate_grid([FREE, OCCUPIED, UNKNOWN, 50], geometry),
                         [0, 100, -1, 50])
        with self.assertRaises(ValueError):
            validate_grid([FREE], geometry)

    def test_nonfinite_and_out_of_range_scan_values_are_unknown(self):
        for value in (float("nan"), float("inf"), -float("inf"), 0.05, 8.1):
            self.assertEqual(classify_range(value, 0.10, 8.0), ("UNKNOWN", None))
        self.assertEqual(classify_range(1.0, 0.10, 8.0), ("VALID", 1.0))

    def test_lidar_obstacles_cannot_be_overwritten_by_rgb(self):
        self.assertEqual(
            fuse_occupancy([OCCUPIED, FREE, UNKNOWN], [FREE, OCCUPIED, OCCUPIED]),
            [OCCUPIED, FREE, UNKNOWN])

    def test_optional_metric_rgb_free_only_fills_lidar_unknown(self):
        self.assertEqual(
            fuse_occupancy([OCCUPIED, UNKNOWN], [FREE, FREE],
                           allow_projected_rgb_free=True), [OCCUPIED, FREE])

    def test_sensor_age_rejects_future_and_stale(self):
        self.assertEqual(input_age_valid(1.0, 1.1, 0.2), (True, "fresh"))
        self.assertFalse(input_age_valid(2.0, 1.0, 0.2)[0])
        self.assertFalse(input_age_valid(0.0, 2.0, 0.2)[0])

    def test_speed_gate_uses_all_distance_terms(self):
        result = speed_gate(
            corridor_m=1.0, age_s=0.1, processing_latency_s=0.02,
            supervisor_period_s=0.05,
            stop_distance_by_speed=[(0.05, 0.001), (0.10, 0.004)],
            localization_allowance_m=0.1, footprint_margin_m=0.2,
            safety_margin_m=0.1, max_speed_mps=0.10)
        self.assertEqual(result["max_linear_speed_mps"], 0.10)
        self.assertAlmostEqual(result["selected"]["required_corridor_m"], 0.421)

    def test_gate_blocks_unmeasured_or_short_clearance(self):
        args = dict(age_s=0.1, processing_latency_s=0.02, supervisor_period_s=0.05,
                    localization_allowance_m=0.1, footprint_margin_m=0.1,
                    safety_margin_m=0.1, max_speed_mps=0.1)
        self.assertEqual(speed_gate(corridor_m=4.0, stop_distance_by_speed=[], **args)
                         ["max_linear_speed_mps"], 0.0)
        self.assertEqual(speed_gate(corridor_m=0.2,
                         stop_distance_by_speed=[(0.1, 0.01)], **args)
                         ["max_linear_speed_mps"], 0.0)


    def test_rotation_calibration_guard_allows_yaw_only_and_blocks_translation(self):
        state = {
            "valid": True, "authorized": True,
            "motion_mode": "rotation_calibration_only",
            "max_linear_speed_mps": 0.0,
            "max_angular_speed_radps": 0.05,
        }
        common = dict(request_age_s=0.01, safety_age_s=0.01,
                      safety_state=state, command_timeout_s=0.3,
                      safety_timeout_s=0.2,
                      hard_max_linear_speed_mps=0.18,
                      hard_max_angular_speed_radps=0.05)
        yaw_only = guarded_command(
            request={"linear_x_mps": 0.0, "angular_z_radps": 0.02}, **common)
        self.assertTrue(yaw_only["authorized"])
        self.assertEqual((yaw_only["linear_x_mps"], yaw_only["angular_z_radps"]),
                         (0.0, 0.02))

        combined = guarded_command(
            request={"linear_x_mps": 0.05, "angular_z_radps": 0.02}, **common)
        self.assertFalse(combined["authorized"])
        self.assertEqual(combined["reason"], "combined_motion_not_validated")
        self.assertEqual((combined["linear_x_mps"], combined["angular_z_radps"]),
                         (0.0, 0.0))

        translation = guarded_command(
            request={"linear_x_mps": 0.05, "angular_z_radps": 0.0}, **common)
        self.assertFalse(translation["authorized"])
        self.assertEqual(translation["reason"],
                         "rotation_calibration_translation_blocked")

        denied = guarded_command(
            request={"linear_x_mps": 0.0, "angular_z_radps": 0.02},
            **(common | {"safety_state": state | {"authorized": False}}))
        self.assertFalse(denied["authorized"])
        self.assertEqual(denied["reason"], "safety_authority_false")

    def test_guard_blocks_missing_stale_invalid_and_clamps(self):
        common = dict(request={"linear_x_mps": 0.18, "angular_z_radps": 0.5},
                      request_age_s=0.01, safety_age_s=0.01,
                      command_timeout_s=0.3, safety_timeout_s=0.2,
                      hard_max_linear_speed_mps=0.25,
                      hard_max_angular_speed_radps=0.6)
        self.assertFalse(guarded_command(**(common | {"safety_state": None}))["authorized"])
        state = {"valid": True, "max_linear_speed_mps": 0.1,
                 "max_angular_speed_radps": 0.2}
        self.assertFalse(guarded_command(**(common | {"safety_state": state,
                            "request_age_s": 0.5}))["authorized"])
        result = guarded_command(**(common | {"safety_state": state}))
        self.assertEqual((result["linear_x_mps"], result["angular_z_radps"]), (0.1, 0.2))
        self.assertEqual(result["reason"], "command_clamped_to_measured_gate")


if __name__ == "__main__":
    unittest.main(verbosity=2)
