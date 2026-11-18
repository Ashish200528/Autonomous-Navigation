#!/usr/bin/env python3
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "scripts"))
from phase8c_readiness import evaluate_readiness


def good_moving(trial_id):
    measurements = {
        "scan_messages": 12, "map_updates": 4, "truth_pose_samples": 8,
        "slam_pose_updates": 3, "map_to_odom_updates": 16,
        "map_free_cells": 80, "map_occupied_cells": 12,
        "truth_forward_displacement_m": 0.22,
        "slam_relative_displacement_error_m": 0.04,
        "slam_relative_yaw_error_rad": 0.02,
        "non_ground_contact_events": 0,
    }
    return {
        "trial_id": trial_id,
        "classification": "S00_BOUNDED_FORWARD_SLAM_AND_COMMAND_TIMEOUT_PROBE",
        "status": "PASS",
        "runtime_received_no_simulator_truth": True,
        "moving_slam_evaluation": {
            "status": "PASS", "failures": [], "measurements": measurements,
        },
    }


def good_turning():
    case_ids = {
        "clockwise", "anticlockwise", "multi_speed", "repeated_turns",
        "extended_rotation", "near_wall", "near_box", "narrow_passage",
        "low_friction", "lidar_loss", "localization_loss", "command_timeout",
        "unknown_swept_region",
    }
    cases = []
    for case_id in case_ids:
        moving = case_id not in {
            "lidar_loss", "localization_loss", "command_timeout", "unknown_swept_region"
        }
        cases.append({
            "case_id": case_id, "trial_pass": True,
            "center_drift_m": 0.02,
            "residual_yaw_after_zero_output_rad": 0.01,
            "zero_request_to_settled_wall_s": 0.50,
            "minimum_clearance_m": 0.20,
            "contact_records_non_ground": 0,
            "requested_angular_speed_radps": 0.05 if moving else 0.0,
            "measured_average_yaw_rate_radps": 0.04 if moving else 0.0,
            "safe_stop_observed": True,
        })
    return {
        "classification": "PHASE8C_PRODUCTION_TURNING_VALIDATION",
        "status": "PASS", "production_angular_authority_enabled": True,
        "cases": cases,
    }


def good_navigation():
    return {
        "classification": "REALISTIC_RGB_LIDAR_NAV2_GOAL_TRIAL",
        "status": "PASS", "scenario_id": "S00",
        "localization_mode": "known-start online SLAM", "route_expected": True,
        "runtime_navigation_receives_no_gazebo_truth": True,
        "nav2_action_status": "SUCCEEDED",
        "sole_authoritative_cmd_vel_publisher": True,
        "travel_limit_exceeded": False, "truth_path_length_m": 1.2,
        "final_xy_error_m": 0.1, "final_heading_error_rad": 0.05,
        "min_evaluator_clearance_m": 0.12,
        "nav2_lifecycle_states": {
            "planner_server": "active", "controller_server": "active",
            "behavior_server": "active", "bt_navigator": "active",
            "velocity_smoother": "active", "collision_monitor": "active",
        },
        "sensor_summary": {
            "scan_count": 100, "rgb_frames": 100, "map_updates": 20,
            "global_costmap_updates": 20, "plan_messages": 1,
        },
        "non_ground_contact_events": [],
        "sensor_integrity": {"rgb_fresh": True, "scan_fresh": True,
                             "map_fresh": True, "filtered_odom_received": True},
        "command_summary": {"/cmd_vel": {"count": 30}},
        "cmd_vel_publisher_audit": {"/cmd_vel": {
            "count": 1, "publisher_names": ["anc_safety_command_guard"]}},
    }


class Phase8CReadinessTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.config = {
            "prerequisites": {
                "production_turning_verified": True,
                "moving_slam_verified": True,
                "guarded_command_chain_verified": True,
            },
            "evidence": {},
        }

    def tearDown(self):
        self.temp.cleanup()

    def write_json(self, name, payload):
        path = self.root / name
        path.write_text(json.dumps(payload))
        return name

    def test_flags_and_file_presence_alone_never_open_gate(self):
        path = self.root / "empty.json"
        path.write_text("{}")
        self.config["evidence"] = {
            "production_turning_verified": "empty.json",
            "moving_slam_verified": ["empty.json"] * 3,
            "guarded_command_chain_verified": "empty.json",
        }
        result = evaluate_readiness(self.config, self.root)
        self.assertEqual(result["status"], "BLOCKED")
        self.assertTrue(result["flags_are_not_proof"])

    def test_calibration_only_rotation_is_not_production_turning(self):
        self.config["evidence"]["production_turning_verified"] = self.write_json(
            "rotation.json", {
                "classification": "BOUNDED_PURE_YAW_CALIBRATION_ONLY",
                "all_trials_pass": True,
            })
        result = evaluate_readiness(self.config, self.root)
        self.assertFalse(result["prerequisites"]["production_turning_verified"]["verified"])
        self.assertTrue(any("classification_mismatch" in error
                            for error in result["prerequisites"]["production_turning_verified"]["errors"]))

    def test_moving_slam_needs_three_distinct_measured_runs(self):
        path = self.write_json("run.json", good_moving("run-1"))
        self.config["evidence"]["moving_slam_verified"] = [path, path, path]
        result = evaluate_readiness(self.config, self.root)
        self.assertFalse(result["prerequisites"]["moving_slam_verified"]["verified"])
        self.assertTrue(any("duplicate_moving_trial_id" in e
                            for e in result["prerequisites"]["moving_slam_verified"]["errors"]))

    def test_valid_contents_can_open_gate_without_editing_cached_flags(self):
        self.config["prerequisites"] = {key: False for key in (
            "production_turning_verified", "moving_slam_verified",
            "guarded_command_chain_verified")}
        self.config["evidence"] = {
            "production_turning_verified": self.write_json("turn.json", good_turning()),
            "moving_slam_verified": [
                self.write_json("move-1.json", good_moving("run-1")),
                self.write_json("move-2.json", good_moving("run-2")),
                self.write_json("move-3.json", good_moving("run-3")),
            ],
            "guarded_command_chain_verified": self.write_json("nav.json", good_navigation()),
        }
        result = evaluate_readiness(self.config, self.root)
        self.assertEqual(result["status"], "READY")
        self.assertTrue(all(not item["declared_flag"]
                            for item in result["prerequisites"].values()))

    def test_navigation_needs_success_and_guard_graph_audit(self):
        nav = good_navigation()
        nav["cmd_vel_publisher_audit"]["/cmd_vel"]["publisher_names"] = ["controller_server"]
        nav["nav2_action_status"] = "ABORTED"
        nav["nav2_lifecycle_states"]["controller_server"] = "inactive"
        nav["sensor_summary"]["global_costmap_updates"] = 0
        path = self.write_json("nav.json", nav)
        self.config["evidence"]["guarded_command_chain_verified"] = path
        result = evaluate_readiness(self.config, self.root)
        errors = result["prerequisites"]["guarded_command_chain_verified"]["errors"]
        self.assertTrue(any("live_cmd_vel_graph_not_owned_by_guard" in error for error in errors))
        self.assertTrue(any("nav2_goal_not_succeeded" in error for error in errors))
        self.assertTrue(any("nav2_lifecycle_activation_unverified" in error
                            for error in errors))
        self.assertTrue(any("live_sensor_costmap_or_plan_updates_missing" in error
                            for error in errors))


    def test_brief_output_uses_real_tab_for_shell_launcher(self):
        config_path = self.root / "readiness.json"
        config_path.write_text(json.dumps(self.config))
        result = subprocess.run(
            [sys.executable, str(ROOT / "scripts" / "phase8c_readiness.py"),
             str(config_path), str(ROOT), "--brief"],
            check=True, capture_output=True, text=True)
        output = result.stdout.rstrip("\n")
        self.assertIn("\t", output)
        self.assertNotIn("\\t", output)
        status, reason = output.split("\t", 1)
        self.assertEqual(status, "BLOCKED")
        self.assertTrue(reason)

    def test_evidence_path_cannot_escape_project_root(self):
        self.config["evidence"]["production_turning_verified"] = "../outside.json"
        result = evaluate_readiness(self.config, self.root)
        self.assertIn("evidence_path_escapes_project",
                      result["prerequisites"]["production_turning_verified"]["errors"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
