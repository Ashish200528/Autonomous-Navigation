#!/usr/bin/env python3
# Evidence-content validator for the Phase 8C autonomous-navigation gate.
import argparse
import json
import math
import sys
from pathlib import Path

REQUIREMENTS = (
    "production_turning_verified",
    "moving_slam_verified",
    "guarded_command_chain_verified",
)
ROTATION_CASES = {
    "clockwise", "anticlockwise", "multi_speed", "repeated_turns",
    "extended_rotation", "near_wall", "near_box", "narrow_passage",
    "low_friction", "lidar_loss", "localization_loss", "command_timeout",
    "unknown_swept_region",
}
ROTATION_MOTION_CASES = ROTATION_CASES - {
    "lidar_loss", "localization_loss", "command_timeout", "unknown_swept_region"
}
ROTATION_FAULT_CASES = {
    "lidar_loss", "localization_loss", "command_timeout", "unknown_swept_region"
}
MOVING_CLASS = "S00_BOUNDED_FORWARD_SLAM_AND_COMMAND_TIMEOUT_PROBE"
NAV_CLASS = "REALISTIC_RGB_LIDAR_NAV2_GOAL_TRIAL"
TURN_CLASS = "PHASE8C_PRODUCTION_TURNING_VALIDATION"


def _finite(value):
    try:
        number = float(value)
        return math.isfinite(number)
    except (TypeError, ValueError, OverflowError):
        return False


def _number(value, label, errors, *, minimum=None, maximum=None):
    if not _finite(value):
        errors.append(label + "_missing_or_nonfinite")
        return None
    number = float(value)
    if minimum is not None and number < minimum:
        errors.append(label + "_below_limit")
    if maximum is not None and number > maximum:
        errors.append(label + "_above_limit")
    return number


def _validate_turn(payload):
    errors = []
    if payload.get("classification") != TURN_CLASS:
        errors.append("classification_mismatch")
    if payload.get("status") != "PASS":
        errors.append("status_not_PASS")
    if payload.get("production_angular_authority_enabled") is not True:
        errors.append("production_angular_authority_not_enabled_in_test")
    cases = payload.get("cases")
    if not isinstance(cases, list):
        errors.append("cases_missing")
        cases = []
    found = {}
    for case in cases:
        if not isinstance(case, dict) or not isinstance(case.get("case_id"), str):
            errors.append("case_record_invalid")
            continue
        case_id = case["case_id"]
        if case_id in found:
            errors.append("duplicate_case:" + case_id)
        found[case_id] = case
    for case_id in sorted(ROTATION_CASES - found.keys()):
        errors.append("required_case_missing:" + case_id)
    for case_id in sorted(ROTATION_CASES & found.keys()):
        case = found[case_id]
        if case.get("trial_pass") is not True:
            errors.append("case_failed:" + case_id)
        _number(case.get("center_drift_m"), case_id + ".center_drift_m",
                errors, maximum=0.10)
        _number(case.get("residual_yaw_after_zero_output_rad"),
                case_id + ".residual_yaw_after_zero_output_rad", errors,
                minimum=0.0, maximum=0.05)
        _number(case.get("zero_request_to_settled_wall_s"),
                case_id + ".zero_request_to_settled_wall_s", errors,
                minimum=0.0, maximum=0.75)
        _number(case.get("minimum_clearance_m"),
                case_id + ".minimum_clearance_m", errors, minimum=0.10)
        contacts = case.get("contact_records_non_ground")
        if isinstance(contacts, bool) or not isinstance(contacts, int) or contacts != 0:
            errors.append("non_ground_contact_or_missing:" + case_id)
        if case_id in ROTATION_MOTION_CASES:
            requested = _number(case.get("requested_angular_speed_radps"),
                                case_id + ".requested_angular_speed_radps", errors)
            measured = _number(case.get("measured_average_yaw_rate_radps"),
                               case_id + ".measured_average_yaw_rate_radps", errors)
            if requested is not None and measured is not None:
                if requested == 0.0 or requested * measured <= 0.0:
                    errors.append("rotation_direction_or_motion_invalid:" + case_id)
        if case_id in ROTATION_FAULT_CASES and case.get("safe_stop_observed") is not True:
            errors.append("fault_safe_stop_unverified:" + case_id)
    return errors


def _validate_moving(payload):
    errors = []
    if payload.get("classification") != MOVING_CLASS:
        errors.append("classification_mismatch")
    if payload.get("status") != "PASS":
        errors.append("status_not_PASS")
    if payload.get("runtime_received_no_simulator_truth") is not True:
        errors.append("runtime_truth_isolation_unverified")
    trial = payload.get("moving_slam_evaluation")
    if not isinstance(trial, dict):
        errors.append("moving_slam_evaluation_missing")
        return errors
    if trial.get("status") != "PASS" or trial.get("failures") != []:
        errors.append("moving_slam_measurement_contract_failed")
    measurements = trial.get("measurements")
    if not isinstance(measurements, dict):
        errors.append("moving_slam_measurements_missing")
        return errors
    for field, minimum in (
        ("scan_messages", 8), ("map_updates", 2), ("truth_pose_samples", 3),
        ("slam_pose_updates", 2), ("map_to_odom_updates", 2),
        ("map_free_cells", 1), ("map_occupied_cells", 1),
    ):
        value = measurements.get(field)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or value < minimum:
            errors.append(field + "_below_minimum")
    _number(measurements.get("truth_forward_displacement_m"),
            "truth_forward_displacement_m", errors, minimum=0.10, maximum=0.50)
    _number(measurements.get("slam_relative_displacement_error_m"),
            "slam_relative_displacement_error_m", errors, minimum=0.0, maximum=0.10)
    _number(measurements.get("slam_relative_yaw_error_rad"),
            "slam_relative_yaw_error_rad", errors, minimum=0.0, maximum=0.10)
    contacts = measurements.get("non_ground_contact_events")
    if isinstance(contacts, bool) or not isinstance(contacts, (int, float)) or contacts != 0:
        errors.append("non_ground_contacts_observed_or_missing")
    if not isinstance(payload.get("trial_id"), str) or not payload["trial_id"].strip():
        errors.append("trial_id_missing")
    return errors


def _validate_navigation(payload):
    errors = []
    if payload.get("classification") != NAV_CLASS:
        errors.append("classification_mismatch")
    if payload.get("status") != "PASS":
        errors.append("status_not_PASS")
    if payload.get("scenario_id") != "S00":
        errors.append("scenario_is_not_S00")
    if payload.get("localization_mode") != "known-start online SLAM":
        errors.append("localization_mode_is_not_online_SLAM")
    if payload.get("route_expected") is not True:
        errors.append("route_not_expected")
    if payload.get("runtime_navigation_receives_no_gazebo_truth") is not True:
        errors.append("runtime_truth_isolation_unverified")
    if payload.get("nav2_action_status") != "SUCCEEDED":
        errors.append("nav2_goal_not_succeeded")
    if payload.get("sole_authoritative_cmd_vel_publisher") is not True:
        errors.append("sole_guard_publisher_not_verified")
    if payload.get("travel_limit_exceeded") is not False:
        errors.append("travel_limit_failed_or_missing")
    _number(payload.get("truth_path_length_m"), "truth_path_length_m",
            errors, minimum=0.10)
    _number(payload.get("final_xy_error_m"), "final_xy_error_m",
            errors, minimum=0.0, maximum=0.30)
    _number(payload.get("final_heading_error_rad"), "final_heading_error_rad",
            errors, minimum=0.0, maximum=0.30)
    if payload.get("non_ground_contact_events") != []:
        errors.append("non_ground_contacts_observed_or_missing")
    _number(payload.get("min_evaluator_clearance_m"), "min_evaluator_clearance_m",
            errors, minimum=0.0)
    states = payload.get("nav2_lifecycle_states")
    required_lifecycle = (
        "planner_server", "controller_server", "behavior_server", "bt_navigator",
        "velocity_smoother", "collision_monitor",
    )
    if not isinstance(states, dict) or not all(
            states.get(name) == "active" for name in required_lifecycle):
        errors.append("nav2_lifecycle_activation_unverified")
    summary = payload.get("sensor_summary")
    if not isinstance(summary, dict) or not all(
            isinstance(summary.get(name), (int, float))
            and not isinstance(summary.get(name), bool)
            and summary[name] > 0
            for name in ("scan_count", "rgb_frames", "map_updates",
                         "global_costmap_updates", "plan_messages")):
        errors.append("live_sensor_costmap_or_plan_updates_missing")
    sensor = payload.get("sensor_integrity")
    if not isinstance(sensor, dict) or not all(sensor.get(name) is True for name in
        ("rgb_fresh", "scan_fresh", "map_fresh", "filtered_odom_received")):
        errors.append("sensor_integrity_failed")
    summary = payload.get("command_summary")
    command = summary.get("/cmd_vel") if isinstance(summary, dict) else None
    if (not isinstance(command, dict) or not isinstance(command.get("count"), int)
            or isinstance(command.get("count"), bool) or command["count"] <= 0):
        errors.append("guard_output_not_observed")
    audits = payload.get("cmd_vel_publisher_audit")
    audit = audits.get("/cmd_vel") if isinstance(audits, dict) else None
    if (not isinstance(audit, dict) or isinstance(audit.get("count"), bool)
            or audit.get("count") != 1
            or audit.get("publisher_names") != ["anc_safety_command_guard"]):
        errors.append("live_cmd_vel_graph_not_owned_by_guard")
    return errors


def _evidence_paths(reference):
    if isinstance(reference, str):
        return [reference]
    if isinstance(reference, list):
        return reference
    if isinstance(reference, dict):
        if isinstance(reference.get("path"), str):
            return [reference["path"]]
        if isinstance(reference.get("paths"), list):
            return reference["paths"]
    return []


def _load_payloads(reference, root):
    errors = []
    raw_paths = _evidence_paths(reference)
    if not raw_paths:
        return [], ["evidence_reference_missing"]
    paths = []
    for raw in raw_paths:
        if not isinstance(raw, str) or not raw:
            errors.append("evidence_path_invalid")
            continue
        candidate = (root / raw).resolve()
        try:
            candidate.relative_to(root.resolve())
        except ValueError:
            errors.append("evidence_path_escapes_project")
            continue
        if not candidate.is_file():
            errors.append("evidence_file_missing:" + raw)
            continue
        try:
            payload = json.loads(candidate.read_text())
        except (OSError, json.JSONDecodeError) as exc:
            errors.append("evidence_json_invalid:" + raw + ":" + str(exc))
            continue
        if not isinstance(payload, dict):
            errors.append("evidence_json_not_object:" + raw)
            continue
        paths.append((raw, payload))
    return paths, errors


def evaluate_readiness(config, project_root):
    root = Path(project_root).resolve()
    evidence_map = config.get("evidence", {})
    if not isinstance(evidence_map, dict):
        evidence_map = {}
    declared_flags = config.get("prerequisites", {})
    if not isinstance(declared_flags, dict):
        declared_flags = {}
    checks = {}
    all_errors = []
    validators = {
        "production_turning_verified": _validate_turn,
        "moving_slam_verified": _validate_moving,
        "guarded_command_chain_verified": _validate_navigation,
    }
    for key in REQUIREMENTS:
        paths, errors = _load_payloads(evidence_map.get(key), root)
        if key == "moving_slam_verified" and len(paths) < 3:
            errors.append("three_distinct_moving_trials_required")
        if key == "guarded_command_chain_verified" and not paths:
            errors.append("at_least_one_S00_navigation_trial_required")
        seen_trial_ids = set()
        for name, payload in paths:
            errors.extend(f"{name}:{error}" for error in validators[key](payload))
            if key == "moving_slam_verified":
                trial_id = payload.get("trial_id")
                if trial_id in seen_trial_ids:
                    errors.append("duplicate_moving_trial_id:" + str(trial_id))
                seen_trial_ids.add(trial_id)
        checks[key] = {
            "verified": bool(paths) and not errors,
            "declared_flag": declared_flags.get(key),
            "evidence_paths": [name for name, _ in paths],
            "errors": errors,
        }
        all_errors.extend(key + ":" + error for error in errors)
    return {
        "status": "READY" if not all_errors else "BLOCKED",
        "prerequisites": checks,
        "reasons": all_errors,
        "flags_are_not_proof": True,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path)
    parser.add_argument("project_root", type=Path)
    parser.add_argument("--brief", action="store_true")
    args = parser.parse_args()
    try:
        config = json.loads(args.config.read_text())
        if not isinstance(config, dict):
            raise ValueError("readiness_config_must_be_object")
        result = evaluate_readiness(config, args.project_root)
    except Exception as exc:
        print("ERROR: " + str(exc), file=sys.stderr)
        return 2
    if args.brief:
        reason = "; ".join(result["reasons"]) or "all evidence-backed prerequisites pass"
        print(result["status"] + "\t" + reason)
    else:
        print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
