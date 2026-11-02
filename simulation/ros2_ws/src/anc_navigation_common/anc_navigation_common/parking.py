"""Sensor-neutral, proposal-only parking controller for simulation scaffolding.

The caller must supply a fresh localized pose and a validated safety state.
All commands are proposals for the existing authoritative command guard.
This module is not a ROS node and does not claim a live parking trial.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class ParkingConfig:
    position_tolerance_m: float = 0.20
    heading_tolerance_rad: float = math.radians(10.0)
    stationary_hold_s: float = 5.0
    linear_speed_cap_mps: float = 0.08
    angular_speed_cap_radps: float = 0.10


def _wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _result(phase, reason, linear=0.0, angular=0.0,
            position_error=None, heading_error=None, complete=False,
            stationary_duration_s=0.0):
    return {
        "phase": phase,
        "reason": reason,
        "linear_x_mps": float(linear),
        "angular_z_radps": float(angular),
        "position_error_m": position_error,
        "heading_error_rad": heading_error,
        "stationary_duration_s": float(stationary_duration_s),
        "complete": bool(complete),
        "authoritative_output": False,
    }


def parking_step(*, current_pose, target_pose, safety_state,
                 localization_valid, measured_stationary,
                 stationary_duration_s, config=None):
    """Return one guarded parking proposal; fail closed on invalid inputs.

    Poses are (x_m, y_m, yaw_rad) in the same valid map frame. The approach
    uses sequential rotate-then-drive, never a combined command. This avoids
    asserting support for a motion mode that the live safety gate has not
    validated.
    """
    config = config or ParkingConfig()
    zero = _result("BLOCKED", "invalid_or_missing_input")
    if not localization_valid:
        return _result("BLOCKED", "localization_invalid")
    if not isinstance(safety_state, dict) or safety_state.get("valid") is not True:
        return _result("BLOCKED", "safety_state_invalid")
    if safety_state.get("authorized") is not True:
        return _result("BLOCKED", "safety_authority_false")
    if safety_state.get("motion_mode") == "rotation_calibration_only":
        return _result("BLOCKED", "parking_requires_production_motion_mode")

    try:
        x, y, yaw = (float(v) for v in current_pose)
        tx, ty, tyaw = (float(v) for v in target_pose)
        stationary_duration_s = float(stationary_duration_s)
        max_v = float(safety_state["max_linear_speed_mps"])
        max_w = float(safety_state["max_angular_speed_radps"])
        cfg = (
            float(config.position_tolerance_m),
            float(config.heading_tolerance_rad),
            float(config.stationary_hold_s),
            float(config.linear_speed_cap_mps),
            float(config.angular_speed_cap_radps),
        )
        if not all(math.isfinite(v) for v in (x, y, yaw, tx, ty, tyaw,
                                                stationary_duration_s, max_v, max_w, *cfg)):
            return zero
        pos_tol, yaw_tol, hold_s, v_cap, w_cap = cfg
        if min(pos_tol, yaw_tol, hold_s, v_cap, w_cap) <= 0:
            return zero
        if stationary_duration_s < 0 or max_v < 0 or max_w < 0:
            return zero
    except (KeyError, TypeError, ValueError, OverflowError):
        return zero

    dx, dy = tx-x, ty-y
    distance = math.hypot(dx, dy)
    final_heading_error = _wrap_angle(tyaw-yaw)
    if distance > pos_tol:
        approach_heading_error = _wrap_angle(math.atan2(dy, dx)-yaw)
        if abs(approach_heading_error) > yaw_tol:
            if max_w <= 0:
                return _result("BLOCKED", "angular_authority_missing",
                               position_error=distance,
                               heading_error=approach_heading_error)
            requested_w = min(w_cap, max_w, max(0.015, 0.8*abs(approach_heading_error)))
            return _result("ALIGN_APPROACH", "rotate_toward_target",
                           angular=math.copysign(requested_w, approach_heading_error),
                           position_error=distance,
                           heading_error=approach_heading_error)
        if max_v <= 0:
            return _result("BLOCKED", "linear_authority_missing",
                           position_error=distance,
                           heading_error=final_heading_error)
        requested_v = min(v_cap, max_v, max(0.01, 0.5*distance))
        return _result("APPROACH", "drive_forward_segment",
                       linear=requested_v, position_error=distance,
                       heading_error=final_heading_error)

    if abs(final_heading_error) > yaw_tol:
        if max_w <= 0:
            return _result("BLOCKED", "angular_authority_missing",
                           position_error=distance,
                           heading_error=final_heading_error)
        requested_w = min(w_cap, max_w, max(0.015, 0.8*abs(final_heading_error)))
        return _result("ALIGN_FINAL", "rotate_to_parking_heading",
                       angular=math.copysign(requested_w, final_heading_error),
                       position_error=distance,
                       heading_error=final_heading_error)

    if not measured_stationary:
        return _result("HOLD", "waiting_for_stationary_feedback",
                       position_error=distance, heading_error=final_heading_error,
                       stationary_duration_s=0.0)
    if stationary_duration_s < hold_s:
        return _result("HOLD", "stationary_dwell_in_progress",
                       position_error=distance, heading_error=final_heading_error,
                       stationary_duration_s=stationary_duration_s)
    return _result("COMPLETE", "parking_criteria_met",
                   position_error=distance, heading_error=final_heading_error,
                   complete=True, stationary_duration_s=stationary_duration_s)