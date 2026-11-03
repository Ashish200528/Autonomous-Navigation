"""Conservative range-based speed calculation and command watchdog decisions."""
import math


def speed_gate(*, corridor_m, age_s, processing_latency_s, supervisor_period_s,
               stop_distance_by_speed, localization_allowance_m,
               footprint_margin_m, safety_margin_m, max_speed_mps):
    """Choose the highest measured speed whose complete forward bound fits.

    Stop data are (speed, worst observed stop distance) pairs. No extrapolation
    above the measured range is permitted. Distances share one reference origin.
    """
    scalars = (corridor_m, age_s, processing_latency_s, supervisor_period_s,
               localization_allowance_m, footprint_margin_m, safety_margin_m,
               max_speed_mps)
    if not all(math.isfinite(float(v)) for v in scalars):
        return {"valid": False, "max_linear_speed_mps": 0.0,
                "reason": "gate_input_nonfinite"}
    if any(v < 0 for v in scalars):
        return {"valid": False, "max_linear_speed_mps": 0.0,
                "reason": "gate_input_negative"}
    pairs = []
    try:
        for speed, stop in stop_distance_by_speed:
            speed, stop = float(speed), float(stop)
            if not math.isfinite(speed) or not math.isfinite(stop) or speed <= 0 or stop < 0:
                raise ValueError
            pairs.append((speed, stop))
    except (TypeError, ValueError):
        return {"valid": False, "max_linear_speed_mps": 0.0,
                "reason": "stopping_model_invalid"}
    pairs = sorted(set(pairs))
    if not pairs:
        return {"valid": False, "max_linear_speed_mps": 0.0,
                "reason": "stopping_model_unmeasured"}
    reaction_s = age_s + processing_latency_s + supervisor_period_s
    tested = []
    for speed, stop in pairs:
        if speed > max_speed_mps:
            continue
        required = (speed * reaction_s + stop + localization_allowance_m
                    + footprint_margin_m + safety_margin_m)
        tested.append({"speed_mps": speed, "stop_distance_m": stop,
                       "reaction_distance_m": speed * reaction_s,
                       "required_corridor_m": required,
                       "passes": required <= corridor_m})
    passing = [item for item in tested if item["passes"]]
    if not passing:
        return {"valid": True, "max_linear_speed_mps": 0.0,
                "reason": "insufficient_connected_clear_corridor",
                "reaction_time_s": reaction_s, "tested_speeds": tested}
    selected = max(passing, key=lambda item: item["speed_mps"])
    return {"valid": True, "max_linear_speed_mps": selected["speed_mps"],
            "reason": "measured_speed_bound_fits_corridor",
            "reaction_time_s": reaction_s, "selected": selected,
            "tested_speeds": tested}


def guarded_command(*, request, request_age_s, safety_state, safety_age_s,
                    command_timeout_s, safety_timeout_s,
                    hard_max_linear_speed_mps, hard_max_angular_speed_radps):
    """Return a zero or bounded (linear.x, angular.z) command and reason."""
    zero = {"linear_x_mps": 0.0, "angular_z_radps": 0.0}
    if not isinstance(request, dict) or not isinstance(safety_state, dict):
        return zero | {"reason": "required_input_missing", "authorized": False}
    try:
        request_age_s, safety_age_s = float(request_age_s), float(safety_age_s)
    except (TypeError, ValueError):
        return zero | {"reason": "input_age_invalid", "authorized": False}
    if (not math.isfinite(request_age_s) or request_age_s < 0
            or request_age_s > command_timeout_s):
        return zero | {"reason": "command_stale", "authorized": False}
    if (not math.isfinite(safety_age_s) or safety_age_s < 0
            or safety_age_s > safety_timeout_s):
        return zero | {"reason": "safety_state_stale", "authorized": False}
    if safety_state.get("valid") is not True:
        return zero | {"reason": str(safety_state.get("reason", "safety_state_invalid")),
                       "authorized": False}
    if safety_state.get("authorized") is False:
        return zero | {"reason": "safety_authority_false", "authorized": False}
    try:
        vx, wz = float(request["linear_x_mps"]), float(request["angular_z_radps"])
        max_v = min(float(safety_state["max_linear_speed_mps"]),
                    float(hard_max_linear_speed_mps))
        max_w = min(float(safety_state["max_angular_speed_radps"]),
                    float(hard_max_angular_speed_radps))
    except (KeyError, TypeError, ValueError):
        return zero | {"reason": "command_or_gate_malformed", "authorized": False}
    if not all(math.isfinite(v) for v in (vx, wz, max_v, max_w)):
        return zero | {"reason": "command_or_gate_nonfinite", "authorized": False}
    if safety_state.get("motion_mode") == "rotation_calibration_only":
        if abs(vx) > 1e-9 and abs(wz) > 1e-9:
            return zero | {"reason": "combined_motion_not_validated", "authorized": False}
        if abs(vx) > 1e-9 and abs(wz) <= 1e-9:
            return zero | {"reason": "rotation_calibration_translation_blocked",
                           "authorized": False}
    if (safety_state.get('combined_motion_authorized') is False
            and abs(vx)>1e-9 and abs(wz)>1e-9):
        return zero | {'reason':'combined_motion_not_validated','authorized':False}
    if safety_state.get('motion_mode')=='production_turn_validation_only' and abs(vx)>1e-9:
        return zero | {'reason':'production_turn_validation_translation_blocked','authorized':False}
    if safety_state.get('motion_mode') in ('forward_only','production_sequential') and vx < -1e-9:
        return zero | {'reason':'reverse_motion_not_validated','authorized':False}
    if max_v < 0 or max_w < 0:
        return zero | {"reason": "gate_speed_limit_negative", "authorized": False}
    v = max(-max_v, min(max_v, vx))
    w = max(-max_w, min(max_w, wz))
    reason = "authorized_bounded_command"
    if v != vx or w != wz:
        reason = "command_clamped_to_measured_gate"
    return {"linear_x_mps": v, "angular_z_radps": w,
            "reason": reason, "authorized": True}


def differential_drive_command(msg):
    values = (msg.linear.x, msg.linear.y, msg.linear.z,
              msg.angular.x, msg.angular.y, msg.angular.z)
    if not all(math.isfinite(float(v)) for v in values):
        return None
    if any(abs(float(v)) > 1e-9 for v in
           (msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y)):
        return None
    return {"linear_x_mps": float(msg.linear.x),
            "angular_z_radps": float(msg.angular.z)}


class SequentialMotionInterlock:
    """Additional authority restriction for separately validated forward/yaw primitives."""
    def __init__(self):
        self.last_motion=None
        self.last_nonzero=None
        self.slow_since=None

    def apply(self, decision, now, odom_velocity, odom_age):
        zero=dict(linear_x_mps=0.0,angular_z_radps=0.0,authorized=False)
        fresh=(odom_velocity is not None and 0<=odom_age<=.20
               and all(math.isfinite(v) for v in odom_velocity))
        slow=fresh and math.hypot(odom_velocity[0],odom_velocity[1])<=.005 and abs(odom_velocity[2])<=.01
        if not slow:self.slow_since=None
        elif self.slow_since is None:self.slow_since=now
        if not fresh:return zero|{'reason':'sequential_odometry_missing_or_stale'}
        if not decision.get('authorized'):return decision
        v,w=decision['linear_x_mps'],decision['angular_z_radps']
        if abs(v)<=1e-9 and abs(w)<=1e-9:return decision
        target=('forward',1) if abs(v)>1e-9 else ('yaw',1 if w>0 else -1)
        if target!=self.last_motion:
            held=self.slow_since is not None and now-self.slow_since>=.50
            stopped_long_enough=self.last_nonzero is None or now-self.last_nonzero>=1.25
            if not held or not stopped_long_enough:
                return zero|{'reason':'waiting_for_confirmed_primitive_stop'}
        self.last_motion=target;self.last_nonzero=now
        return decision
