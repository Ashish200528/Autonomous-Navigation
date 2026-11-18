"""Pure checks used by the bounded RGB + LiDAR presentation demo."""
import math

REQUEST_SPEED_MPS = 0.05
MAX_TRAVEL_M = 0.20
TARGET_TRAVEL_M = 0.15


def _number(value):
    if isinstance(value, bool):
        return None
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return None
    return value if math.isfinite(value) else None


def forward_block_reason(safety, safety_age_s, odometry_age_s, scan_age_s,
                         travelled_m=0.0):
    if not isinstance(safety, dict):
        return "safety_state_missing"
    if safety.get("valid") is not True or safety.get("authorized") is not True:
        return "safety_gate_denied:" + str(safety.get("reason", "unknown"))
    for name, age, limit in (("safety", safety_age_s, 0.35),
                             ("odometry", odometry_age_s, 0.50),
                             ("scan", scan_age_s, 0.60)):
        value = _number(age)
        if value is None or value < 0 or value > limit:
            return name + "_missing_or_stale"
    linear = _number(safety.get("max_linear_speed_mps"))
    angular = _number(safety.get("max_angular_speed_radps"))
    if linear is None or linear < REQUEST_SPEED_MPS:
        return "forward_speed_not_authorized"
    if angular is None or abs(angular) > 1e-9:
        return "forward_only_profile_required"
    distance = _number(travelled_m)
    if distance is None or distance < 0 or distance >= MAX_TRAVEL_M:
        return "demo_distance_limit"
    return None


def process_identity_is_live(stat_text, expected_start_ticks=None):
    """Parse Linux /proc/PID/stat; an unreaped zombie is already stopped."""
    try:
        if not isinstance(stat_text, str) or "(" not in stat_text:
            return False
        fields = stat_text[stat_text.rindex(")") + 2:].split()
        if len(fields) < 20 or fields[0] not in {"R", "S", "D", "T", "t", "I", "W", "P"}:
            return False
        ticks = int(fields[19])
        return expected_start_ticks is None or ticks == int(expected_start_ticks)
    except (TypeError, ValueError, IndexError):
        return False


def stream_problem(elapsed_s, ages, startup_timeout_s=30.0):
    for name in ("safety", "scan", "odometry"):
        age = ages.get(name)
        if age is None:
            if elapsed_s >= startup_timeout_s:
                return "sensor_startup_timeout:" + name
        else:
            value = _number(age)
            if value is None or value < 0 or value > 5.0:
                return "sensor_stream_stopped:" + name
    return None
