"""Offline safety and lifecycle policy checks for the visible LiDAR demo."""
import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "phase8c_demo_policy", ROOT / "scripts" / "phase8c_demo_policy.py")
POLICY = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(POLICY)


@pytest.fixture
def safety():
    return {
        "valid": True,
        "authorized": True,
        "max_linear_speed_mps": 0.18,
        "max_angular_speed_radps": 0.0,
    }


def forward(safety, **changes):
    arguments = dict(safety_age_s=0.1, odometry_age_s=0.1,
                     scan_age_s=0.1, travelled_m=0.0)
    arguments.update(changes)
    return POLICY.forward_block_reason(safety, **arguments)


def assert_blocked(reason):
    assert isinstance(reason, str) and reason.strip()


def stat_text(state="S", start_ticks="123456", comm="ros2 launch"):
    # /proc/PID/stat fields: state is field 3; starttime is field 22.
    fields = [state] + ["0"] * 18 + [str(start_ticks)] + ["0"] * 30
    return "4242 (" + comm + ") " + " ".join(fields) + "\n"


def test_fresh_authorized_straight_request_is_allowed(safety):
    assert forward(safety) is None
    safety["max_linear_speed_mps"] = 0.05
    safety["max_angular_speed_radps"] = -1e-9
    assert forward(safety, safety_age_s=0.35, odometry_age_s=0.5,
                   scan_age_s=0.6, travelled_m=0.199999) is None


@pytest.mark.parametrize("key", ["valid", "authorized"])
@pytest.mark.parametrize("value", [False, None, 1, "true"])
def test_forward_requires_explicit_safety_authorization(safety, key, value):
    safety[key] = value
    assert_blocked(forward(safety))


@pytest.mark.parametrize("key,value", [
    ("max_linear_speed_mps", 0.0499),
    ("max_linear_speed_mps", -0.05),
    ("max_linear_speed_mps", float("nan")),
    ("max_linear_speed_mps", float("inf")),
    ("max_linear_speed_mps", None),
    ("max_linear_speed_mps", "invalid"),
    ("max_angular_speed_radps", 1.01e-9),
    ("max_angular_speed_radps", -1.01e-9),
    ("max_angular_speed_radps", float("nan")),
    ("max_angular_speed_radps", float("inf")),
    ("max_angular_speed_radps", None),
    ("max_angular_speed_radps", "invalid"),
])
def test_forward_rejects_insufficient_or_malformed_caps(safety, key, value):
    safety[key] = value
    assert_blocked(forward(safety))


@pytest.mark.parametrize("key,value", [
    ("safety_age_s", 0.350001),
    ("odometry_age_s", 0.500001),
    ("scan_age_s", 0.600001),
])
def test_forward_stops_when_any_required_stream_is_stale(safety, key, value):
    assert_blocked(forward(safety, **{key: value}))


@pytest.mark.parametrize("key", ["safety_age_s", "odometry_age_s", "scan_age_s"])
@pytest.mark.parametrize("value", [-0.01, float("nan"), float("inf"), None, "bad"])
def test_forward_rejects_invalid_freshness(safety, key, value):
    assert_blocked(forward(safety, **{key: value}))


@pytest.mark.parametrize("distance", [
    0.2, 0.3, -0.01, float("nan"), float("inf"), None, "bad",
])
def test_forward_cannot_exceed_or_corrupt_distance_budget(safety, distance):
    assert_blocked(forward(safety, travelled_m=distance))


@pytest.mark.parametrize("state", [None, [], "valid"])
def test_malformed_safety_state_fails_closed(state):
    assert_blocked(forward(state))


@pytest.mark.parametrize("state", ["R", "S", "D", "T"])
def test_process_identity_accepts_live_states_and_complex_comm(state):
    text = stat_text(state=state, comm="python3 (ROS demo) with spaces")
    assert POLICY.process_identity_is_live(text)
    assert POLICY.process_identity_is_live(text, expected_start_ticks="123456")


@pytest.mark.parametrize("state", ["Z", "X", "x"])
def test_process_identity_rejects_zombies_and_dead_processes(state):
    assert not POLICY.process_identity_is_live(stat_text(state=state))


def test_process_identity_rejects_reused_pid():
    assert not POLICY.process_identity_is_live(
        stat_text(start_ticks="123457"), expected_start_ticks="123456")


@pytest.mark.parametrize("text", [
    "", "4242", "4242 (truncated", "4242 (ros2) S 0 0",
    "4242 (ros2) " + " ".join(["S"] + ["0"] * 18 + ["nan"]),
])
def test_process_identity_rejects_unparseable_stat(text):
    assert not POLICY.process_identity_is_live(text)


@pytest.mark.parametrize("elapsed", [0.0, 29.999])
def test_stream_policy_allows_bounded_startup_without_data(elapsed):
    assert POLICY.stream_problem(
        elapsed, {"safety": None, "scan": None, "odometry": None}) is None


@pytest.mark.parametrize("ages", [
    {"safety": None, "scan": None, "odometry": None},
    {"safety": 0.1, "scan": None, "odometry": 0.1},
    {"safety": 0.1},
])
def test_stream_policy_fails_missing_data_at_startup_deadline(ages):
    assert_blocked(POLICY.stream_problem(30.0, ages))


def test_stream_policy_accepts_fresh_streams_after_startup():
    ages = {"safety": 0.1, "scan": 0.2, "odometry": 0.1}
    assert POLICY.stream_problem(3600.0, ages) is None


@pytest.mark.parametrize("key", ["safety", "scan", "odometry"])
def test_stream_loss_is_detected_even_during_startup(key):
    ages = {"safety": 0.1, "scan": 0.2, "odometry": 0.1}
    ages[key] = 5.000001
    assert_blocked(POLICY.stream_problem(10.0, ages))


@pytest.mark.parametrize("value", [-0.01, float("nan"), float("inf"), "bad"])
def test_stream_policy_rejects_invalid_present_ages(value):
    assert_blocked(POLICY.stream_problem(
        10.0, {"safety": value, "scan": 0.1, "odometry": 0.1}))
