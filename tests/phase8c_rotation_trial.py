#!/usr/bin/env python3
"""Bounded pure-yaw calibration; evaluator truth is isolated from runtime nodes."""
import json
import math
import sys
import time
from collections import Counter
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from simulation_interfaces.srv import GetEntityState
from std_msgs.msg import String


def angle_delta(a, b):
    return math.atan2(math.sin(a-b), math.cos(a-b))


def pose_yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y),
                      1.0-2.0*(q.y*q.y+q.z*q.z))


def pose_rate(previous, current):
    dt = float(current["sim_stamp_s"]-previous["sim_stamp_s"])
    if dt <= 1e-6:
        return None, None
    dx = current["x_m"]-previous["x_m"]
    dy = current["y_m"]-previous["y_m"]
    dyaw = angle_delta(current["yaw_rad"], previous["yaw_rad"])
    return math.hypot(dx, dy)/dt, abs(dyaw)/dt


def parse_json(msg):
    try:
        return json.loads(msg.data)
    except (TypeError, json.JSONDecodeError):
        return {"valid": False, "reason": "safety_json_invalid"}


class RotationTrial(Node):
    def __init__(self):
        super().__init__("phase8c_rotation_calibration_evaluator")
        self.request_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.safety = None
        self.outputs = []
        self.contact_records = 0
        self.non_ground_contact_records = 0
        self.contact_pair_counts = Counter()
        self.contact_pair_examples = []
        self.create_subscription(String, "/realistic/safety_state",
                                 lambda msg: setattr(self, "safety", parse_json(msg)), 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(Contacts, "/contacts", self.on_contacts, 10)
        self.truth = self.create_client(GetEntityState, "/gzserver/get_entity_state")

    def on_output(self, msg):
        self.outputs.append({
            "wall_s": time.monotonic(),
            "linear_x_mps": float(msg.linear.x),
            "angular_z_radps": float(msg.angular.z),
        })
        self.outputs = self.outputs[-1200:]

    def on_contacts(self, msg):
        for contact in msg.contacts:
            c1, c2 = contact.collision1.name, contact.collision2.name
            pair = f"{c1} <> {c2}"
            self.contact_records += 1
            self.contact_pair_counts[pair] += 1
            if "ground_plane" not in (c1+" "+c2).lower():
                self.non_ground_contact_records += 1
            if len(self.contact_pair_examples) < 12 and pair not in self.contact_pair_examples:
                self.contact_pair_examples.append(pair)

    def publish_request(self, angular_z):
        msg = Twist()
        msg.angular.z = float(angular_z)
        self.request_pub.publish(msg)

    def spin_for(self, duration_s, angular_z):
        deadline = time.monotonic()+duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            self.publish_request(angular_z)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_gate(self, cap, timeout_s=12.0, stable_s=0.40):
        deadline = time.monotonic()+timeout_s
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self.safety or {}
            ready = (
                state.get("valid") is True and state.get("authorized") is True
                and state.get("motion_mode") == "rotation_calibration_only"
                and abs(float(state.get("max_linear_speed_mps", -1.0))) < 1e-9
                and float(state.get("max_angular_speed_radps", 0.0)) >= cap-1e-4
                and (state.get("rotation_clearance") or {}).get("authorized") is True
            )
            if ready:
                stable_since = stable_since or time.monotonic()
                if time.monotonic()-stable_since >= stable_s:
                    return dict(state)
            else:
                stable_since = None
        raise RuntimeError("rotation_gate_not_stably_authorized:"+json.dumps(self.safety or {}))

    def get_truth(self, timeout_s=2.5):
        request = GetEntityState.Request()
        request.entity = "anc_robot"
        future = self.truth.call_async(request)
        deadline = time.monotonic()+timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("evaluator_truth_timeout")
        response = future.result()
        if response.result.result != 1:
            raise RuntimeError("evaluator_truth_error:"+response.result.error_message)
        state = response.state
        return {
            "wall_s": time.monotonic(),
            "sim_stamp_s": state.header.stamp.sec+state.header.stamp.nanosec*1e-9,
            "x_m": float(state.pose.position.x),
            "y_m": float(state.pose.position.y),
            "yaw_rad": pose_yaw(state.pose.orientation),
            "linear_x_mps": float(state.twist.linear.x),
            "linear_y_mps": float(state.twist.linear.y),
            "angular_z_radps": float(state.twist.angular.z),
            "evaluator_only": True,
        }

    def settle_from_pose(self, baseline, timeout_s=4.0):
        started = time.monotonic()
        previous = baseline
        final = baseline
        samples = []
        slow_since = None
        first_slow_wall = None
        last_truth_wall = 0.0
        settled = False
        while rclpy.ok() and time.monotonic()-started < timeout_s:
            self.publish_request(0.0)
            rclpy.spin_once(self, timeout_sec=0.03)
            now = time.monotonic()
            if now-last_truth_wall < 0.10:
                continue
            current = self.get_truth()
            last_truth_wall = time.monotonic()
            linear_rate, angular_rate = pose_rate(previous, current)
            current["pose_derived_linear_speed_mps"] = linear_rate
            current["pose_derived_angular_speed_radps"] = angular_rate
            samples.append(current)
            final = current
            slow = (linear_rate is not None and linear_rate <= 0.005
                    and angular_rate is not None and angular_rate <= 0.01)
            if slow:
                if slow_since is None:
                    slow_since = now
                    first_slow_wall = now
                elif now-slow_since >= 0.50:
                    settled = True
                    break
            else:
                slow_since = None
                first_slow_wall = None
            previous = current
        return final, settled, samples, first_slow_wall

    def run_case(self, requested_w, duration_s, cap):
        gate = self.wait_gate(cap)
        contact_start = self.contact_records
        nonground_start = self.non_ground_contact_records
        pairs_start = self.contact_pair_counts.copy()
        output_start = len(self.outputs)
        start_truth = self.get_truth()
        start_wall = time.monotonic()
        self.spin_for(duration_s, requested_w)
        zero_request_wall = time.monotonic()
        self.publish_request(0.0)

        zero_sample = None
        zero_deadline = zero_request_wall+1.0
        while rclpy.ok() and time.monotonic() < zero_deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            candidates = [item for item in self.outputs
                          if item["wall_s"] >= zero_request_wall
                          and abs(item["angular_z_radps"]) <= 1e-8
                          and abs(item["linear_x_mps"]) <= 1e-8]
            if candidates:
                zero_sample = candidates[0]
                break

        after_zero = self.get_truth()
        final_truth, settled, settle_samples, first_slow_wall = self.settle_from_pose(after_zero)
        end_wall = time.monotonic()
        outputs = [item for item in self.outputs[output_start:]
                   if start_wall <= item["wall_s"] <= end_wall]
        active = [item for item in outputs if abs(item["angular_z_radps"]) > 1e-8]
        yaw_change = angle_delta(after_zero["yaw_rad"], start_truth["yaw_rad"])
        sim_elapsed = after_zero["sim_stamp_s"]-start_truth["sim_stamp_s"]
        avg_rate = yaw_change/sim_elapsed if sim_elapsed > 1e-6 else None
        residual_yaw = angle_delta(final_truth["yaw_rad"], after_zero["yaw_rad"])
        drift = math.hypot(final_truth["x_m"]-start_truth["x_m"],
                           final_truth["y_m"]-start_truth["y_m"])
        zero_latency = (None if zero_sample is None else
                        zero_sample["wall_s"]-zero_request_wall)
        settle_wall = end_wall-zero_request_wall
        settle_sim = final_truth["sim_stamp_s"]-after_zero["sim_stamp_s"]
        max_w = max((abs(item["angular_z_radps"]) for item in outputs), default=0.0)
        max_v = max((abs(item["linear_x_mps"]) for item in outputs), default=0.0)
        all_contacts = self.contact_records-contact_start
        nonground_contacts = self.non_ground_contact_records-nonground_start
        pair_counts = dict(self.contact_pair_counts-pairs_start)
        direction_ok = (avg_rate is not None and avg_rate*requested_w > 0
                        and abs(yaw_change) >= 0.005)
        passed = (
            bool(active) and direction_ok and max_w <= cap+1e-5 and max_v <= 1e-5
            and zero_latency is not None and 0.0 <= zero_latency <= 0.30
            and settled and settle_wall <= 0.75 and drift <= 0.10
            and abs(residual_yaw) <= 0.05 and nonground_contacts == 0
        )
        return {
            "requested_angular_speed_radps": requested_w,
            "gate_snapshot": gate,
            "start_truth": start_truth,
            "truth_after_first_zero_output": after_zero,
            "final_truth": final_truth,
            "measured_average_yaw_rate_radps": avg_rate,
            "yaw_change_until_zero_rad": yaw_change,
            "zero_request_to_zero_output_wall_s": zero_latency,
            "zero_request_to_settled_wall_s": settle_wall,
            "first_slow_pose_rate_wall_s_after_zero_request": (
                None if first_slow_wall is None else first_slow_wall-zero_request_wall),
            "sim_seconds_after_zero_output_to_settled": settle_sim,
            "residual_yaw_after_zero_output_rad": residual_yaw,
            "center_drift_m": drift,
            "settled_by_pose_rate_within_4s": settled,
            "settle_speed_source": "finite difference of consecutive evaluator-only Gazebo truth poses in simulation time",
            "max_abs_actual_angular_radps": max_w,
            "max_abs_actual_linear_mps": max_v,
            "reported_truth_twist_after_zero": {
                "linear_x_mps": after_zero["linear_x_mps"],
                "linear_y_mps": after_zero["linear_y_mps"],
                "angular_z_radps": after_zero["angular_z_radps"],
            },
            "contact_records_all": all_contacts,
            "contact_records_non_ground": nonground_contacts,
            "contact_pair_counts": pair_counts,
            "contact_pair_examples": self.contact_pair_examples,
            "trial_pass": passed,
            "pose_settle_samples": settle_samples,
            "output_samples": outputs,
        }


def main():
    output = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        "docs/evidence/phase8c/lidar/stage1-rotation-trials-retry.json")
    cap = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
    duration = float(sys.argv[3]) if len(sys.argv) > 3 else 1.20
    speeds = (0.02, -0.02, 0.05, -0.05, 0.10, -0.10)
    if not 0.02 <= cap <= 0.10 or duration <= 0 or duration > 2.0:
        raise SystemExit("cap must be 0.02..0.10 rad/s and duration must be 0..2 s")
    rclpy.init()
    node = RotationTrial()
    result = {
        "classification": "BOUNDED_PURE_YAW_CALIBRATION_ONLY",
        "runtime_navigation_receives_no_gazebo_truth": True,
        "truth_service_evaluator_only": "/gzserver/get_entity_state",
        "requested_topic": "/cmd_vel_nav",
        "authoritative_topic": "/cmd_vel",
        "production_angular_authority_enabled": False,
        "launch_cap_radps": cap,
        "duration_per_trial_s": duration,
        "requested_speeds_radps": list(speeds),
        "trials": [],
    }
    try:
        result["preflight_gate"] = node.wait_gate(cap)
        for speed in speeds:
            result["trials"].append(node.run_case(speed, duration, cap))
        node.spin_for(0.60, 0.0)
    except Exception as exc:
        result["error"] = f"{type(exc).__name__}: {exc}"
        node.spin_for(0.60, 0.0)
    finally:
        result["all_trials_pass"] = (
            len(result["trials"]) == len(speeds)
            and all(trial.get("trial_pass") for trial in result["trials"])
        )
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True)+"\n")
        summary = {
            "evidence_json": str(output),
            "all_trials_pass": result["all_trials_pass"],
            "production_angular_authority_enabled": False,
            "trials": [{
                "requested_radps": item["requested_angular_speed_radps"],
                "passed": item["trial_pass"],
                "yaw_delta_rad": item["yaw_change_until_zero_rad"],
                "zero_to_zero_output_s": item["zero_request_to_zero_output_wall_s"],
                "settle_wall_s": item["zero_request_to_settled_wall_s"],
                "center_drift_m": item["center_drift_m"],
                "residual_yaw_rad": item["residual_yaw_after_zero_output_rad"],
                "non_ground_contacts": item["contact_records_non_ground"],
            } for item in result["trials"]]
        }
        if "error" in result:
            summary["error"] = result["error"]
        print(json.dumps(summary, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result["all_trials_pass"] else 1)


if __name__ == "__main__":
    main()