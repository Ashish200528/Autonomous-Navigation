#!/usr/bin/env python3
"""Bounded forward motion with stale-command stop and evaluator-only pose checks."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from simulation_interfaces.srv import GetEntityState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage


def yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y),
                      1.0-2.0*(q.y*q.y+q.z*q.z))


def pose_record(x, y, theta, stamp_s, wall_s):
    return {"x_m": float(x), "y_m": float(y), "yaw_rad": float(theta),
            "stamp_s": float(stamp_s), "wall_s": float(wall_s)}


def project(dx, dy, heading):
    return dx*math.cos(heading)+dy*math.sin(heading)


class SlamTimeoutProbe(Node):
    def __init__(self):
        super().__init__("phase8c_slam_timeout_probe")
        self.request_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.safety = None
        self.guard = None
        self.odom = None
        self.slam_pose = None
        self.map_odom = None
        self.output_samples = []
        self.guard_samples = []
        self.create_subscription(String, "/realistic/safety_state",
                                 self.on_safety, 10)
        self.create_subscription(String, "/realistic/command_guard_state",
                                 self.on_guard, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self.on_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/pose",
                                 self.on_slam_pose, 10)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 10)
        self.truth_client = self.create_client(GetEntityState,
                                               "/gzserver/get_entity_state")

    def on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.safety = {"valid": False, "reason": "safety_json_invalid"}

    def on_guard(self, msg):
        try:
            self.guard = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.guard = {"authorized": False, "reason": "guard_json_invalid"}
        self.guard_samples.append({"wall_s": time.monotonic(), **self.guard})
        self.guard_samples = self.guard_samples[-800:]

    def on_output(self, msg):
        self.output_samples.append({
            "wall_s": time.monotonic(), "linear_x_mps": float(msg.linear.x),
            "angular_z_radps": float(msg.angular.z),
        })
        self.output_samples = self.output_samples[-800:]

    def on_odom(self, msg):
        stamp_s = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.odom = pose_record(msg.pose.pose.position.x, msg.pose.pose.position.y,
                                yaw(msg.pose.pose.orientation), stamp_s,
                                time.monotonic())

    def on_slam_pose(self, msg):
        stamp_s = msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        self.slam_pose = pose_record(msg.pose.pose.position.x,
                                     msg.pose.pose.position.y,
                                     yaw(msg.pose.pose.orientation), stamp_s,
                                     time.monotonic())

    def on_tf(self, msg):
        for transform in msg.transforms:
            if (transform.header.frame_id.lstrip("/") == "map"
                    and transform.child_frame_id.lstrip("/") == "odom"):
                t = transform.transform.translation
                self.map_odom = pose_record(
                    t.x, t.y, yaw(transform.transform.rotation),
                    transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9,
                    time.monotonic())

    def publish_request(self, speed):
        msg = Twist()
        msg.linear.x = float(speed)
        self.request_pub.publish(msg)

    def spin_for(self, duration_s):
        deadline = time.monotonic()+duration_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_gate(self, stable_s=0.30, timeout_s=6.0):
        deadline = time.monotonic()+timeout_s
        valid_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            valid = (self.safety and self.safety.get("valid") is True
                     and float(self.safety.get("max_linear_speed_mps", 0.0)) >= 0.05)
            if valid:
                valid_since = valid_since or time.monotonic()
                if time.monotonic()-valid_since >= stable_s:
                    return dict(self.safety)
            else:
                valid_since = None
        reason = None if not self.safety else self.safety.get("reason")
        raise RuntimeError("gate_not_stably_ready:"+str(reason))

    def get_truth(self, timeout_s=2.0):
        request = GetEntityState.Request()
        request.entity = "anc_robot"
        future = self.truth_client.call_async(request)
        deadline = time.monotonic()+timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("evaluator_truth_timeout")
        response = future.result()
        if response.result.result != 1:
            raise RuntimeError("evaluator_truth_error:"+response.result.error_message)
        state = response.state
        stamp_s = state.header.stamp.sec+state.header.stamp.nanosec*1e-9
        return {
            "x_m": float(state.pose.position.x), "y_m": float(state.pose.position.y),
            "yaw_rad": yaw(state.pose.orientation),
            "linear_x_mps": float(state.twist.linear.x),
            "angular_z_radps": float(state.twist.angular.z),
            "sim_stamp_s": stamp_s, "wall_s": time.monotonic(),
            "evaluator_only": True,
        }

    def settle_without_refreshing_command(self, timeout_s=3.0):
        samples = []
        previous = self.get_truth()
        started = time.monotonic()
        slow_since = None
        settled = False
        while rclpy.ok() and time.monotonic()-started < timeout_s:
            self.spin_for(0.08)
            current = self.get_truth()
            dt = current["sim_stamp_s"]-previous["sim_stamp_s"]
            speed = yaw_rate = None
            if dt > 1e-6:
                speed = math.hypot(current["x_m"]-previous["x_m"],
                                   current["y_m"]-previous["y_m"])/dt
                dyaw = math.atan2(math.sin(current["yaw_rad"]-previous["yaw_rad"]),
                                  math.cos(current["yaw_rad"]-previous["yaw_rad"]))
                yaw_rate = abs(dyaw)/dt
            current["pose_derived_linear_speed_mps"] = speed
            current["pose_derived_yaw_rate_radps"] = yaw_rate
            samples.append(current)
            if speed is not None and speed <= 0.005 and yaw_rate is not None and yaw_rate <= 0.01:
                if slow_since is None:
                    slow_since = time.monotonic()
                elif time.monotonic()-slow_since >= 0.50:
                    settled = True
                    break
            else:
                slow_since = None
            previous = current
        return samples, settled

    def run(self):
        gate_at_start = self.wait_gate()
        if not self.truth_client.wait_for_service(timeout_sec=4.0):
            raise RuntimeError("evaluator_truth_service_missing")
        start_truth = self.get_truth()
        start_odom = dict(self.odom) if self.odom else None
        start_slam = dict(self.slam_pose) if self.slam_pose else None
        start_map_odom = dict(self.map_odom) if self.map_odom else None
        drive_start_wall = time.monotonic()
        drive_duration_s = 4.0
        max_displacement_m = 0.50
        latest_truth = start_truth
        truth_samples = [start_truth]
        next_truth_wall = drive_start_wall+0.40
        pretimeout_gate_invalid = None
        last_request_wall = None
        while rclpy.ok() and time.monotonic()-drive_start_wall < drive_duration_s:
            if self.safety and self.safety.get("valid") is not True:
                pretimeout_gate_invalid = dict(self.safety)
                break
            self.publish_request(0.05)
            last_request_wall = time.monotonic()
            rclpy.spin_once(self, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_truth_wall:
                latest_truth = self.get_truth()
                truth_samples.append(latest_truth)
                progress = math.hypot(latest_truth["x_m"]-start_truth["x_m"],
                                      latest_truth["y_m"]-start_truth["y_m"])
                if progress > max_displacement_m:
                    raise RuntimeError("bounded_displacement_limit_exceeded")
                next_truth_wall = time.monotonic()+0.40
        if last_request_wall is None:
            last_request_wall = time.monotonic()
        output_zero_wall = None
        timeout_guard_state = None
        deadline = time.monotonic()+0.80
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.guard:
                age = self.guard.get("command_age_s")
                output = self.guard.get("output", {})
                state = self.guard.get("safety_state", {})
                if (age is not None and float(age) >= 0.30
                        and output.get("is_zero") is True
                        and state.get("valid") is True):
                    output_zero_wall = time.monotonic()
                    timeout_guard_state = dict(self.guard)
                    break
                if state.get("valid") is not True:
                    pretimeout_gate_invalid = dict(state)
                    # The guard is already commanded to fail closed; keep
                    # observing until the request watchdog ages out as well.
        if output_zero_wall is None:
            self.publish_request(0.0)
            self.spin_for(0.50)
        zero_truth = self.get_truth()
        stop_samples, settled = self.settle_without_refreshing_command(3.0)
        final_truth = stop_samples[-1] if stop_samples else self.get_truth()
        initial_yaw = start_truth["yaw_rad"]
        total_forward = project(final_truth["x_m"]-start_truth["x_m"],
                                final_truth["y_m"]-start_truth["y_m"], initial_yaw)
        stop_distance = project(final_truth["x_m"]-zero_truth["x_m"],
                                final_truth["y_m"]-zero_truth["y_m"], initial_yaw)
        timeout_delay = (None if output_zero_wall is None
                         else output_zero_wall-last_request_wall)
        nonzero_samples = [v for v in self.output_samples
                           if v["linear_x_mps"] > 0.005]
        truth_delta = project(final_truth["x_m"]-start_truth["x_m"],
                              final_truth["y_m"]-start_truth["y_m"], initial_yaw)
        final_odom = dict(self.odom) if self.odom else None
        final_slam = dict(self.slam_pose) if self.slam_pose else None
        final_map_odom = dict(self.map_odom) if self.map_odom else None
        odom_delta = (None if start_odom is None or final_odom is None else
                      project(final_odom["x_m"]-start_odom["x_m"],
                              final_odom["y_m"]-start_odom["y_m"], initial_yaw))
        slam_delta = (None if start_slam is None or final_slam is None else
                      project(final_slam["x_m"]-start_slam["x_m"],
                              final_slam["y_m"]-start_slam["y_m"], initial_yaw))
        map_odom_delta = (None if start_map_odom is None or final_map_odom is None else
                          {"x_m": final_map_odom["x_m"]-start_map_odom["x_m"],
                           "y_m": final_map_odom["y_m"]-start_map_odom["y_m"],
                           "yaw_rad": math.atan2(
                               math.sin(final_map_odom["yaw_rad"]-start_map_odom["yaw_rad"]),
                               math.cos(final_map_odom["yaw_rad"]-start_map_odom["yaw_rad"]))})
        report = {
            "classification": "S00_BOUNDED_FORWARD_SLAM_AND_COMMAND_TIMEOUT_PROBE",
            "status": "FAIL",
            "runtime_received_no_simulator_truth": True,
            "evaluator_service": "/gzserver/get_entity_state",
            "request_topic": "/cmd_vel_nav",
            "authoritative_output_topic": "/cmd_vel",
            "drive_speed_request_mps": 0.05,
            "drive_duration_limit_s": drive_duration_s,
            "displacement_limit_m": max_displacement_m,
            "gate_at_start": gate_at_start,
            "pretimeout_gate_invalid": pretimeout_gate_invalid,
            "start_truth": start_truth,
            "final_truth": final_truth,
            "truth_forward_displacement_m": truth_delta,
            "guard_zero_truth": zero_truth,
            "command_timeout_reaction_s": timeout_delay,
            "guard_state_at_timeout_zero": timeout_guard_state,
            "total_forward_displacement_m": total_forward,
            "post_zero_stopping_distance_m": stop_distance,
            "post_zero_stopping_limit_m": 0.00298+0.10,
            "settled": settled,
            "nonzero_output_samples": len(nonzero_samples),
            "maximum_linear_output_mps": max(
                (abs(v["linear_x_mps"]) for v in self.output_samples), default=0.0),
            "start_odom_pose": start_odom, "final_odom_pose": final_odom,
            "start_slam_map_pose": start_slam, "final_slam_map_pose": final_slam,
            "start_map_to_odom": start_map_odom,
            "final_map_to_odom": final_map_odom,
            "slam_map_to_odom_delta": map_odom_delta,
            "odom_forward_displacement_m": odom_delta,
            "slam_forward_displacement_m": slam_delta,
            "odom_relative_displacement_error_m": (
                None if odom_delta is None else abs(odom_delta-truth_delta)),
            "slam_relative_displacement_error_m": (
                None if slam_delta is None else abs(slam_delta-truth_delta)),
            "truth_samples": truth_samples,
            "output_samples": self.output_samples,
            "guard_samples": self.guard_samples,
            "stop_truth_samples": stop_samples,
            "safety_state_at_end": self.safety,
            "localization_comparison_limit": (
                "single short straight S00 run; no turn, return path, loop closure, or multi-seed drift claim"),
        }
        report["status"] = "PASS" if (
            output_zero_wall is not None and timeout_delay is not None
            and timeout_delay <= 0.40 and pretimeout_gate_invalid is None
            and len(nonzero_samples) > 0 and 0.01 < total_forward <= max_displacement_m
            and stop_distance <= report["post_zero_stopping_limit_m"] and settled
        ) else "FAIL"
        return report


def main():
    if len(sys.argv) != 2:
        raise SystemExit("usage: phase8c_slam_timeout_probe.py OUTPUT_JSON")
    output_path = Path(sys.argv[1])
    rclpy.init()
    node = SlamTimeoutProbe()
    result = {"status": "FAIL"}
    try:
        result = node.run()
    except Exception as exc:
        result = {"status": "FAIL", "reason": str(exc),
                  "safety_state": node.safety,
                  "output_samples": node.output_samples,
                  "guard_samples": node.guard_samples}
        node.publish_request(0.0)
        node.spin_for(0.50)
    finally:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, sort_keys=True)+"\n")
        print(json.dumps({k: v for k, v in result.items()
                          if k not in ("output_samples", "guard_samples",
                                       "truth_samples", "stop_truth_samples")},
                         indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
