#!/usr/bin/env python3
"""Bounded Stage C S00 motion test through the Phase 8C command guard.

The only motion request is /cmd_vel_nav. Gazebo truth is queried here as evaluator
data and is never published to the runtime ROS graph.
"""
import json
import math
import os
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from simulation_interfaces.srv import GetEntityState
from std_msgs.msg import String


def yaw(q):
    return math.atan2(2.0*(q.w*q.z+q.x*q.y),
                      1.0-2.0*(q.y*q.y+q.z*q.z))


def parse_json(msg):
    try:
        return json.loads(msg.data)
    except (TypeError, json.JSONDecodeError):
        return {"valid": False, "reason": "json_invalid"}


class GuardedMotionTrial(Node):
    def __init__(self):
        super().__init__("phase8c_guarded_motion_evaluator")
        self.request_pub = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.create_subscription(String, "/realistic/safety_state",
                                 self.on_safety, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.create_subscription(Odometry, "/odometry/filtered",
                                 self.on_odom, qos_profile_sensor_data)
        self.truth_client = self.create_client(GetEntityState, "/gzserver/get_entity_state")
        self.safety = None
        self.odom = None
        self.output_samples = []
        self.t0 = time.monotonic()

    def on_safety(self, msg):
        self.safety = parse_json(msg)

    def on_output(self, msg):
        self.output_samples.append({
            "wall_s": time.monotonic(),
            "linear_x_mps": float(msg.linear.x),
            "angular_z_radps": float(msg.angular.z),
        })
        self.output_samples = self.output_samples[-600:]

    def on_odom(self, msg):
        self.odom = {
            "stamp_s": msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9,
            "frame_id": msg.header.frame_id,
            "child_frame_id": msg.child_frame_id,
            "x_m": float(msg.pose.pose.position.x),
            "y_m": float(msg.pose.pose.position.y),
            "yaw_rad": yaw(msg.pose.pose.orientation),
            "linear_x_mps": float(msg.twist.twist.linear.x),
        }

    def publish_request(self, speed):
        msg = Twist()
        msg.linear.x = float(speed)
        self.request_pub.publish(msg)

    def spin_for(self, duration_s, request_speed):
        deadline = time.monotonic()+duration_s
        while time.monotonic() < deadline and rclpy.ok():
            self.publish_request(request_speed)
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_gate(self, timeout_s=5.0, min_speed=0.05):
        deadline = time.monotonic()+timeout_s
        last_reason = None
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.safety:
                last_reason = self.safety.get("reason")
                if (self.safety.get("valid") is True
                        and float(self.safety.get("max_linear_speed_mps", 0.0)) >= min_speed):
                    return
        raise RuntimeError("measured_gate_not_ready:" + str(last_reason))

    def get_truth(self, timeout_s=3.0):
        req = GetEntityState.Request()
        req.entity = "anc_robot"
        future = self.truth_client.call_async(req)
        deadline = time.monotonic()+timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done() or future.result() is None:
            raise RuntimeError("evaluator_truth_timeout")
        res = future.result()
        if res.result.result != 1:
            raise RuntimeError("evaluator_truth_error:" + res.result.error_message)
        state = res.state
        return {
            "wall_s": time.monotonic(), "sim_stamp_s":
                state.header.stamp.sec+state.header.stamp.nanosec*1e-9,
            "x_m": float(state.pose.position.x), "y_m": float(state.pose.position.y),
            "yaw_rad": yaw(state.pose.orientation),
            "linear_x_mps": float(state.twist.linear.x),
            "linear_y_mps": float(state.twist.linear.y),
            "angular_z_radps": float(state.twist.angular.z),
            "evaluator_only": True,
        }

    def wait_for_guard_zero_after(self, start_wall, timeout_s):
        deadline = time.monotonic()+timeout_s
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            recent = [v for v in self.output_samples if v["wall_s"] >= start_wall]
            if recent and abs(recent[-1]["linear_x_mps"]) < 1e-8:
                return recent[-1]["wall_s"]
        return None

    def settle(self, duration_limit_s=3.0):
        records = []
        previous = self.get_truth()
        started = time.monotonic()
        slow_since = None
        settled = False
        while time.monotonic()-started < duration_limit_s:
            self.publish_request(0.0)
            rclpy.spin_once(self, timeout_sec=0.03)
            if not records or time.monotonic()-records[-1]["wall_s"] >= 0.10:
                current = self.get_truth()
                dt = current["sim_stamp_s"]-previous["sim_stamp_s"]
                speed = None
                yaw_rate = None
                if dt > 1e-6:
                    speed = math.hypot(current["x_m"]-previous["x_m"],
                                       current["y_m"]-previous["y_m"])/dt
                    dyaw = math.atan2(math.sin(current["yaw_rad"]-previous["yaw_rad"]),
                                      math.cos(current["yaw_rad"]-previous["yaw_rad"]))
                    yaw_rate = abs(dyaw)/dt
                current["pose_derived_linear_speed_mps"] = speed
                current["pose_derived_yaw_rate_radps"] = yaw_rate
                records.append(current)
                previous = current
                if speed is not None and speed <= 0.005 and yaw_rate is not None and yaw_rate <= 0.01:
                    if slow_since is None:
                        slow_since = time.monotonic()
                    elif time.monotonic()-slow_since >= 0.50:
                        settled = True
                        break
                else:
                    slow_since = None
        return records, settled

    def run(self, fault_pid, fault_name="lidar_conditioner"):
        self.wait_gate()
        gate_at_start = dict(self.safety)
        start_truth = self.get_truth()
        first_output_index = len(self.output_samples)
        self.spin_for(0.75, 0.05)
        drive_outputs = self.output_samples[first_output_index:]
        if not any(v["linear_x_mps"] > 0.005 for v in drive_outputs):
            self.publish_request(0.0)
            raise RuntimeError("guard_did_not_pass_bounded_0.05_mps_request")
        before_loss_truth = self.get_truth()

        pause_wall = time.monotonic()
        zero_wall = None
        zero_truth = None
        invalid_wall = None
        invalid_safety_state = None
        stale_invalid_wall = None
        stale_safety_state = None
        stale_zero_wall = None
        stale_zero_truth = None
        displacement_limit_exceeded = False
        last_truth_check = pause_wall
        os.kill(fault_pid, signal.SIGSTOP)
        try:
            # This 2.05 s window bounds nominal travel to 0.103 m at
            # 0.05 m/s. Evaluator truth is sampled periodically as a check.
            deadline = pause_wall + 2.05
            while time.monotonic() < deadline and rclpy.ok():
                self.publish_request(0.05)
                rclpy.spin_once(self, timeout_sec=0.02)
                now = time.monotonic()
                if self.safety and self.safety.get("valid") is not True:
                    failures = [str(v) for v in self.safety.get("failures", [])]
                    if fault_name == "rgb_perception":
                        stale_match = any(
                            v.startswith(("rgb_local_map_stale",
                                          "rgb_perception_health_stale"))
                            or (v.startswith("rgb_input:")
                                and ("stale" in v or "missing" in v))
                            for v in failures)
                        if stale_match and stale_invalid_wall is None:
                            stale_invalid_wall = now
                            stale_safety_state = dict(self.safety)
                        target_invalid = any(v.startswith(
                            ("rgb_local_map_", "rgb_perception_health_", "rgb_input:"))
                            for v in failures)
                    elif fault_name == "lidar_conditioner":
                        target_invalid = any(v.startswith((
                            "scan_health", "scan_invalid", "scan_sim_",
                            "measured_scan_rate_", "usable_beam_",
                            "lidar_local_map_", "lidar_map_health_", "lidar_map_invalid"))
                            for v in failures)
                    elif fault_name == "ekf_localization":
                        target_invalid = any(v.startswith("filtered_localization")
                                             for v in failures)
                    else:
                        target_invalid = True
                    if target_invalid and invalid_wall is None:
                        invalid_wall = now
                        invalid_safety_state = dict(self.safety)
                recent = [v for v in self.output_samples
                          if invalid_wall is not None and v["wall_s"] >= invalid_wall]
                if (invalid_wall is not None and recent
                        and abs(recent[-1]["linear_x_mps"]) < 1e-8):
                    if zero_wall is None:
                        zero_wall = recent[-1]["wall_s"]
                        zero_truth = self.get_truth()
                    if fault_name != "rgb_perception":
                        break
                if fault_name == "rgb_perception" and stale_invalid_wall is not None:
                    stale_outputs = [v for v in self.output_samples
                                     if v["wall_s"] >= stale_invalid_wall]
                    if (stale_outputs
                            and abs(stale_outputs[-1]["linear_x_mps"]) < 1e-8):
                        stale_zero_wall = stale_outputs[-1]["wall_s"]
                        stale_zero_truth = self.get_truth()
                        break
                if now-last_truth_check >= 0.25:
                    current = self.get_truth()
                    progress = math.hypot(current["x_m"]-start_truth["x_m"],
                                          current["y_m"]-start_truth["y_m"])
                    last_truth_check = time.monotonic()
                    if progress > 0.50:
                        displacement_limit_exceeded = True
                        break
            if fault_name == "rgb_perception" and stale_zero_wall is None:
                # Keep the actuator request at zero before resuming perception
                # if the expected stale-state failure was not observed.
                self.publish_request(0.0)
                self.spin_for(0.50, 0.0)
        finally:
            # Never leave the conditioner suspended if the trial errors.
            os.kill(fault_pid, signal.SIGCONT)

        if zero_wall is None or displacement_limit_exceeded:
            self.publish_request(0.0)
            self.spin_for(0.50, 0.0)
            zero_truth = self.get_truth()
        stop_records, settled = self.settle(3.0)
        final_truth = stop_records[-1] if stop_records else self.get_truth()
        initial_yaw = start_truth["yaw_rad"]
        stop_dx = final_truth["x_m"]-zero_truth["x_m"]
        stop_dy = final_truth["y_m"]-zero_truth["y_m"]
        stop_distance = stop_dx*math.cos(initial_yaw)+stop_dy*math.sin(initial_yaw)
        drive_dx = final_truth["x_m"]-start_truth["x_m"]
        drive_dy = final_truth["y_m"]-start_truth["y_m"]
        total_distance = drive_dx*math.cos(initial_yaw)+drive_dy*math.sin(initial_yaw)
        max_linear = max((abs(v["linear_x_mps"]) for v in self.output_samples), default=0.0)
        reaction_s = None if zero_wall is None else zero_wall-pause_wall
        safety_reaction_s = (None if invalid_wall is None or zero_wall is None
                             else zero_wall-invalid_wall)
        stopping_limit_m = 0.00298 + 0.10
        fault_specific_confirmed = (
            fault_name != "rgb_perception"
            or (stale_invalid_wall is not None and stale_zero_wall is not None
                and 0.0 <= stale_zero_wall-pause_wall <= 0.55)
        )
        trial_pass = (
            fault_specific_confirmed
            and any(v["linear_x_mps"] > 0.005 for v in drive_outputs)
            and invalid_wall is not None and zero_wall is not None
            and reaction_s is not None and 0.0 <= reaction_s <= 0.55
            and not displacement_limit_exceeded
            and 0.01 < total_distance <= 0.50
            and stop_distance <= stopping_limit_m
            and settled
        )
        report = {
            "trial_pass": trial_pass,
            "classification": f"GUARDED_S00_FORWARD_TRIAL_WITH_INJECTED_{fault_name.upper()}_PROCESS_LOSS",
            "runtime_received_no_simulator_truth": True,
            "evaluator_service": "/gzserver/get_entity_state",
            "requested_topic": "/cmd_vel_nav",
            "authoritative_topic": "/cmd_vel",
            "fault_process_id": fault_pid,
            "fault_name": fault_name,
            "gate_at_start": gate_at_start,
            "start_truth": start_truth,
            "before_lidar_loss_truth": before_loss_truth,
            "loss_injection_wall_s": pause_wall,
            "safety_invalid_wall_s": invalid_wall,
            "safety_state_at_first_invalid": invalid_safety_state,
            "rgb_stale_confirmation_wall_s": stale_invalid_wall,
            "safety_state_at_rgb_stale_confirmation": stale_safety_state,
            "rgb_stale_guard_zero_wall_s": stale_zero_wall,
            "rgb_stale_zero_output_truth": stale_zero_truth,
            "fault_specific_confirmed": fault_specific_confirmed,
            "first_guard_zero_wall_s": zero_wall,
            "guard_reaction_delay_s": reaction_s,
            "guard_delay_after_gate_invalid_s": safety_reaction_s,
            "zero_output_truth": zero_truth,
            "final_truth": final_truth,
            "total_forward_displacement_m": total_distance,
            "post_zero_stopping_distance_m": stop_distance,
            "post_zero_stopping_limit_m": stopping_limit_m,
            "settled": settled,
            "displacement_limit_exceeded": displacement_limit_exceeded,
            "max_linear_output_mps": max_linear,
            "nonzero_output_samples": sum(abs(v["linear_x_mps"])>1e-8
                                           for v in self.output_samples),
            "output_samples": self.output_samples,
            "stop_truth_samples": stop_records,
            "safety_state_at_end": self.safety,
            "limit": "single short empty-arena trial; Phase 8B measured-stop distance plus 0.10 m margin",
        }
        return report


def main():
    if len(sys.argv) < 3:
        raise SystemExit("usage: phase8c_motion_trial.py FAULT_PID OUTPUT_JSON [FAULT_NAME]")
    fault_pid = int(sys.argv[1])
    output_path = Path(sys.argv[2])
    fault_name = sys.argv[3] if len(sys.argv) > 3 else "lidar_conditioner"
    rclpy.init()
    node = GuardedMotionTrial()
    result = {"status": "FAIL"}
    try:
        if not node.truth_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("evaluator_truth_service_missing")
        report = node.run(fault_pid, fault_name)
        result = {"status": "PASS" if report["trial_pass"] else "FAIL", **report}
    except Exception as exc:
        result = {"status": "FAIL", "reason": str(exc), "safety_state": node.safety}
        node.publish_request(0.0)
        for _ in range(10):
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps(result, indent=2, sort_keys=True)+"\n")
        print(json.dumps(result, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
