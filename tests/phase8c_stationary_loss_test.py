#!/usr/bin/env python3
"""Pause a required sensor-process while stationary and verify fail-closed recovery."""
import json
import os
import signal
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


PREFIXES = {
    "lidar_conditioner": ("scan_health", "scan_invalid", "scan_sim_",
                          "measured_scan_rate_", "usable_beam_",
                          "lidar_local_map_", "lidar_map_health_",
                          "lidar_map_invalid"),
    "rgb_perception": ("rgb_input:", "rgb_local_map_",
                        "rgb_perception_health_"),
    "ekf_localization": ("filtered_localization",),
}


class StationaryLoss(Node):
    def __init__(self):
        super().__init__("phase8c_stationary_loss_evaluator")
        self.safety = None
        self.outputs = []
        self.create_subscription(String, "/realistic/safety_state", self.on_safety, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)

    def on_safety(self, msg):
        try:
            self.safety = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.safety = {"valid": False, "failures": ["safety_json_invalid"]}

    def on_output(self, msg):
        self.outputs.append({"wall_s": time.monotonic(),
                             "linear_x_mps": float(msg.linear.x),
                             "angular_z_radps": float(msg.angular.z)})
        self.outputs = self.outputs[-800:]

    def spin_until(self, predicate, deadline):
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            if predicate():
                return True
        return bool(predicate())

    def wait_ready(self, timeout_s=6.0):
        deadline = time.monotonic()+timeout_s
        valid_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
            ready = (self.safety and self.safety.get("valid") is True
                     and float(self.safety.get("max_linear_speed_mps", 0.0)) >= 0.05)
            if ready:
                valid_since = valid_since or time.monotonic()
                if time.monotonic()-valid_since >= 0.30:
                    return dict(self.safety)
            else:
                valid_since = None
        raise RuntimeError("initial_gate_not_ready:"+
                           str(None if self.safety is None else self.safety.get("reason")))

    def run(self, pid, fault_name):
        initial = self.wait_ready()
        pause_wall = time.monotonic()
        invalid_state = None
        zero_after_invalid = False
        os.kill(pid, signal.SIGSTOP)
        try:
            deadline = pause_wall+2.5
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.03)
                failures = [] if not self.safety else [str(v) for v in
                                                       self.safety.get("failures", [])]
                if invalid_state is None and any(
                        value.startswith(PREFIXES[fault_name]) for value in failures):
                    invalid_state = {"wall_s": time.monotonic(), **dict(self.safety)}
                if invalid_state is not None and any(
                        sample["wall_s"] >= invalid_state["wall_s"]
                        and abs(sample["linear_x_mps"]) < 1e-8
                        and abs(sample["angular_z_radps"]) < 1e-8
                        for sample in self.outputs):
                    zero_after_invalid = True
                    break
        finally:
            try:
                os.kill(pid, signal.SIGCONT)
            except ProcessLookupError:
                pass
        recovered = self.spin_until(
            lambda: (self.safety and self.safety.get("valid") is True
                     and float(self.safety.get("max_linear_speed_mps", 0.0)) >= 0.05),
            time.monotonic()+6.0)
        all_zero = all(abs(v["linear_x_mps"]) < 1e-8
                       and abs(v["angular_z_radps"]) < 1e-8 for v in self.outputs)
        return {
            "classification": f"STATIONARY_{fault_name.upper()}_LOSS_AND_RECOVERY",
            "status": "PASS" if invalid_state and zero_after_invalid and recovered and all_zero else "FAIL",
            "runtime_received_no_simulator_truth": True,
            "fault_pid": pid,
            "fault_name": fault_name,
            "initial_gate": initial,
            "loss_injection_wall_s": pause_wall,
            "first_invalid_state": invalid_state,
            "zero_output_after_invalid": zero_after_invalid,
            "gate_recovered_valid": recovered,
            "all_authoritative_outputs_zero": all_zero,
            "output_sample_count": len(self.outputs),
            "safety_state_at_end": self.safety,
        }


def main():
    if len(sys.argv) != 4 or sys.argv[2] not in PREFIXES:
        raise SystemExit("usage: phase8c_stationary_loss_test.py PID {lidar_conditioner|rgb_perception|ekf_localization} OUTPUT_JSON")
    pid, fault_name = int(sys.argv[1]), sys.argv[2]
    output = Path(sys.argv[3])
    rclpy.init()
    node = StationaryLoss()
    result = {"status": "FAIL"}
    try:
        result = node.run(pid, fault_name)
    except Exception as exc:
        try:
            os.kill(pid, signal.SIGCONT)
        except ProcessLookupError:
            pass
        result = {"status": "FAIL", "reason": str(exc),
                  "safety_state": node.safety, "output_samples": node.outputs}
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True)+"\n")
        print(json.dumps({key: value for key, value in result.items()
                          if key != "output_samples"}, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
