#!/usr/bin/env python3
"""Stationary probe of rotational clearance fail-closed behavior."""
import json
import math
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from simulation_interfaces.srv import GetEntityState
from std_msgs.msg import String


def parse_state(msg):
    try:
        return json.loads(msg.data)
    except Exception:
        return {"valid": False, "reason": "invalid_json"}


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


class Probe(Node):
    def __init__(self):
        super().__init__("phase8c_rotation_clearance_probe")
        self.safety = None
        self.outputs = []
        self.create_subscription(String, "/realistic/safety_state",
                                 lambda msg: setattr(self, "safety", parse_state(msg)), 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_output, 10)
        self.truth = self.create_client(GetEntityState, "/gzserver/get_entity_state")

    def on_output(self, msg):
        self.outputs.append({
            "wall_s": time.monotonic(),
            "linear_x_mps": float(msg.linear.x),
            "angular_z_radps": float(msg.angular.z),
        })

    def get_truth(self, timeout_s=4.0):
        if not self.truth.wait_for_service(timeout_sec=timeout_s):
            raise RuntimeError("truth_service_not_ready")
        req = GetEntityState.Request()
        req.entity = "anc_robot"
        fut = self.truth.call_async(req)
        deadline = time.monotonic()+timeout_s
        while rclpy.ok() and not fut.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.03)
        if not fut.done() or fut.result() is None:
            raise RuntimeError("truth_service_timeout")
        res = fut.result()
        if res.result.result != 1:
            raise RuntimeError("truth_error:"+res.result.error_message)
        s = res.state
        return {
            "sim_stamp_s": s.header.stamp.sec+s.header.stamp.nanosec*1e-9,
            "x_m": float(s.pose.position.x),
            "y_m": float(s.pose.position.y),
            "yaw_rad": yaw(s.pose.orientation),
        }

    def wait_for_gate(self, timeout_s=25.0):
        deadline = time.monotonic()+timeout_s
        stable_since = None
        last_signature = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            state = self.safety or {}
            rotation = state.get("rotation_clearance") or {}
            if (rotation.get("authorized") is False and
                rotation.get("reason") in {
                    "swept_disk_occupied", "swept_disk_unknown",
                    "swept_disk_outside_grid", "swept_disk_unknown"}):
                signature = (rotation.get("reason"), rotation.get("checked_cells"),
                             rotation.get("occupied_cells"), rotation.get("unknown_cells"))
                if signature == last_signature:
                    stable_since = stable_since or time.monotonic()
                    if time.monotonic()-stable_since >= 0.40:
                        return dict(state)
                else:
                    signature = signature
                    stable_since = time.monotonic()
                    last_signature = signature
            else:
                stable_since = None
                last_signature = None
        raise RuntimeError("no_stable_rotational_clearance_denial:"+json.dumps(self.safety or {}))


def main():
    if len(sys.argv) != 3:
        raise SystemExit("usage: probe.py OUT.json SCENARIO")
    output = Path(sys.argv[1])
    scenario = sys.argv[2]
    rclpy.init()
    node = Probe()
    error = None
    try:
        before = node.get_truth()
        gate = node.wait_for_gate()
        started = time.monotonic()
        deadline = started+1.2
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.03)
        after = node.get_truth()
        wall_elapsed = time.monotonic()-started
        actual_publishers = node.get_publishers_info_by_topic("/cmd_vel")
        max_v = max((abs(s["linear_x_mps"]) for s in node.outputs), default=0.0)
        max_w = max((abs(s["angular_z_radps"]) for s in node.outputs), default=0.0)
        result = {
            "scenario": scenario,
            "profile": "REALISTIC_RGB_LIDAR",
            "rotation_cap_radps": 0.10,
            "probe_published_no_commands": True,
            "gate_snapshot": gate,
            "cmd_vel_publisher_count": len(actual_publishers),
            "cmd_vel_publishers": [str(p.node_name) for p in actual_publishers],
            "observed_cmd_vel_samples": len(node.outputs),
            "max_abs_cmd_vel_linear_mps": max_v,
            "max_abs_cmd_vel_angular_radps": max_w,
            "before_truth_evaluator_only": before,
            "after_truth_evaluator_only": after,
            "wall_elapsed_s": wall_elapsed,
            "truth_xy_drift_m": math.hypot(after["x_m"]-before["x_m"],
                                           after["y_m"]-before["y_m"]),
            "truth_yaw_change_rad": math.atan2(
                math.sin(after["yaw_rad"]-before["yaw_rad"]),
                math.cos(after["yaw_rad"]-before["yaw_rad"])),
        }
        rotation = gate["rotation_clearance"]
        result["probe_pass"] = (
            gate.get("authorized") is False
            and gate.get("max_angular_speed_radps") == 0.0
            and rotation.get("authorized") is False
            and rotation.get("reason") in {
                "swept_disk_occupied", "swept_disk_unknown", "swept_disk_outside_grid"}
            and len(actual_publishers) == 1
            and max_v <= 1e-8 and max_w <= 1e-8
            and result["truth_xy_drift_m"] <= 0.002
            and abs(result["truth_yaw_change_rad"]) <= 0.002
        )
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        result = {"scenario": scenario, "probe_pass": False, "error": error}
    finally:
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, indent=2, sort_keys=True)+"\n")
        print(json.dumps({
            "scenario": scenario,
            "probe_pass": result.get("probe_pass"),
            "gate_authorized": (result.get("gate_snapshot") or {}).get("authorized"),
            "rotation_clearance": (result.get("gate_snapshot") or {}).get("rotation_clearance"),
            "cmd_vel_publisher_count": result.get("cmd_vel_publisher_count"),
            "max_abs_cmd_vel": [
                result.get("max_abs_cmd_vel_linear_mps"),
                result.get("max_abs_cmd_vel_angular_radps")],
            "truth_xy_drift_m": result.get("truth_xy_drift_m"),
            "truth_yaw_change_rad": result.get("truth_yaw_change_rad"),
            "error": error,
        }, indent=2, sort_keys=True))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if result.get("probe_pass") else 1)


if __name__ == "__main__":
    main()