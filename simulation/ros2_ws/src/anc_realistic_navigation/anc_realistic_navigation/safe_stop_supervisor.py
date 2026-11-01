"""Phase 8 hard-locked stop supervisor; sole /cmd_vel publisher."""
import json
import math
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

class SafeStopSupervisor(Node):
    # Feasibility failed; deliberately not a launch-time option.
    MOTION_AUTHORIZED = False

    def __init__(self):
        super().__init__("anc_realistic_safe_stop_supervisor")
        self.declare_parameter("requested_topic", "/cmd_vel/requested")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("state_topic", "/realistic/navigation_state")
        self.declare_parameter("supervisor_state_topic", "/realistic/supervisor_state")
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("command_timeout_s", 0.30)
        self.requested_topic = str(self.get_parameter("requested_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.state_topic = str(self.get_parameter("state_topic").value)
        self.supervisor_state_topic = str(self.get_parameter("supervisor_state_topic").value)
        self.timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.last_request_wall = None
        self.last_request = None
        self.request_count = 0
        self.map_state = None
        self.map_state_wall = None
        self.output_count = 0
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.state_pub = self.create_publisher(String, self.supervisor_state_topic, 10)
        self.create_subscription(Twist, self.requested_topic, self.on_request, 10)
        self.create_subscription(String, self.state_topic, self.on_map_state, 10)
        self.create_timer(1.0/float(self.get_parameter("publish_rate_hz").value), self.tick)
        self.get_logger().warning(
            "PHASE 8 MOTION LOCKED: only this node publishes /cmd_vel; all output is zero."
        )

    def on_request(self, msg):
        values = [msg.linear.x, msg.linear.y, msg.linear.z,
                  msg.angular.x, msg.angular.y, msg.angular.z]
        finite = all(math.isfinite(float(v)) for v in values)
        self.last_request = {
            "finite": finite,
            "linear_x_mps": float(msg.linear.x) if finite else None,
            "angular_z_radps": float(msg.angular.z) if finite else None,
            "nonzero": bool(finite and any(abs(float(v)) > 1e-9 for v in values)),
        }
        self.last_request_wall = time.monotonic()
        self.request_count += 1

    def on_map_state(self, msg):
        try:
            self.map_state = json.loads(msg.data)
        except (TypeError, json.JSONDecodeError):
            self.map_state = {"status": "INVALID_UNKNOWN",
                              "reason": "navigation_state_json_invalid",
                              "input_failures": ["invalid_state"]}
        self.map_state_wall = time.monotonic()

    def tick(self):
        # Unconditional zero: no Phase 8 request can become a motor command.
        self.cmd_pub.publish(Twist())
        self.output_count += 1
        now = time.monotonic()
        request_fresh = (self.last_request_wall is not None and
                         now-self.last_request_wall <= self.timeout_s)
        state_fresh = (self.map_state is not None and self.map_state_wall is not None and
                       now-self.map_state_wall <= 0.50)
        state = {
            "node": "anc_realistic_safe_stop_supervisor",
            "status": "STOP", "reason": "phase8_feasibility_gate_failed",
            "phase8_motion_lock": True, "motion_authorized": False,
            "requested_command_topic": self.requested_topic,
            "output_command_topic": self.output_topic,
            "requested_command_fresh": request_fresh,
            "requested_command_count": self.request_count,
            "latest_requested_command": self.last_request,
            "navigation_state_fresh": state_fresh, "navigation_state": self.map_state,
            "actual_output_zero": True,
            "actual_output": {"linear_x": 0.0, "linear_y": 0.0, "linear_z": 0.0,
                              "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0},
            "published_zero_commands": self.output_count,
            "watchdog_state": ("command_received_stop_only" if request_fresh
                               else "controller_command_timeout_stop"),
        }
        msg = String()
        msg.data = json.dumps(state, sort_keys=True, separators=(",", ":"))
        self.state_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SafeStopSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
