"""Generic guarded cmd_vel authority with command and safety watchdogs."""
import json
import time
import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_msgs.msg import String
from anc_navigation_common.safety import differential_drive_command, guarded_command


class SafetyCommandGuard(Node):
    def __init__(self):
        super().__init__("anc_safety_command_guard")
        defaults = {
            "requested_topic": "/cmd_vel/requested",
            "safety_state_topic": "/realistic/safety_state",
            "output_topic": "/cmd_vel",
            "guard_state_topic": "/realistic/command_guard_state",
            "publish_rate_hz": 20.0,
            "command_timeout_s": 0.30,
            "safety_timeout_s": 0.25,
            "hard_max_linear_speed_mps": 0.25,
            "hard_max_angular_speed_radps": 0.60,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.command_timeout_s = float(p["command_timeout_s"])
        self.safety_timeout_s = float(p["safety_timeout_s"])
        self.hard_max_linear = float(p["hard_max_linear_speed_mps"])
        self.hard_max_angular = float(p["hard_max_angular_speed_radps"])
        self.request = self.request_wall = self.safety_state = self.safety_wall = None
        self.request_malformed = False
        self.command_count = self.output_count = 0
        self.output = self.create_publisher(Twist, str(p["output_topic"]), 10)
        self.state = self.create_publisher(String, str(p["guard_state_topic"]), 10)
        self.create_subscription(Twist, str(p["requested_topic"]), self.on_request, 10)
        self.create_subscription(String, str(p["safety_state_topic"]),
                                 self.on_safety_state, 10)
        self.create_timer(1.0 / float(p["publish_rate_hz"]), self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.get_logger().warning(
            "Guard owns authoritative cmd_vel; stale or invalid inputs produce zero."
        )

    def on_request(self, msg):
        self.request = differential_drive_command(msg)
        self.request_malformed = self.request is None
        self.request_wall = time.monotonic()
        self.command_count += 1

    def on_safety_state(self, msg):
        try:
            value = json.loads(msg.data)
            if not isinstance(value, dict):
                raise ValueError("safety_state_not_object")
            self.safety_state = value
        except (TypeError, ValueError, json.JSONDecodeError):
            self.safety_state = {"valid": False, "reason": "safety_state_malformed"}
        self.safety_wall = time.monotonic()

    def tick(self):
        now = time.monotonic()
        req_age = None if self.request_wall is None else now - self.request_wall
        state_age = None if self.safety_wall is None else now - self.safety_wall
        decision = guarded_command(
            request=self.request, request_age_s=float("inf") if req_age is None else req_age,
            safety_state=self.safety_state,
            safety_age_s=float("inf") if state_age is None else state_age,
            command_timeout_s=self.command_timeout_s,
            safety_timeout_s=self.safety_timeout_s,
            hard_max_linear_speed_mps=self.hard_max_linear,
            hard_max_angular_speed_radps=self.hard_max_angular,
        )
        msg = Twist()
        if decision["authorized"]:
            msg.linear.x = decision["linear_x_mps"]
            msg.angular.z = decision["angular_z_radps"]
        self.output.publish(msg)
        self.output_count += 1
        state = {
            "node": "anc_safety_command_guard",
            "authorized": decision["authorized"],
            "reason": ("malformed_differential_drive_command"
                       if self.request_malformed else decision["reason"]),
            "requested": self.request,
            "output": {"linear_x_mps": float(msg.linear.x),
                       "angular_z_radps": float(msg.angular.z),
                       "is_zero": abs(msg.linear.x) < 1e-9 and abs(msg.angular.z) < 1e-9},
            "command_age_s": req_age, "safety_state_age_s": state_age,
            "command_count": self.command_count, "output_count": self.output_count,
            "safety_state": self.safety_state,
        }
        out = String()
        out.data = json.dumps(state, sort_keys=True, separators=(",", ":"))
        self.state.publish(out)

    def stop(self):
        self.output.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = SafetyCommandGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
