"""Limit Nav2 commands to the provisional car envelope and stop on command loss."""
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelGuard(Node):
    def __init__(self):
        super().__init__("anc_navigation_cmd_vel_guard")
        self.declare_parameter("max_linear_x_mps", 0.25)
        self.declare_parameter("max_angular_z_radps", 1.0)
        self.declare_parameter("command_timeout_s", 0.40)
        self.max_linear = float(self.get_parameter("max_linear_x_mps").value)
        self.max_angular = float(self.get_parameter("max_angular_z_radps").value)
        self.timeout = float(self.get_parameter("command_timeout_s").value)
        self.last_command_time = 0.0
        self.last_was_zero = True
        self.publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Twist, "/cmd_vel_nav", self.on_command, 10)
        self.create_timer(0.10, self.watchdog)
        self.get_logger().info(
            f"Nav2 command guard active: |linear.x|<={self.max_linear:.3f} m/s, "
            f"|angular.z|<={self.max_angular:.3f} rad/s, timeout={self.timeout:.2f}s"
        )

    def on_command(self, message):
        output = Twist()
        output.linear.x = max(-self.max_linear, min(self.max_linear, message.linear.x))
        output.angular.z = max(-self.max_angular, min(self.max_angular, message.angular.z))
        if not math.isfinite(output.linear.x) or not math.isfinite(output.angular.z):
            output = Twist()
        self.publisher.publish(output)
        self.last_command_time = time.monotonic()
        self.last_was_zero = (abs(output.linear.x) < 1e-9 and
                              abs(output.angular.z) < 1e-9)

    def watchdog(self):
        if (self.last_command_time == 0.0 or
                time.monotonic() - self.last_command_time > self.timeout):
            if not self.last_was_zero:
                self.publisher.publish(Twist())
                self.last_was_zero = True


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
