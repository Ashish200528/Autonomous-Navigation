"""Simulation-only adapter: Gazebo ray cone -> sensor_msgs/Range like an HC-SR04 driver.

On the Raspberry Pi this node is replaced by the GPIO driver publishing the same topic.
"""
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range


class UltrasonicSim(Node):
    def __init__(self):
        super().__init__("ultrasonic_sim")
        self.fov = self.declare_parameter("field_of_view_rad", 0.26).value
        self.pub = self.create_publisher(Range, "/ultrasonic/range", 10)
        self.create_subscription(LaserScan, "/ultrasonic/scan", self.on_scan, 10)

    def on_scan(self, scan):
        hits = [r for r in scan.ranges if math.isfinite(r) and scan.range_min <= r <= scan.range_max]
        msg = Range()
        msg.header = scan.header
        msg.header.frame_id = "ultrasonic_link"
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = float(self.fov)
        msg.min_range, msg.max_range = scan.range_min, scan.range_max
        msg.range = min(hits) if hits else math.inf  # REP-117: +inf = nothing in range
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = UltrasonicSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
