#!/usr/bin/env python3
"""Attach explicit provisional velocity covariance to Gazebo wheel odometry.

The Gazebo DiffDrive message currently reports an all-zero covariance matrix.
This node preserves the measured values and simulation timestamp, marks pose as
unused, and assigns documented velocity variances before the EKF consumes it.
It never queries simulator state or publishes a pose estimate.
"""
import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class WheelOdometryCovariance(Node):
    def __init__(self):
        super().__init__("anc_wheel_odometry_covariance")
        self.declare_parameter("input_topic", "/odom")
        self.declare_parameter("output_topic", "/odometry/wheel")
        self.declare_parameter("linear_x_variance_m2ps2", 0.0004)
        self.declare_parameter("linear_y_variance_m2ps2", 0.01)
        self.declare_parameter("unused_variance", 1000000.0)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.var_x = float(self.get_parameter("linear_x_variance_m2ps2").value)
        self.var_y = float(self.get_parameter("linear_y_variance_m2ps2").value)
        self.unused_variance = float(self.get_parameter("unused_variance").value)
        if not all(math.isfinite(v) and v > 0.0 for v in
                   (self.var_x, self.var_y, self.unused_variance)):
            raise ValueError("wheel odometry variances must be finite and positive")

        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.on_odometry, 10
        )
        self.count = 0

    def on_odometry(self, source):
        message = copy.deepcopy(source)
        message.pose.covariance = [0.0] * 36
        message.twist.covariance = [0.0] * 36
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = self.unused_variance
            message.twist.covariance[index] = self.unused_variance
        message.twist.covariance[0] = self.var_x
        message.twist.covariance[7] = self.var_y
        self.publisher.publish(message)
        self.count += 1
        if self.count == 1:
            self.get_logger().info(
                f"Relaying {self.input_topic} to {self.output_topic}; "
                f"stamp/frame preserved, pose disabled, vx variance={self.var_x:g}, "
                f"vy variance={self.var_y:g}."
            )


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
