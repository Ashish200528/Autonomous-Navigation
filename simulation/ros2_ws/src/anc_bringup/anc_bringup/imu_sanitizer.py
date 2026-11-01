#!/usr/bin/env python3
"""Publish a navigation-safe IMU without Gazebo's truth-derived orientation.

Gazebo Harmonic provides a perfect pose-derived quaternion for its simulated
IMU. This adapter intentionally drops that field. Consumers must treat
orientation as unavailable and use only the configured noisy gyro/acceleration
measurements on /imu/data.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuSanitizer(Node):
    def __init__(self):
        super().__init__("anc_imu_sanitizer")
        self.declare_parameter("publish_enabled", True)
        self.publisher = self.create_publisher(
            Imu, "/imu/data", qos_profile_sensor_data
        )
        self.subscription = self.create_subscription(
            Imu, "/anc_internal/imu_raw", self.on_imu,
            qos_profile_sensor_data,
        )
        self.count = 0

    def on_imu(self, source):
        # Runtime disable is used by Phase 4 dropout tests; it never alters data.
        if not bool(self.get_parameter("publish_enabled").value):
            return
        message = Imu()
        message.header = source.header
        message.orientation.x = 0.0
        message.orientation.y = 0.0
        message.orientation.z = 0.0
        message.orientation.w = 0.0
        message.orientation_covariance[0] = -1.0
        message.angular_velocity = source.angular_velocity
        message.angular_velocity_covariance = source.angular_velocity_covariance
        message.linear_acceleration = source.linear_acceleration
        message.linear_acceleration_covariance = source.linear_acceleration_covariance
        self.publisher.publish(message)
        self.count += 1
        if self.count == 1:
            self.get_logger().info(
                "Publishing /imu/data with orientation unavailable; "
                "retaining simulated gyro and acceleration."
            )


def main(args=None):
    rclpy.init(args=args)
    node = ImuSanitizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
