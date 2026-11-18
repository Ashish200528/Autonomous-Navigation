"""Simulation-only: walk the crossing pedestrian over the zebra crossing and back.

Closed loop on the model's own odometry (world pose from Gazebo's OdometryPublisher), so
it never drifts off the crossing: pause, turn to face the far kerb, walk there, repeat.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

KERBS = [(-0.3, -4.15), (-0.3, -1.85)]  # south sidewalk, infield edge (world, metres)
WALK_SPEED = 0.28   # 1.4 m/s real walking speed at 1:5 scale
PAUSE_S = 9.0


class PedestrianSim(Node):
    def __init__(self):
        super().__init__("pedestrian_sim")
        self.pub = self.create_publisher(Twist, "/model/pedestrian/cmd_vel", 10)
        self.create_subscription(Odometry, "/model/pedestrian/odometry", self.on_odom, 10)
        self.pose = None
        self.target = 1          # walking towards KERBS[target]
        self.pause_until = None  # sim time; starts with a pause so the car meets a full cycle
        self.create_timer(0.05, self.tick)

    def on_odom(self, msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        self.pose = (p.x, p.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2)))

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.pose is None or now == 0.0:
            return
        if self.pause_until is None:
            self.pause_until = now + PAUSE_S
        cmd = Twist()
        if now >= self.pause_until:
            x, y, yaw = self.pose
            tx, ty = KERBS[self.target]
            dist = math.hypot(tx - x, ty - y)
            if dist < 0.05:
                self.target = 1 - self.target
                self.pause_until = now + PAUSE_S
            else:
                err = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw),
                                 math.cos(math.atan2(ty - y, tx - x) - yaw))
                cmd.angular.z = max(-1.5, min(1.5, 3.0 * err))
                if abs(err) < 0.3:  # face the kerb first, then walk
                    cmd.linear.x = min(WALK_SPEED, 1.5 * dist)
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = PedestrianSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
