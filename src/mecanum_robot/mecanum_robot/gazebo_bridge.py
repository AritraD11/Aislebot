#!/usr/bin/env python3
"""
AisleBot Gazebo Bridge v3.0
============================
Converts /wheel_speeds to /cmd_vel Twist for Gazebo planar_move plugin.
Uses asymmetric forward kinematics to compute body velocity from wheel speeds.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


class GazeboBridge(Node):

    def __init__(self):
        super().__init__('gazebo_bridge')

        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('l1', 0.403)
        self.declare_parameter('l2', 0.333)
        self.declare_parameter('d', 0.15769)

        self.r = self.get_parameter('wheel_radius').value
        self.l1 = self.get_parameter('l1').value
        self.l2 = self.get_parameter('l2').value
        self.d = self.get_parameter('d').value
        self.K_outer = self.l1 + self.d
        self.K_inner = self.l2 + self.d

        self.sub = self.create_subscription(
            Float64MultiArray, 'wheel_speeds', self.wheel_cb, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.get_logger().info('Gazebo Bridge started (asymmetric FK)')

    def wheel_cb(self, msg):
        if len(msg.data) < 4:
            return
        w_fr, w_fl, w_rr, w_rl = msg.data

        # Asymmetric forward kinematics
        vx = (self.r / 4.0) * (w_fr + w_fl + w_rr + w_rl)
        vy = (self.r / 4.0) * (w_fr - w_fl - w_rr + w_rl)
        wz = (self.r / 4.0) * (
            w_fr / self.K_outer
            - w_fl / self.K_inner
            + w_rr / self.K_inner
            - w_rl / self.K_outer)

        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        self.pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = GazeboBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
