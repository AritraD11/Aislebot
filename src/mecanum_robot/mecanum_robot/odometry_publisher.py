#!/usr/bin/env python3
"""
AisleBot Odometry Publisher v1.0
================================
Computes robot odometry from wheel encoder feedback using
AisleBot's asymmetric mecanum forward kinematics.

FORWARD KINEMATICS (wheel speeds → body velocity):
  vx = (r/4) * (w_fr + w_fl + w_rr + w_rl)
  vy = (r/4) * (w_fr - w_fl - w_rr + w_rl)
  wz = (r/4) * (w_fr/K_out - w_fl/K_in + w_rr/K_in - w_rl/K_out)

  (Simplified using pseudoinverse of asymmetric kinematics matrix)

TOPICS:
  Subscribes: /wheel_velocities_actual (Float64MultiArray) [FR,FL,RR,RL] rad/s
  Publishes:  /wheel_odom (Odometry) raw wheel odometry
  Publishes:  /tf (odom → base_link transform)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
import time


class OdometryPublisher(Node):

    def __init__(self):
        super().__init__('odometry_publisher')

        # Robot geometry
        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('l1', 0.403)
        self.declare_parameter('l2', 0.333)
        self.declare_parameter('d', 0.15769)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.r = self.get_parameter('wheel_radius').value
        self.l1 = self.get_parameter('l1').value
        self.l2 = self.get_parameter('l2').value
        self.d = self.get_parameter('d').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        self.K_outer = self.l1 + self.d
        self.K_inner = self.l2 + self.d

        # State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # ROS2
        self.sub = self.create_subscription(
            Float64MultiArray, 'wheel_velocities_actual',
            self.velocity_cb, 10)

        self.odom_pub = self.create_publisher(Odometry, 'wheel_odom', 50)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'Odometry publisher started | '
            f'K_out={self.K_outer:.4f} K_in={self.K_inner:.4f}')

    def velocity_cb(self, msg):
        if len(msg.data) < 4:
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        w_fr, w_fl, w_rr, w_rl = msg.data[0], msg.data[1], msg.data[2], msg.data[3]

        # Forward kinematics (asymmetric)
        vx = (self.r / 4.0) * (w_fr + w_fl + w_rr + w_rl)
        vy = (self.r / 4.0) * (w_fr - w_fl - w_rr + w_rl)

        # For rotation, use weighted formula accounting for asymmetry
        wz = (self.r / 4.0) * (
            w_fr / self.K_outer
            - w_fl / self.K_inner
            + w_rr / self.K_inner
            - w_rl / self.K_outer
        )

        # Integrate position (2nd order midpoint for better accuracy)
        half_dtheta = wz * dt * 0.5
        cos_mid = math.cos(self.theta + half_dtheta)
        sin_mid = math.sin(self.theta + half_dtheta)

        self.x += (vx * cos_mid - vy * sin_mid) * dt
        self.y += (vx * sin_mid + vy * cos_mid) * dt
        self.theta += wz * dt

        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Create quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Covariance (diagonal, tuned for mecanum)
        odom.pose.covariance[0] = 0.01   # x
        odom.pose.covariance[7] = 0.01   # y
        odom.pose.covariance[35] = 0.03  # yaw

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        odom.twist.covariance[0] = 0.01
        odom.twist.covariance[7] = 0.01
        odom.twist.covariance[35] = 0.03

        self.odom_pub.publish(odom)

        # Publish TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
