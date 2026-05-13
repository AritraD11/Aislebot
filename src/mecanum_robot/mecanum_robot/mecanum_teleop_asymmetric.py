#!/usr/bin/env python3
"""
AisleBot Mecanum Teleop (Asymmetric Kinematics) v3.0
=====================================================
Converts body velocity commands to individual wheel angular velocities
using AisleBot's asymmetric mecanum inverse kinematics.

GEOMETRY (from IIT Bombay paper):
  l1 = 0.403 m  (outer wheels: FR, RL)
  l2 = 0.333 m  (inner wheels: FL, RR)
  d  = 0.15769 m (half-track width)
  r  = 0.0762 m  (wheel radius, DekuPro 6-inch)
  K_outer = l1 + d = 0.5607 m
  K_inner = l2 + d = 0.4907 m

INVERSE KINEMATICS:
  w_FR = (1/r) * (vx + vy + K_outer * wz)   [outer]
  w_FL = (1/r) * (vx - vy - K_inner * wz)   [inner]
  w_RR = (1/r) * (vx - vy + K_inner * wz)   [inner]
  w_RL = (1/r) * (vx + vy - K_outer * wz)   [outer]

TOPICS:
  Subscribes: /cmd_vel (Twist)       from Nav2 or teleop_twist_keyboard
  Subscribes: /joy     (Joy)         from joystick driver
  Publishes:  /wheel_speeds (Float64MultiArray) [FR, FL, RR, RL] rad/s
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
import math


class MecanumTeleopAsymmetric(Node):

    def __init__(self):
        super().__init__('mecanum_teleop_asymmetric')

        # === Robot geometry parameters ===
        self.declare_parameter('wheel_radius', 0.0762)
        self.declare_parameter('l1', 0.403)       # outer wheel distance
        self.declare_parameter('l2', 0.333)       # inner wheel distance
        self.declare_parameter('d', 0.15769)       # half-track width
        self.declare_parameter('max_linear', 0.48)  # m/s
        self.declare_parameter('max_angular', 1.0)  # rad/s
        self.declare_parameter('max_wheel_speed', 6.28)  # rad/s clamp
        self.declare_parameter('input_source', 'cmd_vel')  # 'cmd_vel' or 'joy'

        self.r = self.get_parameter('wheel_radius').value
        self.l1 = self.get_parameter('l1').value
        self.l2 = self.get_parameter('l2').value
        self.d = self.get_parameter('d').value
        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value
        self.max_wheel = self.get_parameter('max_wheel_speed').value
        self.input_source = self.get_parameter('input_source').value

        # Derived constants
        self.K_outer = self.l1 + self.d  # 0.5607
        self.K_inner = self.l2 + self.d  # 0.4907
        self.inv_r = 1.0 / self.r

        # === ROS2 ===
        self.pub = self.create_publisher(Float64MultiArray, 'wheel_speeds', 10)

        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(Joy, 'joy', self.joy_cb, 10)

        self.get_logger().info(
            f'Teleop Asymmetric started | r={self.r} l1={self.l1} l2={self.l2} d={self.d} '
            f'K_out={self.K_outer:.4f} K_in={self.K_inner:.4f}')

    def compute_wheel_speeds(self, vx, vy, wz):
        """Asymmetric inverse kinematics: body vel → 4 wheel angular velocities."""
        w_fr = self.inv_r * (vx + vy + self.K_outer * wz)
        w_fl = self.inv_r * (vx - vy - self.K_inner * wz)
        w_rr = self.inv_r * (vx - vy + self.K_inner * wz)
        w_rl = self.inv_r * (vx + vy - self.K_outer * wz)

        # Clamp to max wheel speed
        speeds = [w_fr, w_fl, w_rr, w_rl]
        max_abs = max(abs(s) for s in speeds)
        if max_abs > self.max_wheel:
            scale = self.max_wheel / max_abs
            speeds = [s * scale for s in speeds]

        return speeds

    def publish_speeds(self, speeds):
        msg = Float64MultiArray()
        msg.data = speeds
        self.pub.publish(msg)

    def cmd_vel_cb(self, msg):
        """Handle /cmd_vel from Nav2 or teleop_twist_keyboard."""
        vx = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        vy = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        wz = max(-self.max_ang, min(self.max_ang, msg.angular.z))

        speeds = self.compute_wheel_speeds(vx, vy, wz)
        self.publish_speeds(speeds)

        if abs(vx) > 0.01 or abs(vy) > 0.01 or abs(wz) > 0.01:
            self.get_logger().debug(
                f'cmd_vel [{vx:.2f},{vy:.2f},{wz:.2f}] → '
                f'wheels [FR:{speeds[0]:.1f},FL:{speeds[1]:.1f},'
                f'RR:{speeds[2]:.1f},RL:{speeds[3]:.1f}]',
                throttle_duration_sec=0.5)

    def joy_cb(self, msg):
        """Handle /joy from physical joystick."""
        if len(msg.axes) < 4:
            return
        # Standard mapping: axes[0]=leftX(strafe), axes[1]=leftY(fwd),
        #                    axes[3]=rightX(rotate)
        vx = msg.axes[1] * self.max_lin
        vy = -msg.axes[0] * self.max_lin   # negate for left=positive
        wz = msg.axes[3] * self.max_ang

        # Apply deadzone
        if abs(vx) < 0.02:
            vx = 0.0
        if abs(vy) < 0.02:
            vy = 0.0
        if abs(wz) < 0.02:
            wz = 0.0

        speeds = self.compute_wheel_speeds(vx, vy, wz)
        self.publish_speeds(speeds)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumTeleopAsymmetric()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
