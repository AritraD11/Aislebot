#!/usr/bin/env python3
"""
AisleBot Keyboard Teleop v3.0
==============================
Keyboard-based velocity commander for testing without a physical joystick.
Publishes /cmd_vel (Twist) consumed by mecanum_teleop_asymmetric.

Controls:
  W/S = forward/backward    A/D = strafe left/right
  Q/E = rotate left/right   SPACE = stop
  1/2/3 = slow/normal/fast  X = emergency stop & quit
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty
import select


INSTRUCTIONS = """
╔══════════════════════════════════════╗
║     AisleBot Keyboard Teleop        ║
╠══════════════════════════════════════╣
║   W = Forward     S = Backward      ║
║   A = Strafe L    D = Strafe R      ║
║   Q = Rotate L    E = Rotate R      ║
║   SPACE = Stop                      ║
║   1 = Slow  2 = Normal  3 = Fast    ║
║   X = Quit                          ║
╚══════════════════════════════════════╝
"""

SPEED_PRESETS = {
    '1': (0.05, 0.15),   # slow: lin, ang
    '2': (0.15, 0.30),   # normal
    '3': (0.30, 0.60),   # fast
}


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')

        self.declare_parameter('max_linear', 0.30)
        self.declare_parameter('max_angular', 0.60)

        self.max_lin = self.get_parameter('max_linear').value
        self.max_ang = self.get_parameter('max_angular').value

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_cb)  # 10Hz

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.running = True

        # Terminal settings
        self.settings = termios.tcgetattr(sys.stdin)

        print(INSTRUCTIONS)
        print(f'Speed: linear={self.max_lin:.2f} m/s, angular={self.max_ang:.2f} rad/s')

    def get_key(self, timeout=0.1):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def timer_cb(self):
        key = self.get_key()

        if key == 'w':
            self.vx = self.max_lin
            self.vy = 0.0
            self.wz = 0.0
        elif key == 's':
            self.vx = -self.max_lin
            self.vy = 0.0
            self.wz = 0.0
        elif key == 'a':
            self.vx = 0.0
            self.vy = self.max_lin
            self.wz = 0.0
        elif key == 'd':
            self.vx = 0.0
            self.vy = -self.max_lin
            self.wz = 0.0
        elif key == 'q':
            self.vx = 0.0
            self.vy = 0.0
            self.wz = self.max_ang
        elif key == 'e':
            self.vx = 0.0
            self.vy = 0.0
            self.wz = -self.max_ang
        elif key == ' ':
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
        elif key in SPEED_PRESETS:
            self.max_lin, self.max_ang = SPEED_PRESETS[key]
            names = {'1': 'SLOW', '2': 'NORMAL', '3': 'FAST'}
            print(f'\r{names[key]}: linear={self.max_lin:.2f} angular={self.max_ang:.2f}  ')
        elif key == 'x':
            self.vx = 0.0
            self.vy = 0.0
            self.wz = 0.0
            self.publish_cmd()
            self.running = False
            raise KeyboardInterrupt
        elif key == '':
            # No key pressed — coast to stop
            self.vx *= 0.5
            self.vy *= 0.5
            self.wz *= 0.5
            if abs(self.vx) < 0.005:
                self.vx = 0.0
            if abs(self.vy) < 0.005:
                self.vy = 0.0
            if abs(self.wz) < 0.005:
                self.wz = 0.0

        self.publish_cmd()

    def publish_cmd(self):
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.angular.z = self.wz
        self.pub.publish(msg)
        if abs(self.vx) > 0.01 or abs(self.vy) > 0.01 or abs(self.wz) > 0.01:
            print(f'\rvx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f}  ', end='')

    def destroy_node(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
