#!/usr/bin/env python3
"""
joy_to_aislebot.py — single point that turns USB gamepad input into the
two control streams the rest of the stack already understands.

Subscribes
    /joy           sensor_msgs/Joy

Publishes
    /cmd_vel       geometry_msgs/Twist  — drive (chassis)
    /arm/cmd_vel   geometry_msgs/Twist  — arm (continuous)
    /arm/command   std_msgs/String      — arm (discrete: HOME/ESTOP/CLEAR)

DEFAULT MAPPING (Xbox / PS-style controllers, axes_layout = 'xbox'):
    Left stick  X  →  strafe   (vy, +ve = left)
    Left stick  Y  →  forward  (vx, +ve = forward)
    Right stick X  →  rotate   (wz, +ve = CCW)
    Right stick Y  →  lift     (+ve = UP)
    Right trigger  →  arm OPEN  (analog)
    Left trigger   →  arm CLOSE (analog)
    Y / Triangle button  →  HOME
    B / Circle   button  →  ESTOP
    Start        button  →  CLEAR

Dead-band, max-speed and axis indices are all parameters — remap without
editing code. To switch controller layout, change `axes_layout` and the
button/axis indices.

IIT Bombay | Aritra Das (25D0074) | May 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class JoyToAislebot(Node):
    def __init__(self):
        super().__init__('joy_to_aislebot')

        # ── Speed limits (must match teleop_asym + ESP32 firmware) ──
        self.declare_parameter('max_linear',  0.15)   # m/s
        self.declare_parameter('max_angular', 0.30)   # rad/s
        self.declare_parameter('deadzone',    0.10)

        # ── Axis indices (Xbox layout, ROS2 joy package) ────────────
        # Left stick:  axes[0]=LX, axes[1]=LY
        # Right stick: axes[3]=RX, axes[4]=RY  (Xbox); on PS some pads
        #              put RX on 2 and RY on 3 — adjust here if needed.
        self.declare_parameter('axis_lx', 0)
        self.declare_parameter('axis_ly', 1)
        self.declare_parameter('axis_rx', 3)
        self.declare_parameter('axis_ry', 4)
        # Triggers: Xbox = axes[2]=LT, axes[5]=RT (resting +1, fully = -1)
        # joy_node maps them as float in [-1,+1]; we'll convert to [0,1].
        self.declare_parameter('axis_lt', 2)
        self.declare_parameter('axis_rt', 5)

        # ── Buttons (Xbox layout) ───────────────────────────────────
        # 0=A, 1=B, 2=X, 3=Y, 4=LB, 5=RB, 6=BACK, 7=START
        self.declare_parameter('btn_home',  3)   # Y
        self.declare_parameter('btn_estop', 1)   # B
        self.declare_parameter('btn_clear', 7)   # START

        # ── Optional dead-man switch (LB) — drive only fires when held ─
        self.declare_parameter('use_deadman', False)
        self.declare_parameter('btn_deadman', 4)   # LB

        # ── Cache parameter values ──────────────────────────────────
        p = lambda k: self.get_parameter(k).value
        self.max_lin   = p('max_linear')
        self.max_ang   = p('max_angular')
        self.deadzone  = p('deadzone')
        self.axes_idx  = {
            'lx': p('axis_lx'), 'ly': p('axis_ly'),
            'rx': p('axis_rx'), 'ry': p('axis_ry'),
            'lt': p('axis_lt'), 'rt': p('axis_rt'),
        }
        self.btn_home  = p('btn_home')
        self.btn_estop = p('btn_estop')
        self.btn_clear = p('btn_clear')
        self.use_deadman = p('use_deadman')
        self.btn_deadman = p('btn_deadman')

        # Edge detection — only fire button events on press, not hold
        self.prev_buttons = []

        # Subscribers / publishers
        self.create_subscription(Joy, '/joy', self.cb_joy, 10)
        self.pub_drive    = self.create_publisher(Twist,  '/cmd_vel',     10)
        self.pub_arm_vel  = self.create_publisher(Twist,  '/arm/cmd_vel', 10)
        self.pub_arm_cmd  = self.create_publisher(String, '/arm/command', 10)

        self.get_logger().info('joy_to_aislebot ready (Xbox layout)')

    # ─────────────────────────────────────────────────────────────
    @staticmethod
    def _dz(v: float, dz: float) -> float:
        if abs(v) < dz:
            return 0.0
        # Re-scale so output remains in [-1, 1] after deadzone removal
        sign = 1.0 if v > 0 else -1.0
        return sign * (abs(v) - dz) / (1.0 - dz)

    @staticmethod
    def _trigger_to_unit(t: float) -> float:
        # joy_node convention: triggers rest at +1, fully pulled = -1
        # Map to 0..1 (rest = 0, full pull = 1).
        return max(0.0, min(1.0, (1.0 - t) * 0.5))

    def _safe_axis(self, axes, idx, default=0.0):
        return float(axes[idx]) if 0 <= idx < len(axes) else default

    def _safe_button(self, buttons, idx):
        return int(buttons[idx]) if 0 <= idx < len(buttons) else 0

    def _just_pressed(self, buttons, idx):
        if idx >= len(buttons): return False
        if idx >= len(self.prev_buttons): return bool(buttons[idx])
        return buttons[idx] == 1 and self.prev_buttons[idx] == 0

    # ─────────────────────────────────────────────────────────────
    def cb_joy(self, msg: Joy):
        axes, buttons = msg.axes, msg.buttons

        # ── DRIVE ──────────────────────────────────────────────
        lx = self._dz(self._safe_axis(axes, self.axes_idx['lx']), self.deadzone)
        ly = self._dz(self._safe_axis(axes, self.axes_idx['ly']), self.deadzone)
        rx = self._dz(self._safe_axis(axes, self.axes_idx['rx']), self.deadzone)

        deadman_ok = (not self.use_deadman or
                      self._safe_button(buttons, self.btn_deadman) == 1)

        drive = Twist()
        if deadman_ok:
            drive.linear.x  =  ly * self.max_lin   # forward
            drive.linear.y  =  lx * self.max_lin   # strafe (+ve = left)
            drive.angular.z =  rx * self.max_ang   # rotation (+ve CCW)
        # Always publish: zeros from idle gamepad become an explicit "stop"
        self.pub_drive.publish(drive)

        # ── ARM VELOCITY ───────────────────────────────────────
        ry = self._dz(self._safe_axis(axes, self.axes_idx['ry']), self.deadzone)
        # Trigger axes default to 0.0 if controller hasn't been "warmed up"
        # by the user (joy_node only knows after first movement). That's OK
        # — they read 0 = rest, 0 = no command.
        lt = self._trigger_to_unit(self._safe_axis(axes, self.axes_idx['lt'], 1.0))
        rt = self._trigger_to_unit(self._safe_axis(axes, self.axes_idx['rt'], 1.0))

        arm_spd  = rt - lt              # +ve = OPEN, -ve = CLOSE
        lift_spd = ry                   # +ve = UP

        arm_msg = Twist()
        arm_msg.linear.x = float(max(-1.0, min(1.0, arm_spd)))
        arm_msg.linear.z = float(max(-1.0, min(1.0, lift_spd)))
        self.pub_arm_vel.publish(arm_msg)

        # ── ARM DISCRETE EVENTS (edge-triggered) ───────────────
        if self._just_pressed(buttons, self.btn_estop):
            self.pub_arm_cmd.publish(String(data='ESTOP'))
            self.get_logger().warning('ESTOP from gamepad')
        if self._just_pressed(buttons, self.btn_clear):
            self.pub_arm_cmd.publish(String(data='CLEAR'))
            self.get_logger().info('Clear ESTOP')
        if self._just_pressed(buttons, self.btn_home):
            self.pub_arm_cmd.publish(String(data='HOME'))
            self.get_logger().info('HOME requested')

        self.prev_buttons = list(buttons)


def main(args=None):
    rclpy.init(args=args)
    node = JoyToAislebot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
