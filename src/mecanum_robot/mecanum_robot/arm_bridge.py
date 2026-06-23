#!/usr/bin/env python3
"""
arm_bridge.py — ROS2 ↔ Arduino Mega serial bridge for the AisleBot UV arm.

Subscribes
    /arm/cmd_vel   geometry_msgs/Twist  — continuous velocity
                       linear.x  → arm  -1..+1   (+ve = OPEN both arms)
                       linear.z  → lift -1..+1   (+ve = UP)
    /arm/command   std_msgs/String      — discrete events
                       'ENABLE' | 'DISABLE' | 'HOME' | 'ESTOP' | 'CLEAR'
                       'PING'   | 'INFO'    | 'STATUS'
                       'OPEN'   | 'CLOSE'   | 'LIFT'  | 'LOWER' | 'STOP'
                       'UV_ON'  | 'UV_OFF'                       (UV tubes)

Publishes
    /arm/status    std_msgs/String      — last [STATUS,...] line from Mega

Serial protocol mirrors aislebot_arm_v7.ino exactly.
Watchdog: bridge sends <A,0,0> if no Twist received for 300 ms while enabled.

IIT Bombay | Aritra Das (25D0074) | May 2026
"""

import threading
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

import serial


class ArmBridge(Node):
    def __init__(self):
        super().__init__('arm_bridge')

        # ── Parameters ────────────────────────────────────────────
        self.declare_parameter('serial_port', '/dev/mega')          # FIX: was /dev/ttyACM0
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('reconnect_interval', 3.0)
        self.declare_parameter('command_timeout_ms', 300)
        self.declare_parameter('disable_joystick_on_connect', True)
        self.declare_parameter('auto_enable_on_connect', True)      # FIX: was False

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.cmd_timeout = self.get_parameter('command_timeout_ms').value / 1000.0
        self.reconnect_interval = self.get_parameter('reconnect_interval').value
        self.disable_joystick = self.get_parameter('disable_joystick_on_connect').value
        self.auto_enable = self.get_parameter('auto_enable_on_connect').value

        # ── Serial state ──────────────────────────────────────────
        self.ser = None
        self.serial_lock = threading.Lock()
        self.connected = False

        # ── Last command tracking (for watchdog) ─────────────────
        self.last_arm = 0.0
        self.last_lift = 0.0
        self.last_cmd_ts = 0.0

        # ── ROS2 wiring ───────────────────────────────────────────
        self.create_subscription(Twist,  '/arm/cmd_vel', self.cb_twist,   10)
        self.create_subscription(String, '/arm/command', self.cb_command, 10)
        self.status_pub = self.create_publisher(String, '/arm/status', 10)

        # 50 Hz command tx + watchdog
        self.create_timer(0.02, self.tx_loop)
        # Connection monitor
        self.create_timer(self.reconnect_interval, self.monitor_connection)

        self.get_logger().info(
            f'arm_bridge starting on {self.port} @ {self.baud} baud')

        # Reader thread — non-blocking serial RX
        self._stop = threading.Event()
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.try_connect()

    # ───────────────────── Connection ─────────────────────────────
    def try_connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(2.0)               # Mega autoreset settle
            self.ser.reset_input_buffer()
            self.connected = True
            self.get_logger().info(f'Connected to Arduino Mega on {self.port}')

            # Stop fighting with the bench joystick
            if self.disable_joystick:
                self.send('<J0>')
                self.get_logger().info('Physical joystick disabled (<J0>)')

            self.send('<P>')              # ping
            if self.auto_enable:
                self.send('<E1>')         # FIX: now actually sends enable
                self.get_logger().info('Arm enabled on connect (<E1>)')

        except serial.SerialException as e:
            self.connected = False
            self.get_logger().warning(f'Serial open failed: {e}')

    def monitor_connection(self):
        if not self.connected:
            self.try_connect()

    # ───────────────────── Subscribers ────────────────────────────
    def cb_twist(self, msg: Twist):
        """Handle continuous velocity from /arm/cmd_vel (joystick path)."""
        a = max(-1.0, min(1.0, float(msg.linear.x)))
        l = max(-1.0, min(1.0, float(msg.linear.z)))
        self.last_arm = a
        self.last_lift = l
        self.last_cmd_ts = time.monotonic()

    def cb_command(self, msg: String):
        """
        Handle discrete commands from /arm/command.

        Discrete (sent to Mega directly):
            ENABLE DISABLE HOME ESTOP CLEAR PING INFO STATUS

        Motion shortcuts (from phone dashboard hold-buttons):   ← FIX: new
            OPEN   → arm  = +1.0
            CLOSE  → arm  = -1.0
            LIFT   → lift = +1.0
            LOWER  → lift = -1.0
            STOP   → arm  =  0.0, lift = 0.0
        """
        cmd = msg.data.strip().upper()

        # ── Discrete serial commands ───────────────────────────────
        discrete = {
            'ENABLE':  '<E1>',
            'DISABLE': '<E0>',
            'HOME':    '<H>',
            'ESTOP':   '<S>',
            'CLEAR':   '<C>',
            'PING':    '<P>',
            'INFO':    '<I>',
            'STATUS':  '<?>',
            'UV_ON':   '<U1>',   # staged tube sequence on the Mega: t1, +5s t2, +10s t3
            'UV_OFF':  '<U0>',   # all tubes off immediately
        }
        if cmd in discrete:
            self.send(discrete[cmd])
            self.get_logger().info(f'arm cmd → {cmd}')
            return

        # ── Motion shortcuts (set velocity, let tx_loop send <A,...>) ──
        now = time.monotonic()
        if cmd == 'OPEN':                           # FIX: new
            self.last_arm  =  1.0
            self.last_lift =  0.0
            self.last_cmd_ts = now
            self.get_logger().info('arm cmd → OPEN  (arm=+1.0)')
        elif cmd == 'CLOSE':                        # FIX: new
            self.last_arm  = -1.0
            self.last_lift =  0.0
            self.last_cmd_ts = now
            self.get_logger().info('arm cmd → CLOSE (arm=-1.0)')
        elif cmd == 'LIFT':                         # FIX: new
            self.last_arm  =  0.0
            self.last_lift =  1.0
            self.last_cmd_ts = now
            self.get_logger().info('arm cmd → LIFT  (lift=+1.0)')
        elif cmd == 'LOWER':                        # FIX: new
            self.last_arm  =  0.0
            self.last_lift = -1.0
            self.last_cmd_ts = now
            self.get_logger().info('arm cmd → LOWER (lift=-1.0)')
        elif cmd == 'STOP':                         # FIX: new
            self.last_arm  = 0.0
            self.last_lift = 0.0
            # Don't update last_cmd_ts — let watchdog see this as silence
            # so it keeps sending <A,0,0> via the watchdog path naturally.
            self.get_logger().info('arm cmd → STOP')
        else:
            self.get_logger().warning(f'Unknown /arm/command: "{cmd}"')

    # ───────────────────── TX loop ────────────────────────────────
    def tx_loop(self):
        if not self.connected:
            return

        # Watchdog: if no Twist/command in cmd_timeout window, zero out
        if time.monotonic() - self.last_cmd_ts > self.cmd_timeout:
            self.last_arm  = 0.0
            self.last_lift = 0.0

        # Send velocity command at 50 Hz
        self.send(f'<A,{self.last_arm:.3f},{self.last_lift:.3f}>')

    def send(self, text: str):
        if not self.connected or self.ser is None:
            return
        with self.serial_lock:
            try:
                self.ser.write(text.encode('ascii'))
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial write failed: {e}')
                self.connected = False
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

    # ───────────────────── RX loop ────────────────────────────────
    def rx_loop(self):
        buf = b''
        while not self._stop.is_set():
            if not self.connected or self.ser is None:
                time.sleep(0.1)
                continue
            try:
                chunk = self.ser.read(64)
                if not chunk:
                    continue
                buf += chunk
                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    text = line.decode('ascii', errors='replace').strip()
                    if not text:
                        continue
                    if text.startswith('[STATUS'):
                        msg = String(); msg.data = text
                        self.status_pub.publish(msg)
                    else:
                        self.get_logger().info(f'mega: {text}')
            except (serial.SerialException, OSError) as e:
                self.get_logger().error(f'Serial read failed: {e}')
                self.connected = False
                time.sleep(0.5)

    # ───────────────────── Shutdown ───────────────────────────────
    def shutdown(self):
        self._stop.set()
        try:
            self.send('<U0>')      # tubes off
            self.send('<E0>')      # arm disabled
            time.sleep(0.05)
        except Exception:
            pass
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass


def main(args=None):
    rclpy.init(args=args)
    node = ArmBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
