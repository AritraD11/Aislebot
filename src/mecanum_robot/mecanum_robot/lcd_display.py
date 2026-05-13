#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from RPLCD.i2c import CharLCD
import socket
import threading
import time


class LCDDisplay(Node):

    def __init__(self):
        super().__init__('lcd_display')

        try:
            self.lcd = CharLCD(
                i2c_expander='PCF8574',
                address=0x27,
                port=1,
                cols=16,
                rows=2,
                dotsize=8
            )
            self.lcd.clear()
            self.lcd_ok = True
            self.get_logger().info('LCD initialized at 0x27')
        except Exception as e:
            self.lcd_ok = False
            self.get_logger().error(f'LCD init failed: {e}')

        self.drive_state = 'READY'
        self.arm_state = '----'
        self.lift_state = '----'
        self.last_wheel_time = 0.0
        self.lock = threading.Lock()

        self.create_subscription(
            Float64MultiArray, '/wheel_speeds', self.wheel_cb, 10)
        self.create_subscription(
            String, '/arm/command', self.arm_cb, 10)

        self.create_timer(0.25, self.update_display)

        self.boot_done = False
        self.boot_time = time.time()
        self._show_boot()

    def _get_ip(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(('8.8.8.8', 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except Exception:
            return 'NO NETWORK'

    def _show_boot(self):
        if not self.lcd_ok:
            return
        ip = self._get_ip()
        self.lcd.clear()
        self.lcd.write_string('AISLEBOT  READY')
        self.lcd.cursor_pos = (1, 0)
        self.lcd.write_string(ip[:16])

    def wheel_cb(self, msg):
        if len(msg.data) != 4:
            return
        fr, fl, rr, rl = msg.data
        self.last_wheel_time = time.time()
        t = 0.15

        fwd    = (fr + fl + rr + rl) / 4.0
        strafe = (-fr + fl + rr - rl) / 4.0
        rot    = (-fr + fl - rr + rl) / 4.0

        with self.lock:
            if abs(fwd) < t and abs(strafe) < t and abs(rot) < t:
                self.drive_state = 'STOPPED'
            elif abs(fwd) >= abs(strafe) and abs(fwd) >= abs(rot):
                self.drive_state = 'FORWARD' if fwd > 0 else 'BACK'
            elif abs(strafe) >= abs(fwd) and abs(strafe) >= abs(rot):
                self.drive_state = 'RIGHT' if strafe > 0 else 'LEFT'
            else:
                self.drive_state = 'ROT R' if rot > 0 else 'ROT L'

    def arm_cb(self, msg):
        cmd = msg.data.strip().upper()
        with self.lock:
            if 'OPEN' in cmd:
                self.arm_state = 'OPEN'
            elif 'CLOSE' in cmd or 'HOME' in cmd:
                self.arm_state = 'CLSD'
            if 'LIFT' in cmd:
                self.lift_state = 'UP  '
            elif 'LOWER' in cmd or 'HOME' in cmd:
                self.lift_state = 'DOWN'
            if 'ESTOP' in cmd:
                self.arm_state = 'STOP'
                self.lift_state = 'STOP'

    def update_display(self):
        if not self.lcd_ok:
            return

        if not self.boot_done:
            if time.time() - self.boot_time < 4.0:
                return
            self.boot_done = True
            self.lcd.clear()

        if time.time() - self.last_wheel_time > 1.0:
            with self.lock:
                self.drive_state = 'READY'

        with self.lock:
            drive = self.drive_state
            arm   = self.arm_state
            lift  = self.lift_state

        line1 = f'DRIVE:{drive:<10}'[:16]
        line2 = f'ARM:{arm} LFT:{lift}'[:16]

        try:
            self.lcd.cursor_pos = (0, 0)
            self.lcd.write_string(line1)
            self.lcd.cursor_pos = (1, 0)
            self.lcd.write_string(line2)
        except Exception as e:
            self.get_logger().warn(f'LCD write error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LCDDisplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.lcd_ok:
                node.lcd.clear()
                node.lcd.write_string('AISLEBOT OFF')
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()