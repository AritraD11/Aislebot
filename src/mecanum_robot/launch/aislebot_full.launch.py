#!/usr/bin/env python3
"""
aislebot_full.launch.py — bring up the entire AisleBot stack.

Nodes started:
    joy_node               (USB gamepad → /joy)
    joy_to_aislebot        (/joy → /cmd_vel, /arm/cmd_vel, /arm/command)
    teleop_asym            (/cmd_vel → /wheel_speeds)
    esp32_bridge           (/wheel_speeds → ESP32 serial)   /dev/esp32 CP2102
    odom_pub               (encoder feedback → /odom)
    arm_bridge             (arm topics → Arduino Mega)      /dev/mega  CH340
    phone_dashboard        (HTTP + WebSocket → ROS2 publishers)
    lcd_display            (16x2 I2C LCD status at 0x27)

Usage:
    ros2 launch mecanum_robot aislebot_full.launch.py
    ros2 launch mecanum_robot aislebot_full.launch.py use_phone:=false
    ros2 launch mecanum_robot aislebot_full.launch.py use_joystick:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument('esp32_port',   default_value='/dev/esp32',
                              description='ESP32 drive controller (CP2102)'),
        DeclareLaunchArgument('esp32_baud',   default_value='921600'),
        DeclareLaunchArgument('arm_port',     default_value='/dev/mega',
                              description='Arduino Mega arm controller (CH340)'),
        DeclareLaunchArgument('arm_baud',     default_value='115200'),
        DeclareLaunchArgument('use_joystick', default_value='true',
                              description='Start joy_node + joy_to_aislebot'),
        DeclareLaunchArgument('use_phone',    default_value='true',
                              description='Start phone_dashboard HTTP/WS server'),
        DeclareLaunchArgument('http_port',    default_value='8080'),
        DeclareLaunchArgument('max_linear',   default_value='0.15'),
        DeclareLaunchArgument('max_angular',  default_value='0.30'),
    ]

    use_joy   = LaunchConfiguration('use_joystick')
    use_phone = LaunchConfiguration('use_phone')

    banner = LogInfo(msg=[
        '\n══════════════════════════════════════════════════════════\n',
        '  AisleBot Full Stack\n',
        '  ESP32  (drive): ', LaunchConfiguration('esp32_port'),
        ' @ ', LaunchConfiguration('esp32_baud'), ' baud\n',
        '  Mega   (arm)  : ', LaunchConfiguration('arm_port'),
        ' @ ', LaunchConfiguration('arm_baud'), ' baud\n',
        '  Joystick : ', use_joy, '\n',
        '  Phone    : ', use_phone,
        ' (port ', LaunchConfiguration('http_port'), ')\n',
        '══════════════════════════════════════════════════════════\n',
    ])

    # ── INPUT ────────────────────────────────────────────────────
    joy_node = Node(
        package='joy', executable='joy_node', name='joy_node',
        parameters=[{'device_id': 0, 'deadzone': 0.05, 'autorepeat_rate': 25.0}],
        condition=IfCondition(use_joy),
        output='screen',
    )

    joy_translator = Node(
        package='mecanum_robot', executable='joy_to_aislebot',
        name='joy_to_aislebot',
        parameters=[{
            'max_linear':  LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
        }],
        condition=IfCondition(use_joy),
        output='screen',
    )

    phone = Node(
        package='mecanum_robot', executable='phone_dashboard',
        name='phone_dashboard',
        parameters=[{
            'http_port':   LaunchConfiguration('http_port'),
            'max_linear':  LaunchConfiguration('max_linear'),
            'max_angular': LaunchConfiguration('max_angular'),
        }],
        condition=IfCondition(use_phone),
        output='screen',
    )

    # ── DRIVE PIPELINE ───────────────────────────────────────────
    teleop = Node(
        package='mecanum_robot', executable='teleop_asym',
        name='teleop_asym',
        parameters=[{
            'wheel_radius': 0.0762,
            'l1':           0.403,
            'l2':           0.333,
            'd':            0.15769,
            'max_linear':   LaunchConfiguration('max_linear'),
            'max_angular':  LaunchConfiguration('max_angular'),
        }],
        output='screen',
    )

    esp32_bridge = Node(
        package='mecanum_robot', executable='esp32_bridge',
        name='esp32_bridge',
        parameters=[{
            'serial_port':     LaunchConfiguration('esp32_port'),
            'baud_rate':       921600,
            'max_wheel_speed': 6.28,
        }],
        output='screen',
    )

    odom_pub = Node(
        package='mecanum_robot', executable='odom_pub',
        name='odometry_publisher',
        parameters=[{
            'wheel_radius': 0.0762,
            'l1':           0.403,
            'l2':           0.333,
            'd':            0.15769,
            'publish_tf':   True,
        }],
        output='screen',
    )

    # ── ARM PIPELINE ─────────────────────────────────────────────
    arm_bridge = Node(
        package='mecanum_robot', executable='arm_bridge',
        name='arm_bridge',
        parameters=[{
            'serial_port':                 LaunchConfiguration('arm_port'),
            'baud_rate':                   115200,
            'disable_joystick_on_connect': True,
            'auto_enable_on_connect':      True,
        }],
        output='screen',
    )

    # ── STATUS ───────────────────────────────────────────────────
    lcd = Node(
        package='mecanum_robot', executable='lcd_display',
        name='lcd_display',
        output='screen',
    )

    return LaunchDescription([
        *args,
        banner,
        joy_node, joy_translator, phone,
        teleop, esp32_bridge, odom_pub,
        arm_bridge,
        lcd,
    ])
