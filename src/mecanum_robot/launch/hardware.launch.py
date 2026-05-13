"""
AisleBot Hardware Launch
========================
Starts all nodes needed for physical robot operation.
Usage:
  ros2 launch mecanum_robot hardware.launch.py
  ros2 launch mecanum_robot hardware.launch.py serial_port:=/dev/ttyUSB0 use_keyboard:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('mecanum_robot')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'aislebot.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # === Launch Arguments ===
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
                              description='ESP32 serial port'),
        DeclareLaunchArgument('baud_rate', default_value='921600',
                              description='Serial baud rate'),
        DeclareLaunchArgument('use_keyboard', default_value='false',
                              description='Use keyboard teleop instead of joystick'),
        DeclareLaunchArgument('enable_imu', default_value='false',
                              description='Enable IMU data from ESP32'),

        # === Robot State Publisher (TF from URDF) ===
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': False}],
            output='screen'
        ),

        # === ESP32 Bridge ===
        Node(
            package='mecanum_robot',
            executable='esp32_bridge',
            name='esp32_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': int(LaunchConfiguration('baud_rate').perform(None)) if False else 921600,
                'max_wheel_speed': 6.28,
                'enable_imu': LaunchConfiguration('enable_imu'),
            }],
            output='screen'
        ),

        # === Teleop (kinematics: cmd_vel → wheel_speeds) ===
        Node(
            package='mecanum_robot',
            executable='teleop_asym',
            name='mecanum_teleop_asymmetric',
            parameters=[{
                'wheel_radius': 0.0762,
                'l1': 0.403,
                'l2': 0.333,
                'd': 0.15769,
                'max_linear': 0.15,
                'max_angular': 0.30,
            }],
            output='screen'
        ),

        # === Odometry Publisher ===
        Node(
            package='mecanum_robot',
            executable='odom_pub',
            name='odometry_publisher',
            parameters=[{
                'wheel_radius': 0.0762,
                'l1': 0.403,
                'l2': 0.333,
                'd': 0.15769,
                'publish_tf': True,
            }],
            output='screen'
        ),

        # === Keyboard Teleop (if enabled) ===
        Node(
            package='mecanum_robot',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            condition=IfCondition(LaunchConfiguration('use_keyboard')),
            parameters=[{
                'max_linear': 0.15,
                'max_angular': 0.30,
            }],
            output='screen',
            prefix='xterm -e',
        ),

        # === Joystick Driver (if not using keyboard) ===
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            condition=UnlessCondition(LaunchConfiguration('use_keyboard')),
            output='screen'
        ),
    ])
