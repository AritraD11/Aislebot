"""
AisleBot Gazebo Simulation Launch
==================================
Starts Gazebo with AisleBot model, robot_state_publisher, and bridge nodes.
Usage:
  ros2 launch mecanum_robot simulation.launch.py
  ros2 launch mecanum_robot simulation.launch.py world:=warehouse.world
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('mecanum_robot')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'aislebot.urdf')
    world_file = os.path.join(pkg_dir, 'worlds', 'warehouse.world')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    gazebo_ros_dir = get_package_share_directory('gazebo_ros')

    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=world_file,
                              description='Gazebo world file'),
        DeclareLaunchArgument('x_pos', default_value='0.0'),
        DeclareLaunchArgument('y_pos', default_value='0.0'),
        DeclareLaunchArgument('z_pos', default_value='0.15'),

        # === Gazebo Server + Client ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_ros_dir, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'verbose': 'false',
            }.items()
        ),

        # === Robot State Publisher ===
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc,
                         'use_sim_time': True}],
            output='screen'
        ),

        # === Spawn Robot in Gazebo ===
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'aislebot',
                '-x', LaunchConfiguration('x_pos'),
                '-y', LaunchConfiguration('y_pos'),
                '-z', LaunchConfiguration('z_pos'),
            ],
            output='screen'
        ),

        # === Teleop (kinematics) ===
        Node(
            package='mecanum_robot',
            executable='teleop_asym',
            name='mecanum_teleop_asymmetric',
            parameters=[{
                'use_sim_time': True,
                'wheel_radius': 0.0762,
                'l1': 0.403, 'l2': 0.333, 'd': 0.15769,
                'max_linear': 0.30, 'max_angular': 0.60,
            }],
            output='screen'
        ),

        # === Gazebo Bridge (wheel_speeds → cmd_vel for planar_move) ===
        Node(
            package='mecanum_robot',
            executable='gazebo_bridge',
            name='gazebo_bridge',
            parameters=[{
                'use_sim_time': True,
                'wheel_radius': 0.0762,
                'l1': 0.403, 'l2': 0.333, 'd': 0.15769,
            }],
            output='screen'
        ),

        # === Keyboard Teleop ===
        Node(
            package='mecanum_robot',
            executable='keyboard_teleop',
            name='keyboard_teleop',
            parameters=[{
                'use_sim_time': True,
                'max_linear': 0.30,
                'max_angular': 0.60,
            }],
            output='screen',
            prefix='xterm -e',
        ),
    ])
