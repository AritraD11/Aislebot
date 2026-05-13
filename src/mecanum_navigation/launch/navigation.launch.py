"""
AisleBot Navigation Launch
============================
Starts Nav2 stack for autonomous navigation on a pre-built map.
Usage:
  ros2 launch mecanum_navigation navigation.launch.py map:=/path/to/map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory('mecanum_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(nav_dir, 'config', 'nav2_params.yaml')
    ekf_config = os.path.join(nav_dir, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value='',
                              description='Path to map YAML file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # === EKF ===
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # === Nav2 Bringup ===
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'params_file': nav2_params,
                'autostart': 'true',
            }.items()
        ),
    ])
