"""
AisleBot SLAM Launch
=====================
Starts slam_toolbox + EKF for mapping. Run alongside hardware.launch.py.
Usage:
  ros2 launch mecanum_navigation slam.launch.py
  ros2 launch mecanum_navigation slam.launch.py use_sim_time:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory('mecanum_navigation')
    slam_config = os.path.join(nav_dir, 'config', 'slam_params.yaml')
    ekf_config = os.path.join(nav_dir, 'config', 'ekf_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        # === EKF (sensor fusion) ===
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # === SLAM Toolbox ===
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),

        # === RViz2 for visualization ===
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(nav_dir, 'config', 'slam.rviz')],
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
            output='screen'
        ),
    ])
