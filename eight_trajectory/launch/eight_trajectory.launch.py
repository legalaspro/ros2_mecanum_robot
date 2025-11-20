"""
Eight Trajectory starts 2 nodes
- Eight Trajectory (Inverse Kinematics 8 Figure for ROSBot XL)
- Kinematic Model (Forward Kinematics for ROSBot XL)

Usage:
  ros2 launch eight_trajectory eight_trajectory.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description(): 

    config = os.path.join(
        get_package_share_directory('eight_trajectory'),
        'config',
        'robot_params.yaml'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Set node log level (debug/info/warn/error/fatal)'
    )

    log_level_config = LogInfo(msg=["Log Level: ", LaunchConfiguration('log_level')])

    return LaunchDescription([
        log_level_arg,
        log_level_config,
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[config]),
        Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            name='eight_trajectory',
            output='screen',
            emulate_tty=True,
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            parameters=[config]),  
    ])