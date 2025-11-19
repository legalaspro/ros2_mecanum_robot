"""
Kinematic Model starts 2 nodes
- Wheel Velocities Publisher
- Kinematic Model (Forward Kinematics for ROSBot XL)

Usage:
  ros2 launch kinematic_model kinematic_model.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([    
        Node(
            package='kinematic_model',
            executable='kinematic_model',
            name='kinematic_model',
            output='screen',
            emulate_tty=True,
            parameters=[
                { "wheel_radius": 0.05 },
                { "wheel_base": 0.17 },
                { "track_width": 0.26969 }
            ]),
    
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='wheel_velocities_publisher',
                    executable='wheel_velocities_publisher',
                    name='wheel_velocities_publisher',
                    output='screen',
                    emulate_tty=True),
            ]
        )
    ])