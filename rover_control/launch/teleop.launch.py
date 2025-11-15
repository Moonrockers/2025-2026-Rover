"""
Launch file for keyboard teleoperation
Start this separately to control the rover manually
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_control',
            executable='teleop_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',  # Opens in separate terminal
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
