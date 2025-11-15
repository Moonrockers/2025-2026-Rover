"""
Launch file for the rover control system
Starts all core nodes for robot operation
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_control',
            executable='motor_controller',
            name='motor_controller',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        Node(
            package='rover_control',
            executable='sensor_manager',
            name='sensor_manager',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        Node(
            package='rover_control',
            executable='telemetry',
            name='telemetry',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
        Node(
            package='rover_control',
            executable='safety_monitor',
            name='safety_monitor',
            output='screen',
            parameters=[{
                'use_sim_time': False
            }]
        ),
    ])
