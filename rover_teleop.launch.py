"""
rover_teleop.launch.py

Launches the full teleoperation stack:
    - joy_node              (reads PS4 controller via USB/Bluetooth)
    - playstation_controller (maps PS4 inputs to ROS2 topics)
    - motor_controller       (drives SPARK MAX motors via SocketCAN)

Shared parameters (wheel_separation, max speeds) are defined ONCE here
and passed to both nodes so they can never get out of sync.

Usage:
    ros2 launch moonrockers rover_teleop.launch.py

Optional overrides:
    ros2 launch moonrockers rover_teleop.launch.py can_interface:=can1
    ros2 launch moonrockers rover_teleop.launch.py invert_y_axis:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ------------------------------------------------------------------ #
    # Launch arguments — override these on the command line if needed
    # ------------------------------------------------------------------ #
    args = [
        DeclareLaunchArgument('can_interface',    default_value='can0'),
        DeclareLaunchArgument('invert_y_axis',    default_value='false'),
        DeclareLaunchArgument('joy_dev',          default_value='/dev/input/js0'),

        # These are declared as launch args so you can tune them at
        # launch time without editing source files.
        DeclareLaunchArgument('wheel_separation',  default_value='0.5'),   # meters
        DeclareLaunchArgument('max_linear_speed',  default_value='1.0'),   # m/s
        DeclareLaunchArgument('max_angular_speed', default_value='2.0'),   # rad/s
        DeclareLaunchArgument('ramp_rate',         default_value='0.05'),  # m/s per tick

        # SPARK MAX CAN device IDs (set these to match REV Hardware Client)
        DeclareLaunchArgument('spark_left_front_id',  default_value='1'),
        DeclareLaunchArgument('spark_right_front_id', default_value='2'),
        DeclareLaunchArgument('spark_left_rear_id',   default_value='3'),
        DeclareLaunchArgument('spark_right_rear_id',  default_value='4'),

        DeclareLaunchArgument('spark_left_inverted',       default_value='false'),
        DeclareLaunchArgument('spark_right_inverted',      default_value='true'),
        DeclareLaunchArgument('spark_rear_invert_follower', default_value='false'),
    ]

    # ------------------------------------------------------------------ #
    # joy_node — reads raw controller hardware
    # ------------------------------------------------------------------ #
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'dev':          LaunchConfiguration('joy_dev'),
            'autorepeat_rate': 20.0,   # Hz — keeps /joy publishing even when idle
            'deadzone':     0.05,      # hardware deadzone (software deadzone in controller node)
        }]
    )

    # ------------------------------------------------------------------ #
    # PlayStation controller node
    # Shared params (wheel_separation, max speeds) come from launch args.
    # ------------------------------------------------------------------ #
    controller_node = Node(
        package='moonrockers',
        executable='playstation_controller',
        name='playstation_controller',
        parameters=[{
            'controller_type':   'ps4',
            'deadzone':          0.1,
            'invert_y_axis':     LaunchConfiguration('invert_y_axis'),
            'turbo_multiplier':  2.0,

            # Shared — sourced from launch args
            'wheel_separation':  LaunchConfiguration('wheel_separation'),
            'max_linear_speed':  LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),
            'ramp_rate':         LaunchConfiguration('ramp_rate'),
        }],
        output='screen',
    )

    # ------------------------------------------------------------------ #
    # Motor controller node
    # Same shared params ensure the Twist ↔ wheel speed roundtrip is exact.
    # ------------------------------------------------------------------ #
    motor_node = Node(
        package='moonrockers',
        executable='motor_controller',
        name='motor_controller',
        parameters=[{
            'motor_driver':  'sparkmax',
            'control_mode':  'differential',
            'enable_safety': True,
            'timeout':       1.0,

            # Shared — sourced from launch args
            'wheel_separation':  LaunchConfiguration('wheel_separation'),
            'max_speed':         LaunchConfiguration('max_linear_speed'),
            'max_angular_speed': LaunchConfiguration('max_angular_speed'),

            # CAN interface
            'can_interface': LaunchConfiguration('can_interface'),

            # SPARK MAX IDs
            'spark_left_front_id':        LaunchConfiguration('spark_left_front_id'),
            'spark_right_front_id':       LaunchConfiguration('spark_right_front_id'),
            'spark_left_rear_id':         LaunchConfiguration('spark_left_rear_id'),
            'spark_right_rear_id':        LaunchConfiguration('spark_right_rear_id'),
            'spark_left_inverted':        LaunchConfiguration('spark_left_inverted'),
            'spark_right_inverted':       LaunchConfiguration('spark_right_inverted'),
            'spark_rear_invert_follower': LaunchConfiguration('spark_rear_invert_follower'),
        }],
        output='screen',
    )

    return LaunchDescription(args + [joy_node, controller_node, motor_node])