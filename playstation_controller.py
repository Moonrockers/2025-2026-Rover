#!/usr/bin/env python3
"""
PlayStation Controller Interface for Rover
Supports PS4 and PS5 DualShock/DualSense controllers
Provides manual control and mission management
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool, Float32
import numpy as np
from enum import Enum


# FIX: Renamed from ControlMode → RoverMode to avoid collision with the
# ControlMode enum in spark_max_can_driver.py if both modules are ever
# imported in the same process.
class RoverMode(Enum):
    """Rover operating modes"""
    MANUAL_DRIVE   = 0
    MANUAL_DIG     = 1
    MANUAL_DEPOSIT = 2
    AUTONOMOUS     = 3


class PS4Button:
    """PS4 Controller button mapping"""
    CROSS     = 0
    CIRCLE    = 1
    SQUARE    = 2
    TRIANGLE  = 3
    L1        = 4
    R1        = 5
    # L2/R2 are AXES on PS4, not buttons — see PS4Axis below.
    L2        = 6   # kept for edge-case drivers only
    R2        = 7
    SHARE     = 8
    OPTIONS   = 9
    L3        = 10
    R3        = 11
    PS        = 12
    TOUCHPAD  = 13


class PS4Axis:
    """PS4 Controller axis mapping"""
    LEFT_STICK_X  = 0
    LEFT_STICK_Y  = 1
    RIGHT_STICK_X = 2
    RIGHT_STICK_Y = 3
    L2_ANALOG     = 4   # -1.0 released → +1.0 fully pressed
    R2_ANALOG     = 5
    DPAD_X        = 6
    DPAD_Y        = 7


# Threshold for treating an analog trigger as "pressed"
TRIGGER_THRESHOLD = 0.5


class PlayStationController(Node):
    """
    PlayStation controller interface for rover control.
    Maps PS4/PS5 controller inputs to rover commands.
    """

    def __init__(self):
        super().__init__('playstation_controller')

        # ------------------------------------------------------------------ #
        # Parameters — shared values (wheel_separation, max speeds) must match
        # motor_controller.py exactly. Set them once in the launch file and
        # both nodes read from there.
        # ------------------------------------------------------------------ #
        self.declare_parameter('controller_type',   'ps4')
        self.declare_parameter('deadzone',          0.1)
        self.declare_parameter('max_linear_speed',  1.0)   # m/s  — must match motor_controller max_speed
        self.declare_parameter('max_angular_speed', 2.0)   # rad/s — must match motor_controller max_angular_speed
        self.declare_parameter('turbo_multiplier',  2.0)
        self.declare_parameter('enable_rumble',     True)
        self.declare_parameter('invert_y_axis',     False)
        self.declare_parameter('wheel_separation',  0.5)   # m — must match motor_controller wheel_separation

        # FIX: ramp rate — maximum change in speed per control loop tick (50ms).
        # Prevents instant full-speed commands that stress the drivetrain.
        # Default 0.05 m/s per tick = 1.0 m/s per second ramp rate.
        # Set to 1.0 to disable ramping.
        self.declare_parameter('ramp_rate', 0.05)

        self.controller_type   = self.get_parameter('controller_type').value
        self.deadzone          = self.get_parameter('deadzone').value
        self.max_linear_speed  = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.turbo_multiplier  = self.get_parameter('turbo_multiplier').value
        self.enable_rumble     = self.get_parameter('enable_rumble').value
        self.invert_y          = self.get_parameter('invert_y_axis').value
        self.wheel_separation  = self.get_parameter('wheel_separation').value
        self.ramp_rate         = self.get_parameter('ramp_rate').value

        # ------------------------------------------------------------------ #
        # State
        # ------------------------------------------------------------------ #
        self.rover_mode     = RoverMode.MANUAL_DRIVE
        self.emergency_stop = False
        self.last_joy_msg   = None

        self.prev_buttons = []
        self.prev_axes    = []

        # FIX: ramp state — track the last commanded speeds so we can
        # increment toward the target rather than jumping instantly.
        self.ramped_left_speed  = 0.0
        self.ramped_right_speed = 0.0

        # Digging state
        self.digging_active      = False
        self.current_dig_depth   = 0.0
        self.dig_depth_increment = 0.05   # 5 cm per press

        # Deposition state
        self.depositing_active = False
        self.dump_angle        = 0.0
        self.angle_increment   = 5.0      # 5 degrees per press

        # ------------------------------------------------------------------ #
        # Subscribers & publishers
        # ------------------------------------------------------------------ #
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.cmd_vel_pub     = self.create_publisher(Twist,   '/cmd_vel',              10)
        self.dig_cmd_pub     = self.create_publisher(String,  '/digging/command',       10)
        self.dig_depth_pub   = self.create_publisher(Float32, '/digging/target_depth',  10)
        self.deposit_cmd_pub = self.create_publisher(String,  '/deposition/command',    10)
        self.mission_cmd_pub = self.create_publisher(String,  '/mission/command',       10)
        self.estop_pub       = self.create_publisher(Bool,    '/emergency_stop',        10)
        self.status_pub      = self.create_publisher(String,  '/controller/status',     10)

        # Control loop at 20 Hz
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info('=' * 60)
        self.get_logger().info(f'PlayStation {self.controller_type.upper()} Controller Ready!')
        self.get_logger().info(f'  wheel_separation : {self.wheel_separation} m')
        self.get_logger().info(f'  max_linear_speed : {self.max_linear_speed} m/s')
        self.get_logger().info(f'  max_angular_speed: {self.max_angular_speed} rad/s')
        self.get_logger().info(f'  ramp_rate        : {self.ramp_rate} m/s per tick')
        self.get_logger().info('=' * 60)
        self.print_controls()

    # ====================================================================== #
    # Startup info
    # ====================================================================== #

    def print_controls(self):
        self.get_logger().info('')
        self.get_logger().info('CONTROL SCHEME — SKID-STEER (TANK) MODE:')
        self.get_logger().info('-' * 60)
        self.get_logger().info('DRIVING (default mode):')
        self.get_logger().info('  Left Stick Y   → LEFT  track speed')
        self.get_logger().info('  Right Stick Y  → RIGHT track speed')
        self.get_logger().info('  R1 (hold)      → Turbo  (2x speed)')
        self.get_logger().info('  L1 (hold)      → Precision (0.5x speed)')
        self.get_logger().info('')
        self.get_logger().info('MODE SWITCHING:')
        self.get_logger().info('  Triangle → DIGGING mode')
        self.get_logger().info('  Circle   → DEPOSITION mode')
        self.get_logger().info('  Square   → AUTONOMOUS mode')
        self.get_logger().info('  Cross    → MANUAL DRIVE mode')
        self.get_logger().info('')
        self.get_logger().info('DIGGING MODE:')
        self.get_logger().info('  D-Pad Up/Down → Adjust dig depth')
        self.get_logger().info('  R2 (analog)   → Start digging')
        self.get_logger().info('  L2 (analog)   → Stop digging')
        self.get_logger().info('  R1            → Raise excavator')
        self.get_logger().info('  L1            → Lower excavator')
        self.get_logger().info('')
        self.get_logger().info('DEPOSITION MODE:')
        self.get_logger().info('  D-Pad Up/Down → Adjust dump angle')
        self.get_logger().info('  R2 (analog)   → Dump')
        self.get_logger().info('  L2 (analog)   → Return to neutral')
        self.get_logger().info('')
        self.get_logger().info('AUTONOMOUS MODE:')
        self.get_logger().info('  R2 → Start   L2 → Stop')
        self.get_logger().info('  R1 → Pause   L1 → Resume')
        self.get_logger().info('')
        self.get_logger().info('EMERGENCY:')
        self.get_logger().info('  OPTIONS  → Toggle emergency stop')
        self.get_logger().info('  SHARE    → Clear emergency stop')
        self.get_logger().info('  PS       → Reset all systems')
        self.get_logger().info('-' * 60)

    # ====================================================================== #
    # ROS callbacks
    # ====================================================================== #

    def joy_callback(self, msg: Joy):
        self.last_joy_msg = msg
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
        if not self.prev_axes:
            self.prev_axes = list(msg.axes)

    # ====================================================================== #
    # Main control loop (20 Hz)
    # ====================================================================== #

    def control_loop(self):
        if self.last_joy_msg is None:
            return

        msg = self.last_joy_msg

        button_pressed  = self.get_button_presses(msg)
        trigger_pressed = self.get_trigger_presses(msg)

        self.handle_mode_switching(button_pressed)
        self.handle_emergency_stop(button_pressed)

        if self.rover_mode == RoverMode.MANUAL_DRIVE:
            self.handle_manual_drive(msg)
        elif self.rover_mode == RoverMode.MANUAL_DIG:
            self.handle_manual_dig(msg, button_pressed, trigger_pressed)
        elif self.rover_mode == RoverMode.MANUAL_DEPOSIT:
            self.handle_manual_deposit(msg, button_pressed, trigger_pressed)
        elif self.rover_mode == RoverMode.AUTONOMOUS:
            self.handle_autonomous(button_pressed, trigger_pressed)

        self.prev_buttons = list(msg.buttons)
        self.prev_axes    = list(msg.axes)

        self.publish_status()

    # ====================================================================== #
    # Edge detection helpers
    # ====================================================================== #

    def get_button_presses(self, msg: Joy):
        """Return list of button indices that just transitioned 0→1."""
        pressed = []
        for i in range(min(len(msg.buttons), len(self.prev_buttons))):
            if msg.buttons[i] == 1 and self.prev_buttons[i] == 0:
                pressed.append(i)
        return pressed

    def get_trigger_presses(self, msg: Joy):
        """
        Return set of axis indices for L2/R2 that just crossed TRIGGER_THRESHOLD.
        PS4 triggers are axes (-1.0 rest → +1.0 fully pressed), not buttons.
        """
        pressed = set()
        for axis_idx in (PS4Axis.L2_ANALOG, PS4Axis.R2_ANALOG):
            if axis_idx >= len(msg.axes):
                continue
            was = len(self.prev_axes) > axis_idx and self.prev_axes[axis_idx] > TRIGGER_THRESHOLD
            now = msg.axes[axis_idx] > TRIGGER_THRESHOLD
            if now and not was:
                pressed.add(axis_idx)
        return pressed

    # ====================================================================== #
    # Mode switching
    # ====================================================================== #

    def handle_mode_switching(self, button_pressed):
        if PS4Button.TRIANGLE in button_pressed:
            self.rover_mode = RoverMode.MANUAL_DIG
            self.get_logger().info('Switched to DIGGING MODE')
            self.stop_rover()
        elif PS4Button.CIRCLE in button_pressed:
            self.rover_mode = RoverMode.MANUAL_DEPOSIT
            self.get_logger().info('Switched to DEPOSITION MODE')
            self.stop_rover()
        elif PS4Button.SQUARE in button_pressed:
            self.rover_mode = RoverMode.AUTONOMOUS
            self.get_logger().info('Switched to AUTONOMOUS MODE')
            self.stop_rover()
        elif PS4Button.CROSS in button_pressed:
            self.rover_mode = RoverMode.MANUAL_DRIVE
            self.get_logger().info('Switched to MANUAL DRIVE MODE')
            self.stop_all_operations()

    # ====================================================================== #
    # Emergency stop
    # ====================================================================== #

    def handle_emergency_stop(self, button_pressed):
        if PS4Button.OPTIONS in button_pressed:
            self.emergency_stop = not self.emergency_stop
            self._publish_estop(self.emergency_stop)
            if self.emergency_stop:
                self.get_logger().error('EMERGENCY STOP ACTIVATED')
                # Reset ramp state so rover doesn't lurch when estop clears
                self.ramped_left_speed  = 0.0
                self.ramped_right_speed = 0.0
            else:
                self.get_logger().info('Emergency stop released')

        if PS4Button.SHARE in button_pressed:
            self.emergency_stop = False
            self._publish_estop(False)
            self.get_logger().info('Emergency stop cleared')

        if PS4Button.PS in button_pressed:
            self.reset_all_systems()

    def _publish_estop(self, state: bool):
        msg = Bool()
        msg.data = state
        self.estop_pub.publish(msg)

    # ====================================================================== #
    # Driving
    # ====================================================================== #

    def handle_manual_drive(self, msg: Joy):
        """
        Skid-steer control: left stick Y → left track, right stick Y → right track.

        FIX: speed ramp applied here so stick slams don't produce instant
        full-speed Twist commands. The ramped speeds converge to the target
        at ramp_rate m/s per 50 ms tick (configurable).

        FIX: proper differential-drive kinematics used to convert individual
        wheel speeds to (linear, angular) Twist so motor_controller.py
        reconstructs exactly the same wheel speeds on the other side.
        """
        if self.emergency_stop:
            self.stop_rover()
            return

        # Raw stick values with deadzone
        left_y  = self.apply_deadzone(msg.axes[PS4Axis.LEFT_STICK_Y])
        right_y = self.apply_deadzone(msg.axes[PS4Axis.RIGHT_STICK_Y])

        # PS4 left stick Y is positive-forward on most joy drivers.
        # If the rover drives backward when you push forward, set invert_y_axis: true
        if self.invert_y:
            left_y  = -left_y
            right_y = -right_y

        # Speed multiplier
        speed_mult = 1.0
        if len(msg.buttons) > PS4Button.R1 and msg.buttons[PS4Button.R1]:
            speed_mult = self.turbo_multiplier
        elif len(msg.buttons) > PS4Button.L1 and msg.buttons[PS4Button.L1]:
            speed_mult = 0.5

        # Target wheel speeds in m/s
        target_left  = left_y  * self.max_linear_speed * speed_mult
        target_right = right_y * self.max_linear_speed * speed_mult

        # FIX: ramp toward target — cap the change per tick to ramp_rate
        self.ramped_left_speed  = self._ramp(self.ramped_left_speed,  target_left)
        self.ramped_right_speed = self._ramp(self.ramped_right_speed, target_right)

        # Convert wheel speeds → Twist using differential drive kinematics.
        # motor_controller reverses this with the same wheel_separation value.
        #   linear  = (v_L + v_R) / 2
        #   angular = (v_R - v_L) / wheel_separation
        linear  = (self.ramped_left_speed + self.ramped_right_speed) / 2.0
        angular = (self.ramped_right_speed - self.ramped_left_speed) / self.wheel_separation
        angular = float(np.clip(angular, -self.max_angular_speed, self.max_angular_speed))

        twist = Twist()
        twist.linear.x  = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)

    def _ramp(self, current: float, target: float) -> float:
        """
        Step current toward target by at most ramp_rate.
        Returns the new ramped value.
        """
        delta = target - current
        if abs(delta) <= self.ramp_rate:
            return target
        return current + self.ramp_rate * (1.0 if delta > 0 else -1.0)

    # ====================================================================== #
    # Digging
    # ====================================================================== #

    def handle_manual_dig(self, msg: Joy, button_pressed, trigger_pressed):
        if self.emergency_stop:
            return

        if len(msg.axes) > PS4Axis.DPAD_Y:
            dpad_y = msg.axes[PS4Axis.DPAD_Y]
            if dpad_y > 0.5:
                self.current_dig_depth = min(self.current_dig_depth + self.dig_depth_increment, 0.5)
                self.get_logger().info(f'Dig depth: {self.current_dig_depth:.2f} m')
                depth_msg = Float32()
                depth_msg.data = self.current_dig_depth
                self.dig_depth_pub.publish(depth_msg)
            elif dpad_y < -0.5:
                self.current_dig_depth = max(self.current_dig_depth - self.dig_depth_increment, 0.0)
                self.get_logger().info(f'Dig depth: {self.current_dig_depth:.2f} m')
                depth_msg = Float32()
                depth_msg.data = self.current_dig_depth
                self.dig_depth_pub.publish(depth_msg)

        if PS4Axis.R2_ANALOG in trigger_pressed and not self.digging_active:
            self.get_logger().info('Starting digging')
            self._publish_string(self.dig_cmd_pub, 'start')
            self.digging_active = True

        if PS4Axis.L2_ANALOG in trigger_pressed and self.digging_active:
            self.get_logger().info('Stopping digging')
            self._publish_string(self.dig_cmd_pub, 'stop')
            self.digging_active = False

        if PS4Button.R1 in button_pressed:
            self.get_logger().info('Raising excavator')
            self._publish_string(self.dig_cmd_pub, 'raise')

        if PS4Button.L1 in button_pressed:
            self.get_logger().info('Lowering excavator')
            self._publish_string(self.dig_cmd_pub, 'lower')

    # ====================================================================== #
    # Deposition
    # ====================================================================== #

    def handle_manual_deposit(self, msg: Joy, button_pressed, trigger_pressed):
        if self.emergency_stop:
            return

        if len(msg.axes) > PS4Axis.DPAD_Y:
            dpad_y = msg.axes[PS4Axis.DPAD_Y]
            if dpad_y > 0.5:
                self.dump_angle = min(self.dump_angle + self.angle_increment, 90.0)
                self.get_logger().info(f'Dump angle: {self.dump_angle:.1f}°')
            elif dpad_y < -0.5:
                self.dump_angle = max(self.dump_angle - self.angle_increment, 0.0)
                self.get_logger().info(f'Dump angle: {self.dump_angle:.1f}°')

        if PS4Axis.R2_ANALOG in trigger_pressed:
            self.get_logger().info('Dumping material')
            self._publish_string(self.deposit_cmd_pub, 'dump')
            self.depositing_active = True

        if PS4Axis.L2_ANALOG in trigger_pressed:
            self.get_logger().info('Returning to neutral')
            self._publish_string(self.deposit_cmd_pub, 'reset')
            self.depositing_active = False

    # ====================================================================== #
    # Autonomous
    # ====================================================================== #

    def handle_autonomous(self, button_pressed, trigger_pressed):
        if self.emergency_stop:
            return

        if PS4Axis.R2_ANALOG in trigger_pressed:
            self.get_logger().info('Starting autonomous mission')
            self._publish_string(self.mission_cmd_pub, 'start')

        if PS4Axis.L2_ANALOG in trigger_pressed:
            self.get_logger().info('Stopping mission')
            self._publish_string(self.mission_cmd_pub, 'stop')

        if PS4Button.R1 in button_pressed:
            self.get_logger().info('Pausing mission')
            self._publish_string(self.mission_cmd_pub, 'pause')

        if PS4Button.L1 in button_pressed:
            self.get_logger().info('Resuming mission')
            self._publish_string(self.mission_cmd_pub, 'resume')

    # ====================================================================== #
    # Utility
    # ====================================================================== #

    def apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        sign   = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled

    def stop_rover(self):
        self.ramped_left_speed  = 0.0
        self.ramped_right_speed = 0.0
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def stop_all_operations(self):
        self.stop_rover()
        self._publish_string(self.dig_cmd_pub,     'stop')
        self._publish_string(self.deposit_cmd_pub, 'stop')
        self.digging_active    = False
        self.depositing_active = False

    def reset_all_systems(self):
        self.get_logger().info('Resetting all systems')
        self.emergency_stop = False
        self._publish_estop(False)
        self.stop_all_operations()
        self._publish_string(self.dig_cmd_pub,     'reset')
        self._publish_string(self.deposit_cmd_pub, 'reset')
        self.rover_mode = RoverMode.MANUAL_DRIVE
        self.get_logger().info('All systems reset')

    def _publish_string(self, publisher, text: str):
        msg = String()
        msg.data = text
        publisher.publish(msg)

    def publish_status(self):
        mode_name    = self.rover_mode.name.replace('_', ' ')
        estop_status = 'E-STOP' if self.emergency_stop else 'OK'
        status       = f"{estop_status} | Mode: {mode_name}"

        if self.rover_mode == RoverMode.MANUAL_DRIVE:
            status += (f" | L: {self.ramped_left_speed:+.2f} m/s"
                       f"  R: {self.ramped_right_speed:+.2f} m/s")
        elif self.rover_mode == RoverMode.MANUAL_DIG:
            status += f" | Depth: {self.current_dig_depth:.2f} m"
        elif self.rover_mode == RoverMode.MANUAL_DEPOSIT:
            status += f" | Angle: {self.dump_angle:.1f} deg"

        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PlayStationController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
