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


class ControlMode(Enum):
    """Control modes"""
    MANUAL_DRIVE = 0      # Direct motor control
    MANUAL_DIG = 1        # Manual digging operations
    MANUAL_DEPOSIT = 2    # Manual deposition operations
    AUTONOMOUS = 3        # Mission controller takes over


class PS4Button:
    """PS4 Controller button mapping"""
    # Face buttons
    CROSS = 0       # X button
    CIRCLE = 1      # O button
    SQUARE = 2      # Square button
    TRIANGLE = 3    # Triangle button
    
    # Shoulder buttons
    L1 = 4
    R1 = 5
    L2 = 6
    R2 = 7
    
    # Menu buttons
    SHARE = 8
    OPTIONS = 9
    
    # Stick buttons
    L3 = 10
    R3 = 11
    
    # D-Pad (usually axes, but sometimes buttons)
    PS = 12
    TOUCHPAD = 13


class PS4Axis:
    """PS4 Controller axis mapping"""
    LEFT_STICK_X = 0      # Left/right on left stick
    LEFT_STICK_Y = 1      # Up/down on left stick
    RIGHT_STICK_X = 2     # Left/right on right stick
    RIGHT_STICK_Y = 3     # Up/down on right stick
    L2_ANALOG = 4         # L2 trigger pressure
    R2_ANALOG = 5         # R2 trigger pressure
    DPAD_X = 6           # D-pad left/right
    DPAD_Y = 7           # D-pad up/down


class PlayStationController(Node):
    """
    PlayStation controller interface for rover control
    Maps PS4/PS5 controller to rover functions
    """
    
    def __init__(self):
        super().__init__('playstation_controller')
        
        # Parameters
        self.declare_parameter('controller_type', 'ps4')  # ps4, ps5
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('turbo_multiplier', 2.0)
        self.declare_parameter('enable_rumble', True)
        self.declare_parameter('invert_y_axis', False)
        
        # Get parameters
        self.controller_type = self.get_parameter('controller_type').value
        self.deadzone = self.get_parameter('deadzone').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.turbo_multiplier = self.get_parameter('turbo_multiplier').value
        self.enable_rumble = self.get_parameter('enable_rumble').value
        self.invert_y = self.get_parameter('invert_y_axis').value
        
        # State
        self.control_mode = ControlMode.MANUAL_DRIVE
        self.turbo_mode = False
        self.emergency_stop = False
        self.last_joy_msg = None
        
        # Button state tracking (for edge detection)
        self.prev_buttons = []
        
        # Digging state
        self.digging_active = False
        self.current_dig_depth = 0.0
        self.dig_depth_increment = 0.05  # 5cm per button press
        
        # Deposition state
        self.depositing_active = False
        self.dump_angle = 0.0
        self.angle_increment = 5.0  # 5 degrees per button press
        
        # Subscribers
        self.joy_sub = self.create_subscription(
            Joy, '/joy',
            self.joy_callback, 10
        )
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.dig_cmd_pub = self.create_publisher(String, '/digging/command', 10)
        self.dig_depth_pub = self.create_publisher(Float32, '/digging/target_depth', 10)
        self.deposit_cmd_pub = self.create_publisher(String, '/deposition/command', 10)
        self.mission_cmd_pub = self.create_publisher(String, '/mission/command', 10)
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', 10)
        
        # Status publisher for controller feedback
        self.status_pub = self.create_publisher(String, '/controller/status', 10)
        
        # Timer for control loop
        self.create_timer(0.05, self.control_loop)  # 20 Hz
        
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'PlayStation {self.controller_type.upper()} Controller Ready!')
        self.get_logger().info('=' * 60)
        self.print_controls()
    
    def print_controls(self):
        """Print controller mapping"""
        self.get_logger().info('')
        self.get_logger().info('CONTROL SCHEME - SKID-STEER (TANK) MODE:')
        self.get_logger().info('-' * 60)
        self.get_logger().info('DRIVING MODE (Default):')
        self.get_logger().info('  Left Stick Y  → LEFT TRACK speed (forward/backward)')
        self.get_logger().info('  Right Stick Y → RIGHT TRACK speed (forward/backward)')
        self.get_logger().info('  R1            → Turbo mode (hold for 2x speed)')
        self.get_logger().info('  L1            → Precision mode (hold for 0.5x speed)')
        self.get_logger().info('')
        self.get_logger().info('  💡 SKID-STEER TIPS:')
        self.get_logger().info('     Both sticks forward  → Drive straight')
        self.get_logger().info('     Left forward, Right back → Spin left')
        self.get_logger().info('     Both at different speeds → Gradual turn')
        self.get_logger().info('')
        self.get_logger().info('MODE SWITCHING:')
        self.get_logger().info('  Triangle      → Switch to DIGGING mode')
        self.get_logger().info('  Circle        → Switch to DEPOSITION mode')
        self.get_logger().info('  Square        → Switch to AUTONOMOUS mode')
        self.get_logger().info('  Cross (X)     → Return to MANUAL DRIVE')
        self.get_logger().info('')
        self.get_logger().info('DIGGING MODE:')
        self.get_logger().info('  D-Pad Up      → Increase dig depth')
        self.get_logger().info('  D-Pad Down    → Decrease dig depth')
        self.get_logger().info('  R2            → Start digging')
        self.get_logger().info('  L2            → Stop digging')
        self.get_logger().info('  R1            → Raise excavator')
        self.get_logger().info('  L1            → Lower excavator')
        self.get_logger().info('')
        self.get_logger().info('DEPOSITION MODE:')
        self.get_logger().info('  D-Pad Up      → Increase dump angle')
        self.get_logger().info('  D-Pad Down    → Decrease dump angle')
        self.get_logger().info('  R2            → Dump material')
        self.get_logger().info('  L2            → Return to neutral')
        self.get_logger().info('')
        self.get_logger().info('AUTONOMOUS MODE:')
        self.get_logger().info('  R2            → Start mission')
        self.get_logger().info('  L2            → Stop mission')
        self.get_logger().info('  R1            → Pause mission')
        self.get_logger().info('  L1            → Resume mission')
        self.get_logger().info('')
        self.get_logger().info('EMERGENCY:')
        self.get_logger().info('  OPTIONS       → Emergency stop (toggle)')
        self.get_logger().info('  SHARE         → Reset emergency stop')
        self.get_logger().info('  PS Button     → Reset all systems')
        self.get_logger().info('-' * 60)
        self.get_logger().info('')
    
    def joy_callback(self, msg):
        """Store latest joystick message"""
        self.last_joy_msg = msg
        
        # Initialize previous buttons if needed
        if not self.prev_buttons:
            self.prev_buttons = list(msg.buttons)
    
    def control_loop(self):
        """Main control loop"""
        if self.last_joy_msg is None:
            return
        
        msg = self.last_joy_msg
        
        # Check for button presses (edge detection)
        button_pressed = self.get_button_presses(msg)
        
        # Handle mode switching
        self.handle_mode_switching(button_pressed)
        
        # Handle emergency stop
        self.handle_emergency_stop(button_pressed)
        
        # Execute current mode
        if self.control_mode == ControlMode.MANUAL_DRIVE:
            self.handle_manual_drive(msg)
        elif self.control_mode == ControlMode.MANUAL_DIG:
            self.handle_manual_dig(msg, button_pressed)
        elif self.control_mode == ControlMode.MANUAL_DEPOSIT:
            self.handle_manual_deposit(msg, button_pressed)
        elif self.control_mode == ControlMode.AUTONOMOUS:
            self.handle_autonomous(button_pressed)
        
        # Update previous buttons
        self.prev_buttons = list(msg.buttons)
        
        # Publish status
        self.publish_status()
    
    def get_button_presses(self, msg):
        """Detect button press events (rising edge)"""
        pressed = []
        for i in range(min(len(msg.buttons), len(self.prev_buttons))):
            if msg.buttons[i] == 1 and self.prev_buttons[i] == 0:
                pressed.append(i)
        return pressed
    
    def handle_mode_switching(self, button_pressed):
        """Handle control mode switching"""
        if PS4Button.TRIANGLE in button_pressed:
            self.control_mode = ControlMode.MANUAL_DIG
            self.get_logger().info('🔺 Switched to DIGGING MODE')
            self.stop_rover()
        
        elif PS4Button.CIRCLE in button_pressed:
            self.control_mode = ControlMode.MANUAL_DEPOSIT
            self.get_logger().info('⭕ Switched to DEPOSITION MODE')
            self.stop_rover()
        
        elif PS4Button.SQUARE in button_pressed:
            self.control_mode = ControlMode.AUTONOMOUS
            self.get_logger().info('🟦 Switched to AUTONOMOUS MODE')
            self.stop_rover()
        
        elif PS4Button.CROSS in button_pressed:
            self.control_mode = ControlMode.MANUAL_DRIVE
            self.get_logger().info('❌ Switched to MANUAL DRIVE MODE')
            self.stop_all_operations()
    
    def handle_emergency_stop(self, button_pressed):
        """Handle emergency stop button"""
        if PS4Button.OPTIONS in button_pressed:
            self.emergency_stop = not self.emergency_stop
            estop_msg = Bool()
            estop_msg.data = self.emergency_stop
            self.estop_pub.publish(estop_msg)
            
            if self.emergency_stop:
                self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED!')
            else:
                self.get_logger().info('✅ Emergency stop released')
        
        if PS4Button.SHARE in button_pressed:
            self.emergency_stop = False
            estop_msg = Bool()
            estop_msg.data = False
            self.estop_pub.publish(estop_msg)
            self.get_logger().info('✅ Emergency stop cleared')
        
        if PS4Button.PS in button_pressed:
            self.get_logger().info('🔄 Resetting all systems...')
            self.reset_all_systems()
    
    def handle_manual_drive(self, msg):
        """Handle manual driving with joysticks - SKID STEER MODE"""
        if self.emergency_stop:
            self.stop_rover()
            return
        
        # Get stick values for skid-steer
        left_y = self.apply_deadzone(msg.axes[PS4Axis.LEFT_STICK_Y])
        right_y = self.apply_deadzone(msg.axes[PS4Axis.RIGHT_STICK_Y])
        
        # Invert Y if needed (typically needed for PS controllers)
        if self.invert_y:
            left_y = -left_y
            right_y = -right_y
        
        # Check for turbo or slow mode
        speed_multiplier = 1.0
        if len(msg.buttons) > PS4Button.R1 and msg.buttons[PS4Button.R1]:
            speed_multiplier = self.turbo_multiplier  # Turbo
        elif len(msg.buttons) > PS4Button.L1 and msg.buttons[PS4Button.L1]:
            speed_multiplier = 0.5  # Slow/precision mode
        
        # SKID-STEER CONTROL
        # Left stick Y controls left wheel
        # Right stick Y controls right wheel
        left_speed = left_y * self.max_linear_speed * speed_multiplier
        right_speed = right_y * self.max_linear_speed * speed_multiplier
        
        # Convert individual wheel speeds to differential drive (Twist)
        # Linear velocity = average of both wheels
        # Angular velocity = difference between wheels
        linear = (left_speed + right_speed) / 2.0
        
        # Calculate angular velocity from wheel speed difference
        # angular = (right_speed - left_speed) / wheel_separation
        # For Twist, we'll approximate using the difference scaled to max_angular_speed
        wheel_diff = (right_speed - left_speed) / 2.0
        angular = (wheel_diff / self.max_linear_speed) * self.max_angular_speed
        
        # Create and publish twist message
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_vel_pub.publish(twist)
    
    def handle_manual_dig(self, msg, button_pressed):
        """Handle manual digging operations"""
        if self.emergency_stop:
            return
        
        # D-Pad for depth adjustment
        if len(msg.axes) > PS4Axis.DPAD_Y:
            dpad_y = msg.axes[PS4Axis.DPAD_Y]
            if dpad_y > 0.5:  # Up
                self.current_dig_depth += self.dig_depth_increment
                self.current_dig_depth = min(self.current_dig_depth, 0.5)  # Max 50cm
                self.get_logger().info(f'Dig depth: {self.current_dig_depth:.2f}m')
                
                # Publish new depth
                depth_msg = Float32()
                depth_msg.data = self.current_dig_depth
                self.dig_depth_pub.publish(depth_msg)
            
            elif dpad_y < -0.5:  # Down
                self.current_dig_depth -= self.dig_depth_increment
                self.current_dig_depth = max(self.current_dig_depth, 0.0)
                self.get_logger().info(f'Dig depth: {self.current_dig_depth:.2f}m')
                
                depth_msg = Float32()
                depth_msg.data = self.current_dig_depth
                self.dig_depth_pub.publish(depth_msg)
        
        # R2 to start digging
        if len(msg.buttons) > PS4Button.R2 and msg.buttons[PS4Button.R2]:
            if not self.digging_active:
                self.get_logger().info('⛏️  Starting digging operation')
                cmd_msg = String()
                cmd_msg.data = 'start'
                self.dig_cmd_pub.publish(cmd_msg)
                self.digging_active = True
        
        # L2 to stop digging
        if len(msg.buttons) > PS4Button.L2 and msg.buttons[PS4Button.L2]:
            if self.digging_active:
                self.get_logger().info('⏹️  Stopping digging')
                cmd_msg = String()
                cmd_msg.data = 'stop'
                self.dig_cmd_pub.publish(cmd_msg)
                self.digging_active = False
        
        # R1 to raise excavator
        if PS4Button.R1 in button_pressed:
            self.get_logger().info('⬆️  Raising excavator')
            cmd_msg = String()
            cmd_msg.data = 'raise'
            self.dig_cmd_pub.publish(cmd_msg)
        
        # L1 to lower excavator
        if PS4Button.L1 in button_pressed:
            self.get_logger().info('⬇️  Lowering excavator')
            cmd_msg = String()
            cmd_msg.data = 'lower'
            self.dig_cmd_pub.publish(cmd_msg)
    
    def handle_manual_deposit(self, msg, button_pressed):
        """Handle manual deposition operations"""
        if self.emergency_stop:
            return
        
        # D-Pad for angle adjustment
        if len(msg.axes) > PS4Axis.DPAD_Y:
            dpad_y = msg.axes[PS4Axis.DPAD_Y]
            if dpad_y > 0.5:  # Up
                self.dump_angle += self.angle_increment
                self.dump_angle = min(self.dump_angle, 90.0)
                self.get_logger().info(f'Dump angle: {self.dump_angle:.1f}°')
            
            elif dpad_y < -0.5:  # Down
                self.dump_angle -= self.angle_increment
                self.dump_angle = max(self.dump_angle, 0.0)
                self.get_logger().info(f'Dump angle: {self.dump_angle:.1f}°')
        
        # R2 to dump
        if PS4Button.R2 in button_pressed:
            self.get_logger().info('📤 Dumping material')
            cmd_msg = String()
            cmd_msg.data = 'dump'
            self.deposit_cmd_pub.publish(cmd_msg)
            self.depositing_active = True
        
        # L2 to return to neutral
        if PS4Button.L2 in button_pressed:
            self.get_logger().info('↩️  Returning to neutral')
            cmd_msg = String()
            cmd_msg.data = 'reset'
            self.deposit_cmd_pub.publish(cmd_msg)
            self.depositing_active = False
    
    def handle_autonomous(self, button_pressed):
        """Handle autonomous mission control"""
        if self.emergency_stop:
            return
        
        # R2 to start mission
        if PS4Button.R2 in button_pressed:
            self.get_logger().info('🚀 Starting autonomous mission')
            cmd_msg = String()
            cmd_msg.data = 'start'
            self.mission_cmd_pub.publish(cmd_msg)
        
        # L2 to stop mission
        if PS4Button.L2 in button_pressed:
            self.get_logger().info('⏹️  Stopping mission')
            cmd_msg = String()
            cmd_msg.data = 'stop'
            self.mission_cmd_pub.publish(cmd_msg)
        
        # R1 to pause
        if PS4Button.R1 in button_pressed:
            self.get_logger().info('⏸️  Pausing mission')
            cmd_msg = String()
            cmd_msg.data = 'pause'
            self.mission_cmd_pub.publish(cmd_msg)
        
        # L1 to resume
        if PS4Button.L1 in button_pressed:
            self.get_logger().info('▶️  Resuming mission')
            cmd_msg = String()
            cmd_msg.data = 'resume'
            self.mission_cmd_pub.publish(cmd_msg)
    
    def apply_deadzone(self, value):
        """Apply deadzone to joystick value"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale from deadzone to 1.0
        sign = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - self.deadzone) / (1.0 - self.deadzone)
        return sign * scaled
    
    def stop_rover(self):
        """Stop rover movement"""
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
    
    def stop_all_operations(self):
        """Stop all rover operations"""
        self.stop_rover()
        
        # Stop digging
        dig_msg = String()
        dig_msg.data = 'stop'
        self.dig_cmd_pub.publish(dig_msg)
        self.digging_active = False
        
        # Stop deposition
        deposit_msg = String()
        deposit_msg.data = 'stop'
        self.deposit_cmd_pub.publish(deposit_msg)
        self.depositing_active = False
    
    def reset_all_systems(self):
        """Reset all rover systems"""
        # Clear emergency stop
        self.emergency_stop = False
        estop_msg = Bool()
        estop_msg.data = False
        self.estop_pub.publish(estop_msg)
        
        # Stop everything
        self.stop_all_operations()
        
        # Reset digging
        dig_msg = String()
        dig_msg.data = 'reset'
        self.dig_cmd_pub.publish(dig_msg)
        
        # Reset deposition
        deposit_msg = String()
        deposit_msg.data = 'reset'
        self.deposit_cmd_pub.publish(deposit_msg)
        
        # Return to manual drive
        self.control_mode = ControlMode.MANUAL_DRIVE
        
        self.get_logger().info('✅ All systems reset')
    
    def publish_status(self):
        """Publish controller status"""
        status_msg = String()
        mode_name = self.control_mode.name.replace('_', ' ')
        estop_status = '🚨 E-STOP' if self.emergency_stop else '✅'
        
        status_msg.data = f"{estop_status} | Mode: {mode_name}"
        
        if self.control_mode == ControlMode.MANUAL_DIG:
            status_msg.data += f" | Depth: {self.current_dig_depth:.2f}m"
        elif self.control_mode == ControlMode.MANUAL_DEPOSIT:
            status_msg.data += f" | Angle: {self.dump_angle:.1f}°"
        
        self.status_pub.publish(status_msg)


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
