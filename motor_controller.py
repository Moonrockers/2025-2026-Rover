#!/usr/bin/env python3
"""
Rover Motor Controller
Handles differential drive motor control for the moonrockers rover
Supports various motor drivers: L298N, Sabertooth, RoboClaw, etc.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Float32
import numpy as np
from collections import deque
import time

# Try importing GPIO library (if on Jetson/Pi)
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    try:
        import RPi.GPIO as GPIO
        GPIO_AVAILABLE = True
    except ImportError:
        GPIO_AVAILABLE = False
        print("Warning: GPIO not available. Running in simulation mode.")


class MotorController(Node):
    """
    Motor controller for differential drive rover
    Converts Twist commands to individual motor speeds
    """
    
    def __init__(self):
        super().__init__('motor_controller')
        
        # Declare parameters
        self.declare_parameter('motor_driver', 'l298n')  # l298n, sabertooth, roboclaw, pwm
        self.declare_parameter('control_mode', 'differential')  # differential or skid_steer
        self.declare_parameter('wheel_separation', 0.5)  # meters
        self.declare_parameter('wheel_radius', 0.1)  # meters
        self.declare_parameter('max_speed', 1.0)  # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('enable_safety', True)
        self.declare_parameter('timeout', 1.0)  # seconds
        
        # GPIO pins for L298N (example - adjust for your wiring)
        self.declare_parameter('left_motor_pins', [23, 24])  # [forward, backward]
        self.declare_parameter('right_motor_pins', [17, 27])  # [forward, backward]
        self.declare_parameter('left_pwm_pin', 18)
        self.declare_parameter('right_pwm_pin', 12)
        self.declare_parameter('pwm_frequency', 1000)  # Hz
        
        # Get parameters
        self.motor_driver = self.get_parameter('motor_driver').value
        self.control_mode = self.get_parameter('control_mode').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_speed = self.get_parameter('max_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.enable_safety = self.get_parameter('enable_safety').value
        self.timeout = self.get_parameter('timeout').value
        
        # State
        self.emergency_stop = False
        self.last_cmd_time = time.time()
        self.current_left_speed = 0.0
        self.current_right_speed = 0.0
        
        # Moving average for smooth control
        self.left_speed_buffer = deque(maxlen=5)
        self.right_speed_buffer = deque(maxlen=5)
        
        # Initialize motor driver
        self.setup_motors()
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel',
            self.cmd_vel_callback, 10
        )
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop',
            self.estop_callback, 10
        )
        self.joy_sub = self.create_subscription(
            Joy, '/joy',
            self.joy_callback, 10
        )
        
        # Publishers
        self.motor_status_pub = self.create_publisher(
            Float32, '/motor/left_speed', 10
        )
        self.motor_status_pub = self.create_publisher(
            Float32, '/motor/right_speed', 10
        )
        
        # Safety timer
        self.create_timer(0.1, self.safety_check)
        
        self.get_logger().info(f'Motor controller initialized - Driver: {self.motor_driver}, Mode: {self.control_mode}')
        if not GPIO_AVAILABLE:
            self.get_logger().warn('Running in SIMULATION mode - no GPIO control')
    
    def setup_motors(self):
        """Initialize motor driver based on type"""
        if not GPIO_AVAILABLE:
            self.get_logger().warn('GPIO not available - skipping motor setup')
            return
        
        if self.motor_driver == 'l298n':
            self.setup_l298n()
        elif self.motor_driver == 'sabertooth':
            self.setup_sabertooth()
        elif self.motor_driver == 'roboclaw':
            self.setup_roboclaw()
        elif self.motor_driver == 'pwm':
            self.setup_pwm()
        else:
            self.get_logger().error(f'Unknown motor driver: {self.motor_driver}')
    
    def setup_l298n(self):
        """Setup L298N dual H-bridge motor driver"""
        GPIO.setmode(GPIO.BCM)
        
        # Get pin configurations
        left_pins = self.get_parameter('left_motor_pins').value
        right_pins = self.get_parameter('right_motor_pins').value
        left_pwm_pin = self.get_parameter('left_pwm_pin').value
        right_pwm_pin = self.get_parameter('right_pwm_pin').value
        pwm_freq = self.get_parameter('pwm_frequency').value
        
        # Setup direction pins
        for pin in left_pins + right_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        
        # Setup PWM pins
        GPIO.setup(left_pwm_pin, GPIO.OUT)
        GPIO.setup(right_pwm_pin, GPIO.OUT)
        
        self.left_pwm = GPIO.PWM(left_pwm_pin, pwm_freq)
        self.right_pwm = GPIO.PWM(right_pwm_pin, pwm_freq)
        
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        
        self.get_logger().info('L298N motor driver initialized')
    
    def setup_sabertooth(self):
        """Setup Sabertooth motor driver (serial control)"""
        # TODO: Implement Sabertooth serial protocol
        self.get_logger().warn('Sabertooth driver not yet implemented')
        pass
    
    def setup_roboclaw(self):
        """Setup RoboClaw motor driver"""
        try:
            from roboclaw_3 import Roboclaw
            self.roboclaw = Roboclaw("/dev/ttyACM0", 115200)
            self.roboclaw.Open()
            self.roboclaw_address = 0x80
            self.get_logger().info('RoboClaw motor driver initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RoboClaw: {e}')
    
    def setup_pwm(self):
        """Setup generic PWM control"""
        # Similar to L298N but without direction pins
        pass
    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist message to differential drive motor speeds
        
        Twist: linear.x (forward/backward), angular.z (rotation)
        Output: left_speed, right_speed
        """
        if self.emergency_stop:
            self.stop_motors()
            return
        
        # Update command timestamp
        self.last_cmd_time = time.time()
        
        # Extract velocities
        linear = msg.linear.x  # m/s forward
        angular = msg.angular.z  # rad/s counter-clockwise
        
        # Limit velocities
        linear = np.clip(linear, -self.max_speed, self.max_speed)
        angular = np.clip(angular, -self.max_angular_speed, self.max_angular_speed)
        
        # Differential drive kinematics
        # v_left = v - (w * L) / 2
        # v_right = v + (w * L) / 2
        left_speed = linear - (angular * self.wheel_separation / 2.0)
        right_speed = linear + (angular * self.wheel_separation / 2.0)
        
        # Apply smoothing
        self.left_speed_buffer.append(left_speed)
        self.right_speed_buffer.append(right_speed)
        
        left_speed = np.mean(self.left_speed_buffer)
        right_speed = np.mean(self.right_speed_buffer)
        
        # Set motor speeds
        self.set_motor_speeds(left_speed, right_speed)
    
    def set_motor_speeds(self, left_speed, right_speed):
        """
        Set individual motor speeds
        
        Args:
            left_speed: m/s (positive = forward)
            right_speed: m/s (positive = forward)
        """
        if self.emergency_stop:
            self.stop_motors()
            return
        
        # Store current speeds
        self.current_left_speed = left_speed
        self.current_right_speed = right_speed
        
        # Convert to motor driver commands
        if self.motor_driver == 'l298n':
            self.set_l298n_speeds(left_speed, right_speed)
        elif self.motor_driver == 'roboclaw':
            self.set_roboclaw_speeds(left_speed, right_speed)
        elif self.motor_driver == 'sabertooth':
            self.set_sabertooth_speeds(left_speed, right_speed)
        else:
            # Simulation mode
            pass
        
        # Publish motor status
        self.publish_motor_status()
    
    def set_l298n_speeds(self, left_speed, right_speed):
        """Set L298N motor speeds"""
        if not GPIO_AVAILABLE:
            return
        
        # Get pins
        left_pins = self.get_parameter('left_motor_pins').value
        right_pins = self.get_parameter('right_motor_pins').value
        
        # Left motor
        left_duty = abs(left_speed) / self.max_speed * 100.0
        left_duty = np.clip(left_duty, 0, 100)
        
        if left_speed > 0:
            GPIO.output(left_pins[0], GPIO.HIGH)
            GPIO.output(left_pins[1], GPIO.LOW)
        elif left_speed < 0:
            GPIO.output(left_pins[0], GPIO.LOW)
            GPIO.output(left_pins[1], GPIO.HIGH)
        else:
            GPIO.output(left_pins[0], GPIO.LOW)
            GPIO.output(left_pins[1], GPIO.LOW)
        
        self.left_pwm.ChangeDutyCycle(left_duty)
        
        # Right motor
        right_duty = abs(right_speed) / self.max_speed * 100.0
        right_duty = np.clip(right_duty, 0, 100)
        
        if right_speed > 0:
            GPIO.output(right_pins[0], GPIO.HIGH)
            GPIO.output(right_pins[1], GPIO.LOW)
        elif right_speed < 0:
            GPIO.output(right_pins[0], GPIO.LOW)
            GPIO.output(right_pins[1], GPIO.HIGH)
        else:
            GPIO.output(right_pins[0], GPIO.LOW)
            GPIO.output(right_pins[1], GPIO.LOW)
        
        self.right_pwm.ChangeDutyCycle(right_duty)
    
    def set_roboclaw_speeds(self, left_speed, right_speed):
        """Set RoboClaw motor speeds"""
        if not hasattr(self, 'roboclaw'):
            return
        
        # Convert m/s to encoder counts per second
        # This depends on your encoder resolution
        # Example: 1000 counts/revolution, wheel circumference = 2*pi*r
        counts_per_meter = 1000 / (2 * np.pi * self.wheel_radius)
        
        left_qpps = int(left_speed * counts_per_meter)
        right_qpps = int(right_speed * counts_per_meter)
        
        # RoboClaw speed command
        self.roboclaw.SpeedM1M2(self.roboclaw_address, left_qpps, right_qpps)
    
    def set_sabertooth_speeds(self, left_speed, right_speed):
        """Set Sabertooth motor speeds"""
        # TODO: Implement Sabertooth packet protocol
        pass
    
    def stop_motors(self):
        """Emergency stop - immediately halt all motors"""
        self.set_motor_speeds(0.0, 0.0)
        self.left_speed_buffer.clear()
        self.right_speed_buffer.clear()
        self.get_logger().warn('Motors stopped')
    
    def estop_callback(self, msg):
        """Handle emergency stop command"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.stop_motors()
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
        else:
            self.get_logger().info('Emergency stop released')
    
    def joy_callback(self, msg):
        """Handle joystick input for manual control"""
        # Typical joystick mapping:
        # Left stick Y-axis: linear velocity
        # Right stick X-axis: angular velocity
        # Button 0 (A): emergency stop
        
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            self.emergency_stop = True
            self.stop_motors()
            return
        
        if len(msg.axes) >= 4:
            linear = msg.axes[1] * self.max_speed  # Left stick Y
            angular = msg.axes[2] * self.max_angular_speed  # Right stick X
            
            # Create Twist message
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            
            # Process as normal command
            self.cmd_vel_callback(twist)
    
    def safety_check(self):
        """Check for command timeout and other safety conditions"""
        if not self.enable_safety:
            return
        
        # Check command timeout
        time_since_cmd = time.time() - self.last_cmd_time
        if time_since_cmd > self.timeout and not self.emergency_stop:
            if self.current_left_speed != 0 or self.current_right_speed != 0:
                self.get_logger().warn(f'Command timeout ({time_since_cmd:.1f}s) - stopping motors')
                self.stop_motors()
    
    def publish_motor_status(self):
        """Publish current motor speeds"""
        # Could expand this to include current draw, temperature, etc.
        pass
    
    def shutdown(self):
        """Clean shutdown - stop motors and cleanup GPIO"""
        self.get_logger().info('Shutting down motor controller')
        self.stop_motors()
        
        if GPIO_AVAILABLE and self.motor_driver == 'l298n':
            self.left_pwm.stop()
            self.right_pwm.stop()
            GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
