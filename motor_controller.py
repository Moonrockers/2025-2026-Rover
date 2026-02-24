#!/usr/bin/env python3
"""
Rover Motor Controller
Handles differential drive motor control for the moonrockers rover.

Supported motor drivers:
    l298n     - L298N dual H-bridge via GPIO (original)
    sabertooth - Sabertooth via serial (stub)
    roboclaw  - RoboClaw via serial
    pwm       - Generic PWM via GPIO
    sparkmax  - REV SPARK MAX via SocketCAN  <-- NEW

Hardware setup for SPARK MAX:
    Jetson Nano SPI → MCP2515 CAN module → CAN bus → SPARK MAX(es)

    Bring up the interface before launching:
        sudo modprobe mcp251x
        sudo ip link set can0 up type can bitrate 1000000
        sudo ip link set can0 txqueuelen 1000

    Install python-can:
        pip3 install python-can
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
import threading

# GPIO (Jetson / Pi only)
try:
    import Jetson.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    try:
        import RPi.GPIO as GPIO
        GPIO_AVAILABLE = True
    except ImportError:
        GPIO_AVAILABLE = False

# SocketCAN / python-can (SPARK MAX path)
try:
    import can
    CAN_AVAILABLE = True
except ImportError:
    CAN_AVAILABLE = False


class MotorController(Node):
    """
    Motor controller for differential drive rover.
    Converts Twist commands to individual motor speeds.
    """

    def __init__(self):
        super().__init__('motor_controller')

        # ------------------------------------------------------------------ #
        # Parameters
        # ------------------------------------------------------------------ #
        self.declare_parameter('motor_driver', 'sparkmax')
        self.declare_parameter('control_mode', 'differential')
        self.declare_parameter('wheel_separation', 0.5)   # m
        self.declare_parameter('wheel_radius', 0.1)       # m
        self.declare_parameter('max_speed', 1.0)          # m/s
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s
        self.declare_parameter('enable_safety', True)
        self.declare_parameter('timeout', 1.0)            # s

        # GPIO pins (L298N)
        self.declare_parameter('left_motor_pins', [23, 24])
        self.declare_parameter('right_motor_pins', [17, 27])
        self.declare_parameter('left_pwm_pin', 18)
        self.declare_parameter('right_pwm_pin', 12)
        self.declare_parameter('pwm_frequency', 1000)

        # SPARK MAX CAN parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 1000000)

        # Leader motors (front) — receive commands every cycle
        self.declare_parameter('spark_left_front_id',  1)
        self.declare_parameter('spark_right_front_id', 2)

        # Follower motors (rear) — configured once at startup, then mirror leaders
        self.declare_parameter('spark_left_rear_id',  3)
        self.declare_parameter('spark_right_rear_id', 4)

        self.declare_parameter('spark_left_inverted',  False)
        self.declare_parameter('spark_right_inverted', True)

        # If True, rear motors invert relative to their leader.
        # Usually False since they face the same direction on the same side.
        self.declare_parameter('spark_rear_invert_follower', False)

        # Read parameters
        self.motor_driver       = self.get_parameter('motor_driver').value
        self.control_mode       = self.get_parameter('control_mode').value
        self.wheel_separation   = self.get_parameter('wheel_separation').value
        self.wheel_radius       = self.get_parameter('wheel_radius').value
        self.max_speed          = self.get_parameter('max_speed').value
        self.max_angular_speed  = self.get_parameter('max_angular_speed').value
        self.enable_safety      = self.get_parameter('enable_safety').value
        self.timeout            = self.get_parameter('timeout').value

        # ------------------------------------------------------------------ #
        # State
        # ------------------------------------------------------------------ #
        self.emergency_stop      = False
        self.last_cmd_time       = time.time()
        self.current_left_speed  = 0.0
        self.current_right_speed = 0.0
        self._shutting_down      = False

        self.left_speed_buffer  = deque(maxlen=5)
        self.right_speed_buffer = deque(maxlen=5)

        # SPARK MAX driver objects (set in setup_sparkmax)
        self._can_bus          = None
        self._left_spark       = None   # left  front (leader)
        self._right_spark      = None   # right front (leader)
        self._left_rear_spark  = None   # left  rear  (follower)
        self._right_rear_spark = None   # right rear  (follower)
        self._can_rx_thread    = None

        # ------------------------------------------------------------------ #
        # Motor driver initialisation
        # ------------------------------------------------------------------ #
        self.setup_motors()

        # ------------------------------------------------------------------ #
        # ROS subscribers & publishers
        # ------------------------------------------------------------------ #
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.estop_sub = self.create_subscription(
            Bool, '/emergency_stop', self.estop_callback, 10)
        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        self.left_speed_pub  = self.create_publisher(Float32, '/motor/left_speed', 10)
        self.right_speed_pub = self.create_publisher(Float32, '/motor/right_speed', 10)

        # Safety watchdog timer (10 Hz)
        self.create_timer(0.1, self.safety_check)

        self.get_logger().info(
            f'Motor controller ready — driver: {self.motor_driver}, '
            f'mode: {self.control_mode}'
        )

    # ====================================================================== #
    # Motor driver setup
    # ====================================================================== #

    def setup_motors(self):
        if self.motor_driver == 'sparkmax':
            self.setup_sparkmax()
        elif self.motor_driver == 'l298n':
            self.setup_l298n()
        elif self.motor_driver == 'sabertooth':
            self.setup_sabertooth()
        elif self.motor_driver == 'roboclaw':
            self.setup_roboclaw()
        elif self.motor_driver == 'pwm':
            self.setup_pwm()
        else:
            self.get_logger().error(f'Unknown motor driver: {self.motor_driver}')

    def setup_sparkmax(self):
        """
        Initialise four SPARK MAX controllers for skid steer over SocketCAN.

        Left  front + right front are leaders — commands sent every cycle.
        Left  rear  + right rear  are followers — configured once at startup,
        then mirror their respective leader automatically over CAN without any
        per-cycle intervention from the Jetson.
        """
        if not CAN_AVAILABLE:
            self.get_logger().error(
                'python-can not installed. Run: pip3 install python-can')
            return

        from spark_max_can_driver import SparkMaxCanDriver, IdleMode

        can_iface      = self.get_parameter('can_interface').value
        lf_id          = self.get_parameter('spark_left_front_id').value
        rf_id          = self.get_parameter('spark_right_front_id').value
        lr_id          = self.get_parameter('spark_left_rear_id').value
        rr_id          = self.get_parameter('spark_right_rear_id').value
        left_inverted  = self.get_parameter('spark_left_inverted').value
        right_inverted = self.get_parameter('spark_right_inverted').value
        rear_invert    = self.get_parameter('spark_rear_invert_follower').value

        try:
            self._can_bus = can.Bus(interface='socketcan', channel=can_iface)
        except Exception as e:
            self.get_logger().error(
                f'Failed to open CAN interface "{can_iface}": {e}\n'
                f'Run: sudo ip link set {can_iface} up type can bitrate 1000000'
            )
            return

        # Instantiate all four drivers (we need driver objects for the
        # followers too so their startup config frames can be sent)
        self._left_spark       = SparkMaxCanDriver(self._can_bus, device_id=lf_id)
        self._right_spark      = SparkMaxCanDriver(self._can_bus, device_id=rf_id)
        self._left_rear_spark  = SparkMaxCanDriver(self._can_bus, device_id=lr_id)
        self._right_rear_spark = SparkMaxCanDriver(self._can_bus, device_id=rr_id)

        # ------------------------------------------------------------------ #
        # Configure leader motors
        # ------------------------------------------------------------------ #
        self._left_spark.set_idle_mode(IdleMode.kBrake)
        self._left_spark.set_inverted(left_inverted)

        self._right_spark.set_idle_mode(IdleMode.kBrake)
        self._right_spark.set_inverted(right_inverted)

        # ------------------------------------------------------------------ #
        # Configure follower motors
        # set_follower() sends the REV follower frame which tells each rear
        # SPARK MAX to continuously mirror its leader's output over CAN.
        # After this call, no further commands need to be sent to the rears.
        # ------------------------------------------------------------------ #
        self._left_rear_spark.set_idle_mode(IdleMode.kBrake)
        self._left_rear_spark.set_follower(leader_id=lf_id, invert=rear_invert)

        self._right_rear_spark.set_idle_mode(IdleMode.kBrake)
        self._right_rear_spark.set_follower(leader_id=rf_id, invert=rear_invert)

        # Start background thread to receive telemetry from all four motors
        self._can_rx_thread = threading.Thread(
            target=self._can_receive_loop, daemon=True)
        self._can_rx_thread.start()

        self.get_logger().info(
            f'SPARK MAX skid steer ready — '
            f'L-front: {lf_id}, R-front: {rf_id}, '
            f'L-rear: {lr_id} (follower), R-rear: {rr_id} (follower), '
            f'CAN: {can_iface}'
        )

    def _can_receive_loop(self):
        """
        Background thread: pull incoming CAN frames and dispatch to all four
        SPARK MAX drivers so telemetry stays up to date.
        """
        motors = [m for m in [
            self._left_spark, self._right_spark,
            self._left_rear_spark, self._right_rear_spark
        ] if m is not None]

        while not self._shutting_down and self._can_bus is not None:
            try:
                msg = self._can_bus.recv(timeout=0.01)
                if msg is not None:
                    for motor in motors:
                        motor.handle_incoming(msg)
            except Exception as e:
                if not self._shutting_down:
                    self.get_logger().warn(f'CAN receive error: {e}')

    def setup_l298n(self):
        if not GPIO_AVAILABLE:
            self.get_logger().warn('GPIO not available — skipping L298N setup')
            return
        GPIO.setmode(GPIO.BCM)
        left_pins      = self.get_parameter('left_motor_pins').value
        right_pins     = self.get_parameter('right_motor_pins').value
        left_pwm_pin   = self.get_parameter('left_pwm_pin').value
        right_pwm_pin  = self.get_parameter('right_pwm_pin').value
        pwm_freq       = self.get_parameter('pwm_frequency').value
        for pin in left_pins + right_pins:
            GPIO.setup(pin, GPIO.OUT)
            GPIO.output(pin, GPIO.LOW)
        GPIO.setup(left_pwm_pin, GPIO.OUT)
        GPIO.setup(right_pwm_pin, GPIO.OUT)
        self.left_pwm  = GPIO.PWM(left_pwm_pin, pwm_freq)
        self.right_pwm = GPIO.PWM(right_pwm_pin, pwm_freq)
        self.left_pwm.start(0)
        self.right_pwm.start(0)
        self.get_logger().info('L298N motor driver initialized')

    def setup_sabertooth(self):
        self.get_logger().warn('Sabertooth driver not yet implemented')

    def setup_roboclaw(self):
        try:
            from roboclaw_3 import Roboclaw
            self.roboclaw = Roboclaw("/dev/ttyACM0", 115200)
            self.roboclaw.Open()
            self.roboclaw_address = 0x80
            self.get_logger().info('RoboClaw motor driver initialized')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize RoboClaw: {e}')

    def setup_pwm(self):
        pass

    # ====================================================================== #
    # ROS callbacks
    # ====================================================================== #

    def cmd_vel_callback(self, msg: Twist):
        if self.emergency_stop:
            self.stop_motors()
            return

        self.last_cmd_time = time.time()

        linear  = float(np.clip(msg.linear.x,  -self.max_speed,         self.max_speed))
        angular = float(np.clip(msg.angular.z, -self.max_angular_speed, self.max_angular_speed))

        # Differential drive kinematics
        left_speed  = linear - (angular * self.wheel_separation / 2.0)
        right_speed = linear + (angular * self.wheel_separation / 2.0)

        # Smooth with moving average
        self.left_speed_buffer.append(left_speed)
        self.right_speed_buffer.append(right_speed)
        left_speed  = float(np.mean(self.left_speed_buffer))
        right_speed = float(np.mean(self.right_speed_buffer))

        self.set_motor_speeds(left_speed, right_speed)

    def estop_callback(self, msg: Bool):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.stop_motors()
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
        else:
            self.get_logger().info('Emergency stop released')

    def joy_callback(self, msg: Joy):
        if len(msg.buttons) > 0 and msg.buttons[0] == 1:
            self.emergency_stop = True
            self.stop_motors()
            return
        if len(msg.axes) >= 4:
            twist = Twist()
            twist.linear.x  = msg.axes[1] * self.max_speed
            twist.angular.z = msg.axes[2] * self.max_angular_speed
            self.cmd_vel_callback(twist)

    # ====================================================================== #
    # Motor speed control
    # ====================================================================== #

    def set_motor_speeds(self, left_speed: float, right_speed: float):
        if self.emergency_stop:
            left_speed  = 0.0
            right_speed = 0.0

        self.current_left_speed  = left_speed
        self.current_right_speed = right_speed

        if self.motor_driver == 'sparkmax':
            self._set_sparkmax_speeds(left_speed, right_speed)
        elif self.motor_driver == 'l298n':
            self.set_l298n_speeds(left_speed, right_speed)
        elif self.motor_driver == 'roboclaw':
            self.set_roboclaw_speeds(left_speed, right_speed)
        elif self.motor_driver == 'sabertooth':
            self.set_sabertooth_speeds(left_speed, right_speed)

        self.publish_motor_status()

    def _set_sparkmax_speeds(self, left_speed: float, right_speed: float):
        """Convert m/s to [-1, 1] duty cycle and send to SPARK MAX controllers."""
        if self._left_spark is None or self._right_spark is None:
            return

        # Normalise to duty cycle
        left_power  = float(np.clip(left_speed  / self.max_speed, -1.0, 1.0))
        right_power = float(np.clip(right_speed / self.max_speed, -1.0, 1.0))

        self._left_spark.set_power(left_power)
        self._right_spark.set_power(right_power)

    def set_l298n_speeds(self, left_speed: float, right_speed: float):
        if not GPIO_AVAILABLE:
            return
        left_pins  = self.get_parameter('left_motor_pins').value
        right_pins = self.get_parameter('right_motor_pins').value

        left_duty = float(np.clip(abs(left_speed) / self.max_speed * 100.0, 0, 100))
        if left_speed > 0:
            GPIO.output(left_pins[0], GPIO.HIGH); GPIO.output(left_pins[1], GPIO.LOW)
        elif left_speed < 0:
            GPIO.output(left_pins[0], GPIO.LOW);  GPIO.output(left_pins[1], GPIO.HIGH)
        else:
            GPIO.output(left_pins[0], GPIO.LOW);  GPIO.output(left_pins[1], GPIO.LOW)
        self.left_pwm.ChangeDutyCycle(left_duty)

        right_duty = float(np.clip(abs(right_speed) / self.max_speed * 100.0, 0, 100))
        if right_speed > 0:
            GPIO.output(right_pins[0], GPIO.HIGH); GPIO.output(right_pins[1], GPIO.LOW)
        elif right_speed < 0:
            GPIO.output(right_pins[0], GPIO.LOW);  GPIO.output(right_pins[1], GPIO.HIGH)
        else:
            GPIO.output(right_pins[0], GPIO.LOW);  GPIO.output(right_pins[1], GPIO.LOW)
        self.right_pwm.ChangeDutyCycle(right_duty)

    def set_roboclaw_speeds(self, left_speed: float, right_speed: float):
        if not hasattr(self, 'roboclaw'):
            return
        counts_per_meter = 1000 / (2 * np.pi * self.wheel_radius)
        left_qpps  = int(left_speed  * counts_per_meter)
        right_qpps = int(right_speed * counts_per_meter)
        self.roboclaw.SpeedM1M2(self.roboclaw_address, left_qpps, right_qpps)

    def set_sabertooth_speeds(self, left_speed: float, right_speed: float):
        pass  # TODO: implement Sabertooth packet protocol

    def stop_motors(self):
        """Emergency stop — immediately halt all motors."""
        self.current_left_speed  = 0.0
        self.current_right_speed = 0.0
        self.left_speed_buffer.clear()
        self.right_speed_buffer.clear()

        if self.motor_driver == 'sparkmax':
            self._set_sparkmax_speeds(0.0, 0.0)
        elif self.motor_driver == 'l298n':
            self.set_l298n_speeds(0.0, 0.0)
        elif self.motor_driver == 'roboclaw':
            self.set_roboclaw_speeds(0.0, 0.0)
        elif self.motor_driver == 'sabertooth':
            self.set_sabertooth_speeds(0.0, 0.0)

        self.publish_motor_status()
        self.get_logger().warn('Motors stopped')

    # ====================================================================== #
    # Safety watchdog
    # ====================================================================== #

    def safety_check(self):
        if not self.enable_safety:
            return
        elapsed = time.time() - self.last_cmd_time
        if elapsed > self.timeout and not self.emergency_stop:
            if self.current_left_speed != 0 or self.current_right_speed != 0:
                self.get_logger().warn(
                    f'Command timeout ({elapsed:.1f}s) — stopping motors')
                self.stop_motors()

    # ====================================================================== #
    # Publishers
    # ====================================================================== #

    def publish_motor_status(self):
        if self._shutting_down:
            return
        left_msg = Float32()
        left_msg.data = float(self.current_left_speed)
        self.left_speed_pub.publish(left_msg)

        right_msg = Float32()
        right_msg.data = float(self.current_right_speed)
        self.right_speed_pub.publish(right_msg)

    # ====================================================================== #
    # Shutdown
    # ====================================================================== #

    def shutdown(self):
        self.get_logger().info('Shutting down motor controller')
        self._shutting_down = True
        self.stop_motors()

        # Close all SPARK MAX drivers
        for motor in [self._left_spark, self._right_spark,
                      self._left_rear_spark, self._right_rear_spark]:
            if motor is not None:
                motor.close()
        if self._can_bus is not None:
            self._can_bus.shutdown()

        # Clean up GPIO if using L298N
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
