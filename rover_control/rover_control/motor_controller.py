"""
Motor Controller Node
Converts velocity commands to motor speeds for differential drive rover
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray, Bool
import time


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Subscribe to velocity commands from teleop
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Subscribe to emergency stop
        self.estop_sub = self.create_subscription(
            Bool,
            'emergency_stop',
            self.estop_callback,
            10
        )
        
        # Publish motor speeds to hardware interface
        self.motor_pub = self.create_publisher(
            Float64MultiArray,
            'motor_speeds',
            10
        )
        
        # Rover parameters
        self.wheel_base = 0.5  # meters between left and right wheels
        self.wheel_radius = 0.1  # meters
        self.max_speed = 2.0  # max m/s
        
        # Safety parameters
        self.emergency_stop = False
        self.last_cmd_time = time.time()
        self.cmd_timeout = 1.0  # stop if no command for 1 second
        
        # Watchdog timer
        self.timer = self.create_timer(0.1, self.watchdog)
        
        self.get_logger().info('Motor Controller initialized')
    
    def cmd_vel_callback(self, msg):
        """Convert twist commands to individual wheel speeds"""
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop active - ignoring commands')
            return
        
        self.last_cmd_time = time.time()
        
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Limit maximum speed
        linear_vel = max(-self.max_speed, min(self.max_speed, linear_vel))
        
        # Differential drive kinematics
        left_vel = (linear_vel - angular_vel * self.wheel_base / 2.0) / self.wheel_radius
        right_vel = (linear_vel + angular_vel * self.wheel_base / 2.0) / self.wheel_radius
        
        # Publish motor speeds (assuming 4-wheel drive)
        motor_speeds = Float64MultiArray()
        motor_speeds.data = [left_vel, right_vel, left_vel, right_vel]
        self.motor_pub.publish(motor_speeds)
        
        self.get_logger().debug(f'Motor speeds: L={left_vel:.2f}, R={right_vel:.2f}')
    
    def estop_callback(self, msg):
        """Handle emergency stop"""
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.stop_motors()
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
        else:
            self.get_logger().info('Emergency stop released')
    
    def watchdog(self):
        """Stop motors if no command received recently"""
        if time.time() - self.last_cmd_time > self.cmd_timeout:
            self.stop_motors()
    
    def stop_motors(self):
        """Send stop command to all motors"""
        motor_speeds = Float64MultiArray()
        motor_speeds.data = [0.0, 0.0, 0.0, 0.0]
        self.motor_pub.publish(motor_speeds)


def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
