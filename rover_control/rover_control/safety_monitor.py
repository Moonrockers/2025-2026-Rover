"""
Safety Monitor Node
Monitors critical parameters and triggers emergency stop if needed
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature
from std_msgs.msg import Float32, Bool


class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')
        
        # Subscribe to critical sensors
        self.battery_sub = self.create_subscription(
            Float32, 'battery_voltage', self.battery_callback, 10
        )
        self.temp_sub = self.create_subscription(
            Temperature, 'temperature', self.temp_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        
        # Emergency stop publisher
        self.estop_pub = self.create_publisher(Bool, 'emergency_stop', 10)
        
        # Safety thresholds
        self.min_battery_voltage = 10.5  # Volts
        self.max_temperature = 85.0  # Celsius
        self.max_tilt_angle = 45.0  # Degrees
        
        self.current_battery = 12.0
        self.current_temp = 25.0
        
        self.get_logger().info('Safety Monitor initialized')
    
    def battery_callback(self, msg):
        self.current_battery = msg.data
        if msg.data < self.min_battery_voltage:
            self.get_logger().error(f'CRITICAL: Battery voltage low: {msg.data:.2f}V')
            self.trigger_estop('Low battery')
    
    def temp_callback(self, msg):
        self.current_temp = msg.temperature
        if msg.temperature > self.max_temperature:
            self.get_logger().error(f'CRITICAL: Temperature high: {msg.temperature:.1f}C')
            self.trigger_estop('High temperature')
    
    def imu_callback(self, msg):
        # Check for excessive tilt
        # TODO: Convert quaternion to euler and check tilt angle
        pass
    
    def trigger_estop(self, reason):
        """Trigger emergency stop"""
        estop_msg = Bool()
        estop_msg.data = True
        self.estop_pub.publish(estop_msg)
        self.get_logger().error(f'EMERGENCY STOP: {reason}')


def main(args=None):
    rclpy.init(args=args)
    node = SafetyMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
