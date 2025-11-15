"""
Telemetry Node
Collects and consolidates all telemetry data for transmission to ground station
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, Temperature
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Float32, Float64MultiArray
import json


class Telemetry(Node):
    def __init__(self):
        super().__init__('telemetry')
        
        # Subscribe to all sensor topics
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, 'gps/fix', self.gps_callback, 10)
        self.temp_sub = self.create_subscription(Temperature, 'temperature', self.temp_callback, 10)
        self.battery_sub = self.create_subscription(
            Float32, 'battery_voltage', self.battery_callback, 10
        )
        self.motor_sub = self.create_subscription(
            Float64MultiArray, 'motor_speeds', self.motor_callback, 10
        )
        
        # Publisher for consolidated telemetry
        self.telemetry_pub = self.create_publisher(String, 'telemetry', 10)
        
        # Store latest data
        self.telemetry_data = {
            'timestamp': None,
            'position': {'x': 0, 'y': 0, 'z': 0},
            'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0},
            'gps': {'lat': 0, 'lon': 0, 'alt': 0},
            'temperature': 0,
            'battery_voltage': 0,
            'motor_speeds': [0, 0, 0, 0],
            'status': 'OK'
        }
        
        # Timer to publish telemetry (2 Hz)
        self.timer = self.create_timer(0.5, self.publish_telemetry)
        
        self.get_logger().info('Telemetry initialized')
    
    def odom_callback(self, msg):
        self.telemetry_data['position'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
    
    def imu_callback(self, msg):
        self.telemetry_data['orientation'] = {
            'x': msg.orientation.x,
            'y': msg.orientation.y,
            'z': msg.orientation.z,
            'w': msg.orientation.w
        }
    
    def gps_callback(self, msg):
        self.telemetry_data['gps'] = {
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude
        }
    
    def temp_callback(self, msg):
        self.telemetry_data['temperature'] = msg.temperature
    
    def battery_callback(self, msg):
        self.telemetry_data['battery_voltage'] = msg.data
    
    def motor_callback(self, msg):
        self.telemetry_data['motor_speeds'] = list(msg.data)
    
    def publish_telemetry(self):
        """Publish consolidated telemetry"""
        self.telemetry_data['timestamp'] = self.get_clock().now().to_msg().sec
        
        telemetry_msg = String()
        telemetry_msg.data = json.dumps(self.telemetry_data)
        self.telemetry_pub.publish(telemetry_msg)
        
        self.get_logger().debug('Telemetry published')


def main(args=None):
    rclpy.init(args=args)
    node = Telemetry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
