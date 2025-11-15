"""
Sensor Manager Node
Manages all sensors on the rover and publishes their data
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix, Temperature, Range
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class SensorManager(Node):
    def __init__(self):
        super().__init__('sensor_manager')
        
        # Publishers for sensor data
        self.camera_front_pub = self.create_publisher(Image, 'camera/front/image_raw', 10)
        self.camera_rear_pub = self.create_publisher(Image, 'camera/rear/image_raw', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.temp_pub = self.create_publisher(Temperature, 'temperature', 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.battery_pub = self.create_publisher(Float32, 'battery_voltage', 10)
        
        # Ultrasonic/lidar sensors
        self.range_front_pub = self.create_publisher(Range, 'range/front', 10)
        self.range_rear_pub = self.create_publisher(Range, 'range/rear', 10)
        
        # Timer for publishing sensor data
        self.imu_timer = self.create_timer(0.02, self.publish_imu)  # 50 Hz
        self.sensor_timer = self.create_timer(0.05, self.publish_sensors)  # 20 Hz
        
        self.get_logger().info('Sensor Manager initialized')
    
    def publish_imu(self):
        """Publish IMU data at high frequency"""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = 'imu_link'
        
        # TODO: Populate with actual sensor data
        # imu_msg.orientation.x = ...
        # imu_msg.angular_velocity.x = ...
        # imu_msg.linear_acceleration.x = ...
        
        self.imu_pub.publish(imu_msg)
    
    def publish_sensors(self):
        """Publish other sensor data"""
        timestamp = self.get_clock().now().to_msg()
        
        # GPS
        gps_msg = NavSatFix()
        gps_msg.header.stamp = timestamp
        gps_msg.header.frame_id = 'gps_link'
        # TODO: Populate with actual GPS data
        self.gps_pub.publish(gps_msg)
        
        # Temperature
        temp_msg = Temperature()
        temp_msg.header.stamp = timestamp
        # TODO: Populate with actual temperature sensor
        self.temp_pub.publish(temp_msg)
        
        # Battery voltage
        battery_msg = Float32()
        # TODO: Read actual battery voltage
        # battery_msg.data = read_battery_voltage()
        self.battery_pub.publish(battery_msg)
        
        # Range sensors
        range_msg = Range()
        range_msg.header.stamp = timestamp
        range_msg.radiation_type = Range.ULTRASOUND
        range_msg.min_range = 0.02
        range_msg.max_range = 4.0
        # TODO: Read actual range sensors
        self.range_front_pub.publish(range_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SensorManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
