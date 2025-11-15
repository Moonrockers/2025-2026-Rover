"""
Keyboard Teleoperation Node
Allows manual control of the rover using keyboard inputs
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Control parameters
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 1.0  # rad/s
        self.speed_increment = 0.1
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.publish_cmd)
        
        # Current velocities
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_instructions()
        self.get_logger().info('Keyboard Teleop initialized')
    
    def print_instructions(self):
        msg = """
        Rover Keyboard Teleoperation
        ----------------------------------
        Moving:
            w/s : forward/backward
            a/d : turn left/right
            x   : stop
        
        Speed Control:
            q/z : increase/decrease max speeds by 10%
        
        CTRL-C to quit
        """
        print(msg)
    
    def get_key(self):
        """Get keyboard input"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def publish_cmd(self):
        """Publish velocity commands"""
        try:
            key = self.get_key()
            
            if key == 'w':
                self.linear_vel = self.linear_speed
                self.angular_vel = 0.0
            elif key == 's':
                self.linear_vel = -self.linear_speed
                self.angular_vel = 0.0
            elif key == 'a':
                self.linear_vel = 0.0
                self.angular_vel = self.angular_speed
            elif key == 'd':
                self.linear_vel = 0.0
                self.angular_vel = -self.angular_speed
            elif key == 'x':
                self.linear_vel = 0.0
                self.angular_vel = 0.0
            elif key == 'q':
                self.linear_speed += self.speed_increment
                self.angular_speed += self.speed_increment
                self.get_logger().info(
                    f'Speed increased: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}'
                )
            elif key == 'z':
                self.linear_speed = max(0.1, self.linear_speed - self.speed_increment)
                self.angular_speed = max(0.1, self.angular_speed - self.speed_increment)
                self.get_logger().info(
                    f'Speed decreased: linear={self.linear_speed:.2f}, angular={self.angular_speed:.2f}'
                )
            elif key == '\x03':  # CTRL-C
                raise KeyboardInterrupt
            
            # Publish command
            twist = Twist()
            twist.linear.x = self.linear_vel
            twist.angular.z = self.angular_vel
            self.cmd_vel_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
