"""
Motor Controller - Waveshare USB-CAN-A direct serial edition
Talks to the Waveshare adapter via /dev/ttyUSB* using its proprietary
serial protocol. No SocketCAN required.

Drives 6 SPARK MAX controllers in CAN mode over the daisy-chained bus.

REV SPARK MAX device IDs:
    1 = left front  (drive)
    4 = right front (drive)
    2 = left rear   (drive follower of left front)
    3 = right rear  (drive follower of right front)
    5 = conveyor    (excavation)
    6 = hopper      (deposition)

Waveshare USB-CAN-A serial frame format (variable-length CAN frame):
    aa <type> <id_low> <id_high> [<id_byte3> <id_byte4>] <data...> 55

For extended CAN frames (29-bit IDs which SPARK MAX uses):
    Type byte = 0xC0 | DLC      (extended data frame, DLC bytes of data)
    e.g. DLC=4 -> type = 0xC4
    Header: aa <type> <id 4 bytes little-endian> <data DLC bytes> 55
"""

import os
import time
import struct
import threading
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32, String
import numpy as np
from collections import deque


# REV SPARK MAX CAN protocol constants
REV_DEVICE_TYPE  = 0x02    # Motor controller
REV_MFR          = 0x05    # REV Robotics
API_DUTY_CYCLE   = 0x002   # Duty cycle control
API_FOLLOWER     = 0x062   # Follower configuration
API_IDLE_MODE    = 0x054   # Brake/Coast


def build_can_id(api_id: int, device_id: int) -> int:
    """
    Build a 29-bit REV SPARK MAX CAN arbitration ID.
    Layout (29 bits):
      [28:24] device type (5 bits)
      [23:16] manufacturer (8 bits)
      [15:6 ] API ID (10 bits)
      [5:0  ] device ID (6 bits)
    """
    return (((REV_DEVICE_TYPE & 0x1F) << 24) |
            ((REV_MFR          & 0xFF) << 16) |
            ((api_id           & 0x3FF) << 6) |
            (device_id         & 0x3F))


class WaveshareCanBus:
    """
    Minimal Waveshare USB-CAN-A driver: opens /dev/ttyUSB*, configures
    the adapter for 1 Mbps, and provides a send() method.
    """

    def __init__(self, device='/dev/ttyUSB0', baudrate=2_000_000):
        self.dev = device
        self.ser = serial.Serial(device, baudrate, timeout=0.1)
        self._lock = threading.Lock()
        self._configure()

    def _configure(self):
        """
        Send the Waveshare init/settings frame.
        Format (per Waveshare wiki USB-CAN-A):
          aa 55 12 <speed> <frame_type> <filter_id 4B> <mask_id 4B> 
          <mode> <retransmit> 00 00 00 00 <checksum>
        speed code 0x01 = 1 Mbps
        frame_type 0x01 = extended frame
        mode 0x00 = normal
        """
        # Init frame for 1 Mbps, extended, normal mode
        # This matches the bytes we saw in the wireshark trace earlier
        frame = bytearray([
            0xAA, 0x55,
            0x12,       # config command
            0x01,       # speed: 1 Mbps
            0x01,       # frame type: extended
            0x00, 0x00, 0x00, 0x00,   # filter id (no filter)
            0x00, 0x00, 0x00, 0x00,   # mask id (no mask)
            0x00,       # mode: normal
            0x01,       # auto-retransmit on
            0x00, 0x00, 0x00, 0x00,   # reserved
        ])
        checksum = sum(frame[2:]) & 0xFF
        frame.append(checksum)
        with self._lock:
            self.ser.write(bytes(frame))
            self.ser.flush()
        time.sleep(0.2)

    def send_extended(self, arb_id: int, data: bytes):
        """
        Send a single extended (29-bit) CAN frame.
        Frame format:
          aa <type> <id_LE_4B> <data DLC bytes> 55
        type byte: 1100_xxxx where xxxx = DLC
          bit 7..6 = 11 (extended data frame in CAN 2.0B)
          bit 5..4 = 00
          bit 3..0 = DLC
        Actually per the Waveshare protocol the type byte encodes
        frame-format and DLC together; for extended data frames it is
        0xC0 | DLC.
        """
        if len(data) > 8:
            data = data[:8]
        dlc = len(data)
        type_byte = 0xC0 | (dlc & 0x0F)

        frame = bytearray()
        frame.append(0xAA)
        frame.append(type_byte)
        frame.extend(struct.pack('<I', arb_id & 0x1FFFFFFF))  # 4 bytes LE
        frame.extend(data)
        frame.append(0x55)

        with self._lock:
            self.ser.write(bytes(frame))
            self.ser.flush()

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


class SparkMaxDriver:
    """Per-device helper to build and send SPARK MAX command frames."""

    def __init__(self, bus: WaveshareCanBus, device_id: int):
        self.bus = bus
        self.device_id = device_id
        self.inverted = False

    def set_inverted(self, inv: bool):
        self.inverted = bool(inv)

    def set_power(self, duty: float):
        """duty in [-1.0, 1.0]"""
        duty = max(-1.0, min(1.0, float(duty)))
        if self.inverted:
            duty = -duty
        arb = build_can_id(API_DUTY_CYCLE, self.device_id)
        data = struct.pack('<f', duty)  # 4-byte little-endian float
        # SPARK MAX expects 8 bytes; pad with zeros
        data = data + b'\x00\x00\x00\x00'
        self.bus.send_extended(arb, data)

    def set_follower(self, leader_id: int, invert: bool = False):
        """Configure this controller to follow another."""
        # Follower payload: leader's full 32-bit arb ID + invert byte
        leader_arb = build_can_id(0, leader_id)
        data = struct.pack('<I', leader_arb)
        data += bytes([0x01 if invert else 0x00])
        data += b'\x00\x00\x00'
        arb = build_can_id(API_FOLLOWER, self.device_id)
        self.bus.send_extended(arb, data)


class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')

        # Parameters
        self.declare_parameter('wheel_separation',  0.5)
        self.declare_parameter('max_speed',         1.0)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('enable_safety',     True)
        self.declare_parameter('timeout',           1.0)
        self.declare_parameter('serial_device',     '/dev/ttyUSB0')

        # Motor IDs (match the rover wiring)
        self.declare_parameter('lf_id', 1)
        self.declare_parameter('rf_id', 4)
        self.declare_parameter('lr_id', 2)
        self.declare_parameter('rr_id', 3)
        self.declare_parameter('conveyor_id', 5)
        self.declare_parameter('hopper_id',   6)

        self.declare_parameter('left_inverted',     False)
        self.declare_parameter('right_inverted',    True)
        self.declare_parameter('conveyor_speed',     0.6)
        self.declare_parameter('hopper_dump_speed',  0.5)
        self.declare_parameter('hopper_reset_speed', -0.3)

        # Read params
        self.wheel_separation   = self.get_parameter('wheel_separation').value
        self.max_speed          = self.get_parameter('max_speed').value
        self.max_angular_speed  = self.get_parameter('max_angular_speed').value
        self.enable_safety      = self.get_parameter('enable_safety').value
        self.timeout            = self.get_parameter('timeout').value
        self.conveyor_speed     = self.get_parameter('conveyor_speed').value
        self.hopper_dump_speed  = self.get_parameter('hopper_dump_speed').value
        self.hopper_reset_speed = self.get_parameter('hopper_reset_speed').value

        # State
        self.emergency_stop      = False
        self.last_cmd_time       = time.time()
        self.current_left_speed  = 0.0
        self.current_right_speed = 0.0
        self.left_speed_buffer   = deque(maxlen=5)
        self.right_speed_buffer  = deque(maxlen=5)

        # Auto-detect the Waveshare port - try ttyUSB0 then ttyUSB1
        dev = self.get_parameter('serial_device').value
        if not os.path.exists(dev):
            for alt in ('/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2'):
                if os.path.exists(alt):
                    dev = alt
                    break

        self.get_logger().info(f'Opening Waveshare USB-CAN at {dev}')
        try:
            self._bus = WaveshareCanBus(device=dev)
        except Exception as e:
            self.get_logger().fatal(f'Failed to open {dev}: {e}')
            raise

        # SPARK MAX drivers
        lf = self.get_parameter('lf_id').value
        rf = self.get_parameter('rf_id').value
        lr = self.get_parameter('lr_id').value
        rr = self.get_parameter('rr_id').value
        conv = self.get_parameter('conveyor_id').value
        hop  = self.get_parameter('hopper_id').value

        self._left  = SparkMaxDriver(self._bus, lf)
        self._right = SparkMaxDriver(self._bus, rf)
        self._left_rear  = SparkMaxDriver(self._bus, lr)
        self._right_rear = SparkMaxDriver(self._bus, rr)
        self._conveyor   = SparkMaxDriver(self._bus, conv)
        self._hopper     = SparkMaxDriver(self._bus, hop)

        self._left.set_inverted(self.get_parameter('left_inverted').value)
        self._right.set_inverted(self.get_parameter('right_inverted').value)

        # Configure followers
        time.sleep(0.1)
        self._left_rear.set_follower(lf)
        self._right_rear.set_follower(rf)
        time.sleep(0.1)

        # ROS subs/pubs
        self.cmd_vel_sub     = self.create_subscription(Twist,  '/cmd_vel',            self.cmd_vel_callback,     10)
        self.estop_sub       = self.create_subscription(Bool,   '/emergency_stop',     self.estop_callback,       10)
        self.dig_cmd_sub     = self.create_subscription(String, '/digging/command',    self.dig_cmd_callback,     10)
        self.deposit_cmd_sub = self.create_subscription(String, '/deposition/command', self.deposit_cmd_callback, 10)

        self.left_speed_pub  = self.create_publisher(Float32, '/motor/left_speed',  10)
        self.right_speed_pub = self.create_publisher(Float32, '/motor/right_speed', 10)

        # Keep-alive at 20 Hz: SPARK MAX requires regular commands
        self._keep_alive_left  = 0.0
        self._keep_alive_right = 0.0
        self._keep_alive_conv  = 0.0
        self._keep_alive_hop   = 0.0
        self.create_timer(0.05, self._keep_alive_tick)
        self.create_timer(0.1,  self.safety_check)

        self.get_logger().info('Waveshare motor controller ready')

    # ─── Drive ────────────────────────────────────────────────────────────────
    def cmd_vel_callback(self, msg: Twist):
        if self.emergency_stop:
            self.stop_all_motors()
            return
        self.last_cmd_time = time.time()

        linear  = float(np.clip(msg.linear.x,  -self.max_speed,        self.max_speed))
        angular = float(np.clip(msg.angular.z, -self.max_angular_speed, self.max_angular_speed))

        left_speed  = linear - (angular * self.wheel_separation / 2.0)
        right_speed = linear + (angular * self.wheel_separation / 2.0)

        self.left_speed_buffer.append(left_speed)
        self.right_speed_buffer.append(right_speed)
        left_speed  = float(np.mean(self.left_speed_buffer))
        right_speed = float(np.mean(self.right_speed_buffer))

        self.set_drive_speeds(left_speed, right_speed)

    def set_drive_speeds(self, left_speed: float, right_speed: float):
        if self.emergency_stop:
            left_speed = right_speed = 0.0

        self.current_left_speed  = left_speed
        self.current_right_speed = right_speed

        left_power  = float(np.clip(left_speed  / self.max_speed, -1.0, 1.0))
        right_power = float(np.clip(right_speed / self.max_speed, -1.0, 1.0))

        self._keep_alive_left  = left_power
        self._keep_alive_right = right_power

        self._left.set_power(left_power)
        self._right.set_power(right_power)

        self.publish_motor_status()

    # ─── Conveyor ─────────────────────────────────────────────────────────────
    def dig_cmd_callback(self, msg: String):
        if self.emergency_stop:
            return
        cmd = msg.data.lower()
        if cmd in ('start', 'lower'):
            self._keep_alive_conv = self.conveyor_speed
            self._conveyor.set_power(self.conveyor_speed)
            self.get_logger().info(f'Conveyor ON {self.conveyor_speed:+.2f}')
        elif cmd == 'raise':
            self._keep_alive_conv = -self.conveyor_speed
            self._conveyor.set_power(-self.conveyor_speed)
            self.get_logger().info(f'Conveyor REVERSE {-self.conveyor_speed:+.2f}')
        elif cmd in ('stop', 'reset'):
            self._keep_alive_conv = 0.0
            self._conveyor.set_power(0.0)
            self.get_logger().info('Conveyor STOPPED')

    # ─── Hopper ───────────────────────────────────────────────────────────────
    def deposit_cmd_callback(self, msg: String):
        if self.emergency_stop:
            return
        cmd = msg.data.lower()
        if cmd == 'dump':
            self._keep_alive_hop = self.hopper_dump_speed
            self._hopper.set_power(self.hopper_dump_speed)
            self.get_logger().info(f'Hopper DUMPING {self.hopper_dump_speed:+.2f}')
        elif cmd == 'reset':
            self._keep_alive_hop = self.hopper_reset_speed
            self._hopper.set_power(self.hopper_reset_speed)
            self.get_logger().info(f'Hopper RESETTING {self.hopper_reset_speed:+.2f}')
        elif cmd == 'stop':
            self._keep_alive_hop = 0.0
            self._hopper.set_power(0.0)
            self.get_logger().info('Hopper STOPPED')

    # ─── Safety & keep-alive ──────────────────────────────────────────────────
    def estop_callback(self, msg: Bool):
        self.emergency_stop = msg.data
        if self.emergency_stop:
            self.stop_all_motors()
            self.get_logger().error('EMERGENCY STOP ACTIVATED')
        else:
            self.get_logger().info('Emergency stop released')

    def stop_all_motors(self):
        self.current_left_speed = self.current_right_speed = 0.0
        self._keep_alive_left = self._keep_alive_right = 0.0
        self._keep_alive_conv = self._keep_alive_hop = 0.0
        self.left_speed_buffer.clear()
        self.right_speed_buffer.clear()
        self._left.set_power(0.0)
        self._right.set_power(0.0)
        self._conveyor.set_power(0.0)
        self._hopper.set_power(0.0)
        self.publish_motor_status()

    def safety_check(self):
        if not self.enable_safety:
            return
        if (time.time() - self.last_cmd_time) > self.timeout and not self.emergency_stop:
            if self.current_left_speed != 0 or self.current_right_speed != 0:
                self.get_logger().warn('Drive timeout - stopping')
                self._keep_alive_left = self._keep_alive_right = 0.0
                self._left.set_power(0.0)
                self._right.set_power(0.0)
                self.current_left_speed = self.current_right_speed = 0.0
                self.publish_motor_status()

    def _keep_alive_tick(self):
        """SPARK MAX needs a duty cycle frame at least every 50ms or it
        will time out and disable the motor output. Resend last setpoints."""
        try:
            self._left.set_power(self._keep_alive_left)
            self._right.set_power(self._keep_alive_right)
            self._conveyor.set_power(self._keep_alive_conv)
            self._hopper.set_power(self._keep_alive_hop)
        except Exception as e:
            self.get_logger().warn(f'Keep-alive write failed: {e}')

    def publish_motor_status(self):
        l = Float32(); l.data = float(self.current_left_speed)
        r = Float32(); r.data = float(self.current_right_speed)
        self.left_speed_pub.publish(l)
        self.right_speed_pub.publish(r)

    def shutdown(self):
        try:
            self.stop_all_motors()
            time.sleep(0.1)
            self._bus.close()
        except Exception:
            pass


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
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
