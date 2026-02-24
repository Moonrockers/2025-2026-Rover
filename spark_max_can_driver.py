"""
spark_max_can_driver.py

Python SPARK MAX driver using SocketCAN (python-can) for the Jetson Nano.
Translates the same interface as the C++ SparkMax subclass into native Linux
SocketCAN calls via python-can.

Hardware setup:
    Jetson Nano SPI pins → MCP2515 module → CAN bus → SPARK MAX controllers
    (or any USB-CAN adapter that exposes a SocketCAN interface)

Bring up the CAN interface before running:
    sudo modprobe mcp251x          # if using MCP2515 SPI
    sudo ip link set can0 up type can bitrate 1000000
    sudo ip link set can0 txqueuelen 1000

Install dependencies:
    pip3 install python-can

REV SPARK MAX CAN ID (29-bit extended):
    Bits [28:24] = 0x02  Device Type  (Motor Controller)
    Bits [23:16] = 0x05  Manufacturer (REV Robotics)
    Bits [15:6]  = API Class + Index
    Bits [5:0]   = Device ID (0-62)
"""

import can
import struct
import threading
import time
import logging
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Optional

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# REV SPARK MAX CAN constants
# ---------------------------------------------------------------------------

_DEV_TYPE    = 0x02  # Motor Controller
_MFR_REV     = 0x05  # REV Robotics

# Outbound API IDs (bits [15:6] of the 29-bit CAN ID)
class _Api(IntEnum):
    CMD_DUTY_CYCLE      = 0x002
    CMD_SPEED           = 0x012
    CMD_POSITION        = 0x032
    CMD_FOLLOWER        = 0x062

    CFG_IDLE_MODE       = 0x054
    CFG_MOTOR           = 0x056   # inverted flag
    CFG_PID_P           = 0x058
    CFG_PID_I           = 0x05A
    CFG_PID_D           = 0x05C
    CFG_PID_F           = 0x05E
    CFG_PID_IZONE       = 0x060
    CFG_OUTPUT_MIN      = 0x064
    CFG_OUTPUT_MAX      = 0x066
    CFG_SOFT_LIMIT_FWD  = 0x068
    CFG_SOFT_LIMIT_REV  = 0x06A
    CFG_SOFT_LIMIT_EN   = 0x06C
    CFG_BURN_FLASH      = 0x072

    CMD_CLEAR_FAULTS    = 0x06E

    # Incoming periodic status frame IDs
    STATUS_0            = 0x060   # Applied output, faults        (10 ms)
    STATUS_1            = 0x061   # Velocity, temp, voltage, cur  (20 ms)
    STATUS_2            = 0x062   # Position                      (20 ms)


class ControlMode(IntEnum):
    kDutyCycle = 0
    kVelocity  = 1
    kPosition  = 3
    kFollower  = 5


class IdleMode(IntEnum):
    kCoast = 0
    kBrake = 1


class Fault(IntEnum):
    Brownout        = (1 << 0)
    Overcurrent     = (1 << 1)
    IWDTReset       = (1 << 2)
    MotorFault      = (1 << 3)
    SensorFault     = (1 << 4)
    Stall           = (1 << 5)
    EEPROMCRC       = (1 << 6)
    CANTx           = (1 << 7)
    CANRx           = (1 << 8)
    HasReset        = (1 << 9)
    DRVFault        = (1 << 10)
    OtherFault      = (1 << 11)
    SoftLimitFwd    = (1 << 12)
    SoftLimitRev    = (1 << 13)
    HardLimitFwd    = (1 << 14)
    HardLimitRev    = (1 << 15)


# ---------------------------------------------------------------------------
# Telemetry data containers
# ---------------------------------------------------------------------------

@dataclass
class SparkMaxStatus:
    # Status 0
    applied_output:  float   = 0.0
    faults:          int     = 0
    sticky_faults:   int     = 0
    is_inverted:     bool    = False
    motor_running:   bool    = False
    # Status 1
    velocity_rpm:    float   = 0.0   # RPM
    temperature_c:   float   = 0.0   # °C
    bus_voltage:     float   = 0.0   # V
    current_amps:    float   = 0.0   # A
    # Status 2
    position_rotations: float = 0.0  # rotations


# ---------------------------------------------------------------------------
# CAN ID helpers
# ---------------------------------------------------------------------------

def _build_can_id(api_id: int, device_id: int) -> int:
    """
    Pack a 29-bit extended SPARK MAX CAN arbitration ID.

        [28:24] Device type  (0x02)
        [23:16] Manufacturer (0x05)
        [15:6]  API class + index
        [5:0]   Device ID
    """
    return ((_DEV_TYPE & 0x1F) << 24) | ((_MFR_REV & 0xFF) << 16) | ((api_id & 0x3FF) << 6) | (device_id & 0x3F)


def _extract_api_id(can_id: int) -> int:
    """Extract API class+index from a received 29-bit CAN ID."""
    return (can_id >> 6) & 0x3FF


def _extract_device_id(can_id: int) -> int:
    """Extract device ID from a received 29-bit CAN ID."""
    return can_id & 0x3F


def _extract_manufacturer(can_id: int) -> int:
    return (can_id >> 16) & 0xFF


# ---------------------------------------------------------------------------
# SparkMaxCanDriver
# ---------------------------------------------------------------------------

class SparkMaxCanDriver:
    """
    Python driver for a single REV SPARK MAX over SocketCAN.

    Usage:
        bus = can.Bus(interface='socketcan', channel='can0')
        motor = SparkMaxCanDriver(bus, device_id=1)
        motor.set_idle_mode(IdleMode.kBrake)
        motor.set_power(0.5)   # 50% duty cycle forward
        ...
        motor.set_power(0.0)
    """

    # Keep-alive interval in seconds (SPARK MAX faults if no frame for ~1 s)
    KEEP_ALIVE_INTERVAL = 0.025  # 25 ms

    def __init__(self, bus: can.BusABC, device_id: int):
        """
        Args:
            bus:        An open python-can Bus instance (shared across motors).
            device_id:  SPARK MAX CAN device ID (0-62).
        """
        if not (0 <= device_id <= 62):
            raise ValueError(f"Device ID must be 0-62, got {device_id}")

        self._bus        = bus
        self._device_id  = device_id
        self._ctrl_mode  = ControlMode.kDutyCycle
        self._last_setpoint: float = 0.0
        self._inverted   = False
        self._status     = SparkMaxStatus()
        self._status_lock = threading.Lock()

        # Keep-alive periodic task (uses SocketCAN BCM via python-can)
        self._keep_alive_task: Optional[can.CyclicSendTaskABC] = None
        self._start_keep_alive()

        logger.info(f"SparkMaxCanDriver: device_id={device_id} initialized")

    # -----------------------------------------------------------------------
    # Core control interface (matches FrcMotorController API)
    # -----------------------------------------------------------------------

    def set_power(self, power: float) -> None:
        """Set duty cycle output. power ∈ [-1.0, 1.0]."""
        power = max(-1.0, min(1.0, power))
        self._ctrl_mode = ControlMode.kDutyCycle
        self._last_setpoint = power
        self._send_float(int(_Api.CMD_DUTY_CYCLE), power)
        self._update_keep_alive()

    def set_inverted(self, invert: bool = True) -> None:
        """Invert motor direction."""
        self._inverted = invert
        buf = bytearray(8)
        buf[3] = 0x01 if invert else 0x00
        self._send_raw(int(_Api.CFG_MOTOR), buf)

    def set_idle_mode(self, mode: IdleMode) -> None:
        """Set coast or brake idle mode."""
        buf = bytearray(8)
        buf[0] = int(mode)
        self._send_raw(int(_Api.CFG_IDLE_MODE), buf, length=1)

    def get_applied_output(self) -> float:
        """Return last received applied output [-1, 1]."""
        with self._status_lock:
            return self._status.applied_output

    def clear_faults(self) -> None:
        buf = bytearray(8)
        self._send_raw(int(_Api.CMD_CLEAR_FAULTS), buf, length=1)

    # -----------------------------------------------------------------------
    # Extended control
    # -----------------------------------------------------------------------

    def set_control_mode(self, mode: ControlMode) -> None:
        self._ctrl_mode = mode

    def set_reference(self, value: float) -> None:
        """
        Set control reference for the current control mode.
            kDutyCycle → [-1.0, 1.0]
            kVelocity  → RPM
            kPosition  → rotations
            kFollower  → use set_follower() instead
        """
        if self._ctrl_mode == ControlMode.kDutyCycle:
            value = max(-1.0, min(1.0, value))
            api_id = int(_Api.CMD_DUTY_CYCLE)
        elif self._ctrl_mode == ControlMode.kVelocity:
            api_id = int(_Api.CMD_SPEED)
        elif self._ctrl_mode == ControlMode.kPosition:
            api_id = int(_Api.CMD_POSITION)
        elif self._ctrl_mode == ControlMode.kFollower:
            logger.warning("Use set_follower() for follower mode, ignoring set_reference()")
            return
        else:
            api_id = int(_Api.CMD_DUTY_CYCLE)

        self._last_setpoint = value
        self._send_float(api_id, value)
        self._update_keep_alive()

    def set_follower(self, leader_id: int, invert: bool = False) -> None:
        """
        Configure follower mode.

        Args:
            leader_id:  CAN device ID of the leader SPARK MAX.
            invert:     Invert output relative to leader.
        """
        self._ctrl_mode = ControlMode.kFollower

        # Build the leader's full 29-bit CAN device ID
        leader_full_id = _build_can_id(0, leader_id)  # API=0 for device address

        buf = bytearray(8)
        buf[0] = (leader_full_id)       & 0xFF
        buf[1] = (leader_full_id >> 8)  & 0xFF
        buf[2] = (leader_full_id >> 16) & 0xFF
        buf[3] = (leader_full_id >> 24) & 0xFF
        buf[4] = 0x40 if invert else 0x00  # bit 6 = invert

        self._send_raw(int(_Api.CMD_FOLLOWER), buf)

    # -----------------------------------------------------------------------
    # PID configuration
    # -----------------------------------------------------------------------

    def set_pid(self, kp: float, ki: float, kd: float, kf: float = 0.0, slot: int = 0) -> None:
        offset = slot * 0x10
        self._send_float(int(_Api.CFG_PID_P) + offset, kp)
        self._send_float(int(_Api.CFG_PID_I) + offset, ki)
        self._send_float(int(_Api.CFG_PID_D) + offset, kd)
        self._send_float(int(_Api.CFG_PID_F) + offset, kf)

    def set_izone(self, izone: float, slot: int = 0) -> None:
        offset = slot * 0x10
        self._send_float(int(_Api.CFG_PID_IZONE) + offset, izone)

    def set_output_range(self, min_out: float, max_out: float, slot: int = 0) -> None:
        offset = slot * 0x10
        self._send_float(int(_Api.CFG_OUTPUT_MIN) + offset, min_out)
        self._send_float(int(_Api.CFG_OUTPUT_MAX) + offset, max_out)

    # -----------------------------------------------------------------------
    # Soft limits
    # -----------------------------------------------------------------------

    def enable_soft_limits(self, forward: bool, reverse: bool) -> None:
        buf = bytearray(8)
        buf[0] = 0x01 if forward else 0x00
        buf[1] = 0x01 if reverse else 0x00
        self._send_raw(int(_Api.CFG_SOFT_LIMIT_EN), buf, length=2)

    def set_soft_limit(self, forward: bool, limit_rotations: float) -> None:
        api_id = int(_Api.CFG_SOFT_LIMIT_FWD) if forward else int(_Api.CFG_SOFT_LIMIT_REV)
        self._send_float(api_id, limit_rotations)

    # -----------------------------------------------------------------------
    # Flash
    # -----------------------------------------------------------------------

    def burn_flash(self) -> None:
        """Persist current configuration to SPARK MAX onboard flash."""
        buf = bytearray(8)
        buf[0] = 0xA3
        buf[1] = 0x3A
        self._send_raw(int(_Api.CFG_BURN_FLASH), buf, length=2)

    # -----------------------------------------------------------------------
    # Telemetry accessors
    # -----------------------------------------------------------------------

    def get_velocity(self) -> float:
        """Motor velocity in RPM."""
        with self._status_lock:
            return self._status.velocity_rpm

    def get_position(self) -> float:
        """Motor position in rotations."""
        with self._status_lock:
            return self._status.position_rotations

    def get_temperature(self) -> float:
        """Motor temperature in °C."""
        with self._status_lock:
            return self._status.temperature_c

    def get_bus_voltage(self) -> float:
        """Bus voltage in volts."""
        with self._status_lock:
            return self._status.bus_voltage

    def get_current(self) -> float:
        """Motor current in amps."""
        with self._status_lock:
            return self._status.current_amps

    def get_faults(self) -> int:
        """Raw fault bitmask."""
        with self._status_lock:
            return self._status.faults

    def get_sticky_faults(self) -> int:
        with self._status_lock:
            return self._status.sticky_faults

    def has_fault(self, fault: Fault) -> bool:
        with self._status_lock:
            return bool(self._status.faults & int(fault))

    def get_status(self) -> SparkMaxStatus:
        """Return a snapshot copy of all telemetry."""
        with self._status_lock:
            return SparkMaxStatus(
                applied_output=self._status.applied_output,
                faults=self._status.faults,
                sticky_faults=self._status.sticky_faults,
                is_inverted=self._status.is_inverted,
                motor_running=self._status.motor_running,
                velocity_rpm=self._status.velocity_rpm,
                temperature_c=self._status.temperature_c,
                bus_voltage=self._status.bus_voltage,
                current_amps=self._status.current_amps,
                position_rotations=self._status.position_rotations,
            )

    # -----------------------------------------------------------------------
    # Incoming frame parsing (call from your CAN receive loop)
    # -----------------------------------------------------------------------

    def handle_incoming(self, msg: can.Message) -> bool:
        """
        Parse an incoming CAN frame addressed to this device.
        Returns True if the frame was consumed, False otherwise.

        Call this from your bus receive thread for every incoming message:
            for msg in bus:
                for motor in motors:
                    motor.handle_incoming(msg)
        """
        if not msg.is_extended_id:
            return False
        if _extract_manufacturer(msg.arbitration_id) != _MFR_REV:
            return False
        if _extract_device_id(msg.arbitration_id) != self._device_id:
            return False

        api_id = _extract_api_id(msg.arbitration_id)
        dat    = msg.data

        if api_id == int(_Api.STATUS_0):
            return self._parse_status0(dat)
        elif api_id == int(_Api.STATUS_1):
            return self._parse_status1(dat)
        elif api_id == int(_Api.STATUS_2):
            return self._parse_status2(dat)

        return False

    # -----------------------------------------------------------------------
    # Cleanup
    # -----------------------------------------------------------------------

    def close(self) -> None:
        """Stop keep-alive and release resources. Does not close the shared bus."""
        if self._keep_alive_task is not None:
            try:
                self._keep_alive_task.stop()
            except Exception:
                pass
            self._keep_alive_task = None

    # -----------------------------------------------------------------------
    # Private helpers
    # -----------------------------------------------------------------------

    def _send_float(self, api_id: int, value: float) -> None:
        """Send a command frame carrying a single IEEE 754 float (little-endian)."""
        data = bytearray(4)
        struct.pack_into('<f', data, 0, value)
        self._send_raw(api_id, data, length=4)

    def _send_raw(self, api_id: int, buf: bytearray, length: int = 8) -> None:
        """Send a raw CAN frame."""
        arb_id = _build_can_id(api_id, self._device_id)
        msg = can.Message(
            arbitration_id=arb_id,
            data=bytes(buf[:length]),
            is_extended_id=True
        )
        try:
            self._bus.send(msg)
        except can.CanError as e:
            logger.error(f"CAN send error (device {self._device_id}, api 0x{api_id:03X}): {e}")

    def _start_keep_alive(self) -> None:
        """
        Schedule a periodic keep-alive frame using the SocketCAN BCM
        (Broadcast Manager) so the kernel handles the timing, not Python.
        We send the last duty-cycle setpoint at 25 ms intervals.
        """
        arb_id = _build_can_id(int(_Api.CMD_DUTY_CYCLE), self._device_id)
        data = bytearray(4)
        struct.pack_into('<f', data, 0, 0.0)
        msg = can.Message(
            arbitration_id=arb_id,
            data=bytes(data),
            is_extended_id=True
        )
        try:
            self._keep_alive_task = self._bus.send_periodic(
                msg, self.KEEP_ALIVE_INTERVAL
            )
        except Exception as e:
            # send_periodic may not be supported on all interfaces (e.g. vcan)
            logger.warning(f"Could not start BCM keep-alive for device {self._device_id}: {e}. "
                           "Ensure your application calls set_power() or set_reference() "
                           "more frequently than 1 s to avoid SPARK MAX watchdog fault.")

    def _update_keep_alive(self) -> None:
        """Update the keep-alive frame payload to match the last setpoint."""
        if self._keep_alive_task is None:
            return

        api_id = int(_Api.CMD_DUTY_CYCLE)
        if self._ctrl_mode == ControlMode.kVelocity:
            api_id = int(_Api.CMD_SPEED)
        elif self._ctrl_mode == ControlMode.kPosition:
            api_id = int(_Api.CMD_POSITION)

        arb_id = _build_can_id(api_id, self._device_id)
        data = bytearray(4)
        struct.pack_into('<f', data, 0, self._last_setpoint)
        msg = can.Message(
            arbitration_id=arb_id,
            data=bytes(data),
            is_extended_id=True
        )
        try:
            self._keep_alive_task.modify(msgs=[msg])
        except Exception:
            pass  # Not all interfaces support modify()

    # -----------------------------------------------------------------------
    # Status frame parsers
    # -----------------------------------------------------------------------

    def _parse_status0(self, dat: bytes) -> bool:
        """
        Status 0 frame (10 ms):
            [0..1] applied output as int16 / 32767.0  → [-1, 1]
            [2..3] faults bitmask (uint16 LE)
            [4..5] sticky faults bitmask (uint16 LE)
            [6]    bit 3 = inverted, bit 7 = motor running
        """
        if len(dat) < 7:
            return False
        raw_out = struct.unpack_from('<h', dat, 0)[0]  # signed int16
        with self._status_lock:
            self._status.applied_output  = raw_out / 32767.0
            self._status.faults          = struct.unpack_from('<H', dat, 2)[0]
            self._status.sticky_faults   = struct.unpack_from('<H', dat, 4)[0]
            self._status.is_inverted     = bool(dat[6] & 0x08)
            self._status.motor_running   = bool(dat[6] & 0x80)
        return True

    def _parse_status1(self, dat: bytes) -> bool:
        """
        Status 1 frame (20 ms):
            [0..3] velocity (float32 LE, RPM)
            [4]    temperature (uint8, °C)
            [5..6] bus voltage uint16 LE: value * (32.0 / 4096.0) = volts
            [7]    current uint8: value * 0.125 = amps
        """
        if len(dat) < 8:
            return False
        with self._status_lock:
            self._status.velocity_rpm  = struct.unpack_from('<f', dat, 0)[0]
            self._status.temperature_c = float(dat[4])
            raw_v = struct.unpack_from('<H', dat, 5)[0]
            self._status.bus_voltage   = raw_v * (32.0 / 4096.0)
            self._status.current_amps  = dat[7] * 0.125
        return True

    def _parse_status2(self, dat: bytes) -> bool:
        """
        Status 2 frame (20 ms):
            [0..3] position (float32 LE, rotations)
        """
        if len(dat) < 4:
            return False
        with self._status_lock:
            self._status.position_rotations = struct.unpack_from('<f', dat, 0)[0]
        return True