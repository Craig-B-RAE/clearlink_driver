"""
ClearLink EtherNet/IP Communication Layer

Low-level communication with ClearLink controller using pycomm3.
Based on ClearLink EtherNet/IP Object Reference.
"""

import threading
from dataclasses import dataclass
from typing import Optional, Tuple, List
import logging

try:
    from pycomm3 import CIPDriver, Services, ClassCode
    PYCOMM3_AVAILABLE = True
except ImportError:
    PYCOMM3_AVAILABLE = False
    logging.warning("pycomm3 not installed - ClearLink will run in stub mode")


@dataclass
class AxisStatus:
    """Status data for a single axis."""
    enabled: bool = False
    fault: bool = False
    moving: bool = False
    homed: bool = False
    position: int = 0
    velocity: int = 0


@dataclass
class ClearLinkStatus:
    """Complete status from ClearLink controller."""
    connected: bool = False
    connection_error: str = ""
    axes: List[AxisStatus] = None
    digital_inputs: List[bool] = None
    digital_outputs: List[bool] = None
    firmware_version: str = ""
    supply_voltage: float = 0.0

    def __post_init__(self):
        if self.axes is None:
            self.axes = [AxisStatus() for _ in range(4)]
        if self.digital_inputs is None:
            self.digital_inputs = [False] * 13
        if self.digital_outputs is None:
            self.digital_outputs = [False] * 13


class ClearLinkEIP:
    """
    EtherNet/IP communication layer for ClearLink controller.

    ClearLink EtherNet/IP Object Classes:
    - Motor Output Object: 0x66-0x69 (axes 1-4)
    - Motor Input Object: 0x6A-0x6D (axes 1-4)
    - Digital I/O Assembly Object

    Attributes for Motor Output (Class 0x66+):
    - 1: Enable (BOOL)
    - 2: Move Type (USINT)
    - 3: Target Position (DINT)
    - 4: Max Velocity (DINT)
    - 5: Acceleration (DINT)
    - 6: Deceleration (DINT)
    - 7: Trigger Move (BOOL)
    - 8: Clear Faults (BOOL)

    Attributes for Motor Input (Class 0x6A+):
    - 1: Enabled (BOOL)
    - 2: Moving (BOOL)
    - 3: In Fault (BOOL)
    - 4: Homed (BOOL)
    - 5: Position (DINT)
    - 6: Velocity (DINT)
    """

    # ClearLink EtherNet/IP Class IDs
    MOTOR_OUTPUT_BASE = 0x66  # Motor 1 output, +1 for each axis
    MOTOR_INPUT_BASE = 0x6A   # Motor 1 input, +1 for each axis

    # Motor Output Attributes
    ATTR_ENABLE = 1
    ATTR_MOVE_TYPE = 2
    ATTR_TARGET_POSITION = 3
    ATTR_MAX_VELOCITY = 4
    ATTR_ACCELERATION = 5
    ATTR_DECELERATION = 6
    ATTR_TRIGGER_MOVE = 7
    ATTR_CLEAR_FAULTS = 8

    # Motor Input Attributes
    ATTR_IN_ENABLED = 1
    ATTR_IN_MOVING = 2
    ATTR_IN_FAULT = 3
    ATTR_IN_HOMED = 4
    ATTR_IN_POSITION = 5
    ATTR_IN_VELOCITY = 6

    # Move types
    MOVE_TYPE_VELOCITY = 1
    MOVE_TYPE_INCREMENTAL = 2
    MOVE_TYPE_ABSOLUTE = 3

    def __init__(self, ip_address: str, port: int = 44818, num_axes: int = 4):
        self._ip_address = ip_address
        self._port = port
        self._num_axes = num_axes
        self._driver: Optional[CIPDriver] = None
        self._lock = threading.RLock()
        self._connected = False
        self._connection_error = ""
        self._logger = logging.getLogger(__name__)

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def connection_error(self) -> str:
        return self._connection_error

    def connect(self) -> bool:
        """Establish EtherNet/IP connection to ClearLink."""
        if not PYCOMM3_AVAILABLE:
            self._connection_error = "pycomm3 not installed"
            self._logger.warning("pycomm3 not available, using stub mode")
            return False

        with self._lock:
            try:
                self._driver = CIPDriver(self._ip_address)
                self._driver.open()
                self._connected = True
                self._connection_error = ""
                self._logger.info(f"Connected to ClearLink at {self._ip_address}")
                return True
            except Exception as e:
                self._connected = False
                self._connection_error = str(e)
                self._logger.error(f"Failed to connect to ClearLink: {e}")
                return False

    def disconnect(self):
        """Close EtherNet/IP connection."""
        with self._lock:
            if self._driver:
                try:
                    self._driver.close()
                except Exception as e:
                    self._logger.warning(f"Error closing connection: {e}")
                finally:
                    self._driver = None
                    self._connected = False

    def _get_motor_output_class(self, axis: int) -> int:
        """Get the CIP class ID for motor output object."""
        return self.MOTOR_OUTPUT_BASE + (axis - 1)

    def _get_motor_input_class(self, axis: int) -> int:
        """Get the CIP class ID for motor input object."""
        return self.MOTOR_INPUT_BASE + (axis - 1)

    def _write_attribute(self, class_id: int, instance: int, attribute: int,
                         value: any, data_type: str = 'BOOL') -> bool:
        """Write a single attribute via explicit messaging."""
        if not self._connected or not self._driver:
            return False

        with self._lock:
            try:
                # Use generic_message for explicit messaging
                result = self._driver.generic_message(
                    service=Services.set_attribute_single,
                    class_code=class_id,
                    instance=instance,
                    attribute=attribute,
                    request_data=self._encode_value(value, data_type)
                )
                return result is not None
            except Exception as e:
                self._logger.error(f"Write attribute error: {e}")
                return False

    def _read_attribute(self, class_id: int, instance: int, attribute: int,
                        data_type: str = 'BOOL') -> Tuple[bool, any]:
        """Read a single attribute via explicit messaging."""
        if not self._connected or not self._driver:
            return False, None

        with self._lock:
            try:
                result = self._driver.generic_message(
                    service=Services.get_attribute_single,
                    class_code=class_id,
                    instance=instance,
                    attribute=attribute
                )
                if result:
                    return True, self._decode_value(result.value, data_type)
                return False, None
            except Exception as e:
                self._logger.error(f"Read attribute error: {e}")
                return False, None

    def _encode_value(self, value: any, data_type: str) -> bytes:
        """Encode a value for CIP messaging."""
        import struct
        if data_type == 'BOOL':
            return struct.pack('?', bool(value))
        elif data_type == 'USINT':
            return struct.pack('B', int(value))
        elif data_type == 'DINT':
            return struct.pack('<i', int(value))
        elif data_type == 'UDINT':
            return struct.pack('<I', int(value))
        return bytes([int(value)])

    def _decode_value(self, data: bytes, data_type: str) -> any:
        """Decode a value from CIP messaging."""
        import struct
        if not data:
            return None
        if data_type == 'BOOL':
            return struct.unpack('?', data[:1])[0]
        elif data_type == 'USINT':
            return struct.unpack('B', data[:1])[0]
        elif data_type == 'DINT':
            return struct.unpack('<i', data[:4])[0]
        elif data_type == 'UDINT':
            return struct.unpack('<I', data[:4])[0]
        return data

    # Motor Control Methods

    def set_motor_enable(self, axis: int, enable: bool) -> bool:
        """Enable or disable a motor axis."""
        if axis < 1 or axis > self._num_axes:
            return False
        class_id = self._get_motor_output_class(axis)
        return self._write_attribute(class_id, 1, self.ATTR_ENABLE, enable, 'BOOL')

    def set_velocity(self, axis: int, steps_per_sec: int, accel: int = 10000) -> bool:
        """Set velocity for continuous motion."""
        if axis < 1 or axis > self._num_axes:
            return False

        class_id = self._get_motor_output_class(axis)

        # Set move type to velocity mode
        if not self._write_attribute(class_id, 1, self.ATTR_MOVE_TYPE,
                                      self.MOVE_TYPE_VELOCITY, 'USINT'):
            return False

        # Set velocity (signed for direction)
        if not self._write_attribute(class_id, 1, self.ATTR_MAX_VELOCITY,
                                      steps_per_sec, 'DINT'):
            return False

        # Set acceleration
        if not self._write_attribute(class_id, 1, self.ATTR_ACCELERATION,
                                      accel, 'UDINT'):
            return False

        return True

    def trigger_move(self, axis: int) -> bool:
        """Trigger the configured move on an axis."""
        if axis < 1 or axis > self._num_axes:
            return False
        class_id = self._get_motor_output_class(axis)
        return self._write_attribute(class_id, 1, self.ATTR_TRIGGER_MOVE, True, 'BOOL')

    def clear_faults(self, axis: int) -> bool:
        """Clear faults on an axis."""
        if axis < 1 or axis > self._num_axes:
            return False
        class_id = self._get_motor_output_class(axis)
        return self._write_attribute(class_id, 1, self.ATTR_CLEAR_FAULTS, True, 'BOOL')

    def stop_motor(self, axis: int) -> bool:
        """Stop a motor by setting velocity to zero."""
        return self.set_velocity(axis, 0, 50000)  # High decel for quick stop

    def stop_all(self) -> bool:
        """Stop all motors."""
        success = True
        for axis in range(1, self._num_axes + 1):
            if not self.stop_motor(axis):
                success = False
        return success

    # Status Reading Methods

    def get_axis_status(self, axis: int) -> AxisStatus:
        """Read complete status for one axis."""
        status = AxisStatus()

        if axis < 1 or axis > self._num_axes:
            return status

        if not self._connected:
            return status

        class_id = self._get_motor_input_class(axis)

        # Read enabled state
        success, value = self._read_attribute(class_id, 1, self.ATTR_IN_ENABLED, 'BOOL')
        if success:
            status.enabled = value

        # Read moving state
        success, value = self._read_attribute(class_id, 1, self.ATTR_IN_MOVING, 'BOOL')
        if success:
            status.moving = value

        # Read fault state
        success, value = self._read_attribute(class_id, 1, self.ATTR_IN_FAULT, 'BOOL')
        if success:
            status.fault = value

        # Read homed state
        success, value = self._read_attribute(class_id, 1, self.ATTR_IN_HOMED, 'BOOL')
        if success:
            status.homed = value

        # Read position
        success, value = self._read_attribute(class_id, 1, self.ATTR_IN_POSITION, 'DINT')
        if success:
            status.position = value

        # Read velocity
        success, value = self._read_attribute(class_id, 1, self.ATTR_IN_VELOCITY, 'DINT')
        if success:
            status.velocity = value

        return status

    def get_all_status(self) -> ClearLinkStatus:
        """Read complete status from ClearLink."""
        status = ClearLinkStatus()
        status.connected = self._connected
        status.connection_error = self._connection_error

        if not self._connected:
            return status

        # Read status for each axis
        for i in range(self._num_axes):
            status.axes[i] = self.get_axis_status(i + 1)

        return status
