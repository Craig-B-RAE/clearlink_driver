"""
ClearLink EtherNet/IP Communication Layer

Low-level communication with ClearLink controller using pycomm3.
Based on ClearLink EtherNet/IP Object Reference for Step & Direction motors.

IMPORTANT: For Step & Direction motors, all motor axes use the SAME class
with different INSTANCES (not different classes):
- Motor Config:  Class 0x64, Instance 1-4 for M0-M3
- Motor Input:   Class 0x65, Instance 1-4 for M0-M3
- Motor Output:  Class 0x66, Instance 1-4 for M0-M3
"""

import threading
from dataclasses import dataclass
from typing import Optional, Tuple, List
import logging
import time

try:
    from pycomm3 import CIPDriver
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
    torque: float = -9999.0  # Measured torque percentage (-100 to +100, -9999 = N/A)
    shutdown: int = 0  # Raw shutdown register


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

    ClearLink EtherNet/IP Object Classes (Step & Direction motors):
    - Motor Config:  0x64 (Instance 1-4 for M0-M3)
    - Motor Input:   0x65 (Instance 1-4 for M0-M3)
    - Motor Output:  0x66 (Instance 1-4 for M0-M3)

    Motor Config (0x64) Attributes:
    - 1: Negative Soft Limit (DINT)
    - 2: Positive Soft Limit (DINT)

    Motor Output (0x66) Attributes:
    - 2: Velocity Limit (DINT)
    - 3: Acceleration (DINT)
    - 4: Deceleration (DINT)
    - 5: Position Target (DINT)
    - 6: Output Register (DINT) - command bits
    - 7: Jog Velocity (DINT) - signed for direction

    Motor Input (0x65) Attributes:
    - 1: Commanded Position (DINT)
    - 2: Commanded Velocity (DINT)
    - 7: Status Register (DWORD)
    - 8: Shutdown Register (DWORD)

    Output Register bits:
    - Bit 0 (0x01): Enable
    - Bit 4 (0x10): Load Velocity Move
    - Bit 6 (0x40): Clear Alerts
    - Bit 7 (0x80): Clear Motor Fault

    Status Register bits:
    - Bit 1: Steps Active (moving)
    - Bit 9: Motor In Fault
    - Bit 10: Enabled
    - Bit 17: Shutdowns Present

    IMPORTANT: To move, you must HOLD 0x11 (Enable + Load Velocity Move) high.
    """

    # ClearLink EtherNet/IP Class IDs (Step & Direction)
    MOTOR_CONFIG_CLASS = 0x64
    MOTOR_INPUT_CLASS = 0x65
    MOTOR_OUTPUT_CLASS = 0x66

    # Motor Config Attributes
    ATTR_CFG_NEG_SOFT_LIMIT = 1
    ATTR_CFG_POS_SOFT_LIMIT = 2

    # Motor Output Attributes (corrected per CLAUDE.md)
    ATTR_OUT_JOG_VEL = 2      # Jog Velocity (signed for direction)
    ATTR_OUT_VEL_LIMIT = 3    # Velocity Limit
    ATTR_OUT_ACCEL = 4        # Acceleration
    ATTR_OUT_DECEL = 5        # Deceleration
    ATTR_OUT_OUTPUT_REG = 6   # Output Register (command bits)

    # Motor Input Attributes
    ATTR_IN_CMD_POSITION = 1
    ATTR_IN_CMD_VELOCITY = 2
    ATTR_IN_TORQUE = 6  # Measured torque (REAL, -100 to +100%, -9999 = N/A)
    ATTR_IN_STATUS_REG = 7
    ATTR_IN_SHUTDOWN_REG = 8

    # Output Register bits
    BIT_ENABLE = 0x01
    BIT_LOAD_VEL_MOVE = 0x10
    BIT_CLEAR_ALERTS = 0x40
    BIT_CLEAR_MOTOR_FAULT = 0x80

    # Status Register bits
    BIT_STEPS_ACTIVE = (1 << 1)
    BIT_MOTOR_FAULT = (1 << 9)
    BIT_ENABLED = (1 << 10)
    BIT_SHUTDOWNS_PRESENT = (1 << 17)
    BIT_LOAD_VEL_MOVE_ACK = (1 << 20)  # Load Velocity Move Acknowledge

    def __init__(self, ip_address: str, port: int = 44818, num_axes: int = 4):
        self._ip_address = ip_address
        self._port = port
        self._num_axes = num_axes
        self._driver: Optional[CIPDriver] = None
        self._lock = threading.RLock()
        self._connected = False
        self._connection_error = ""
        self._logger = logging.getLogger(__name__)

        # Track current output register state for each axis
        self._output_reg = [0] * num_axes
        # Track which motors have been enabled (to avoid disruptive re-enable)
        self._motors_enabled = [False] * num_axes
        # Track consecutive read failures to detect connection loss
        self._consecutive_read_failures = 0
        self._max_read_failures = 3

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
                self._consecutive_read_failures = 0
                self._logger.info(f"Connected to ClearLink at {self._ip_address}")

                # Initialize all axes with wide soft limits
                self._init_axes()

                return True
            except Exception as e:
                self._connected = False
                self._connection_error = str(e)
                self._logger.error(f"Failed to connect to ClearLink: {e}")
                return False

    def _init_axes(self):
        """Initialize all axes with wide soft limits."""
        for axis in range(1, self._num_axes + 1):
            # Set very wide soft limits to prevent soft limit faults
            self._write_attr(self.MOTOR_CONFIG_CLASS, axis,
                           self.ATTR_CFG_NEG_SOFT_LIMIT, -2000000000)
            self._write_attr(self.MOTOR_CONFIG_CLASS, axis,
                           self.ATTR_CFG_POS_SOFT_LIMIT, 2000000000)

            # Set default velocity parameters
            self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                           self.ATTR_OUT_VEL_LIMIT, 1000000)
            self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                           self.ATTR_OUT_ACCEL, 500000)
            self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                           self.ATTR_OUT_DECEL, 500000)

        self._logger.info("Initialized axes with wide soft limits")

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
                    self._motors_enabled = [False] * self._num_axes

    def reconnect(self) -> bool:
        """Disconnect any stale driver, then attempt a fresh connection."""
        self.disconnect()
        return self.connect()

    def _write_attr(self, class_id: int, instance: int, attribute: int,
                    value: int) -> bool:
        """Write a DINT attribute."""
        if not self._connected or not self._driver:
            return False

        try:
            result = self._driver.generic_message(
                service=0x10,  # Set Attribute Single
                class_code=class_id,
                instance=instance,
                attribute=attribute,
                request_data=value.to_bytes(4, 'little', signed=True if value < 0 else False)
            )
            return result is not None
        except Exception as e:
            self._logger.error(f"Write attribute error: {e}")
            return False

    def _read_attr(self, class_id: int, instance: int, attribute: int) -> Optional[int]:
        """Read a DINT attribute."""
        if not self._connected or not self._driver:
            return None

        try:
            result = self._driver.generic_message(
                service=0x0E,  # Get Attribute Single
                class_code=class_id,
                instance=instance,
                attribute=attribute
            )
            if result and result.value:
                self._consecutive_read_failures = 0
                return int.from_bytes(result.value, 'little', signed=True)
            return None
        except Exception as e:
            self._consecutive_read_failures += 1
            self._logger.error(f"Read attribute error: {e}")
            if self._consecutive_read_failures >= self._max_read_failures:
                self._logger.error(f"Too many consecutive read failures ({self._consecutive_read_failures}), marking disconnected")
                self._connected = False
                self._connection_error = f"Read failures: {e}"
            return None

    def _read_attr_float(self, class_id: int, instance: int, attribute: int) -> Optional[float]:
        """Read a REAL (float32) attribute."""
        import struct
        if not self._connected or not self._driver:
            return None

        try:
            result = self._driver.generic_message(
                service=0x0E,  # Get Attribute Single
                class_code=class_id,
                instance=instance,
                attribute=attribute
            )
            if result and result.value and len(result.value) >= 4:
                self._consecutive_read_failures = 0
                return struct.unpack('<f', result.value[:4])[0]
            return None
        except Exception as e:
            self._consecutive_read_failures += 1
            self._logger.error(f"Read float attribute error: {e}")
            if self._consecutive_read_failures >= self._max_read_failures:
                self._logger.error(f"Too many consecutive read failures ({self._consecutive_read_failures}), marking disconnected")
                self._connected = False
                self._connection_error = f"Read failures: {e}"
            return None

    def _write_output_reg(self, axis: int, value: int) -> bool:
        """Write the output register for an axis."""
        if axis < 1 or axis > self._num_axes:
            return False
        success = self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                                  self.ATTR_OUT_OUTPUT_REG, value)
        if success:
            self._output_reg[axis - 1] = value
        return success

    def _read_status_reg(self, axis: int) -> Optional[int]:
        """Read the status register for an axis."""
        if axis < 1 or axis > self._num_axes:
            return None
        return self._read_attr(self.MOTOR_INPUT_CLASS, axis, self.ATTR_IN_STATUS_REG)

    def _read_shutdown_reg(self, axis: int) -> Optional[int]:
        """Read the shutdown register for an axis."""
        if axis < 1 or axis > self._num_axes:
            return None
        return self._read_attr(self.MOTOR_INPUT_CLASS, axis, self.ATTR_IN_SHUTDOWN_REG)

    # Motor Control Methods

    def clear_faults(self, axis: int) -> bool:
        """Clear faults on an axis."""
        if axis < 1 or axis > self._num_axes:
            return False

        with self._lock:
            # Write Clear Alerts + Clear Motor Fault
            self._write_output_reg(axis, self.BIT_CLEAR_ALERTS | self.BIT_CLEAR_MOTOR_FAULT)
            time.sleep(0.3)
            # Clear the command
            self._write_output_reg(axis, 0)
            time.sleep(0.1)

            # Check if shutdown cleared
            shutdown = self._read_shutdown_reg(axis)
            if shutdown and shutdown != 0:
                self._logger.warning(f"Axis {axis} shutdown not fully cleared: 0x{shutdown:08X}")
                return False

            return True

    def set_motor_enable(self, axis: int, enable: bool) -> bool:
        """Enable or disable a motor axis.

        Note: ClearLink may not report Enabled bit in status register even
        when motor is operational. We verify by checking shutdown register
        instead of status enabled bit.
        
        IMPORTANT: If motor is already enabled, we skip the fault-clear sequence
        to avoid disrupting an active motor (writing 0x00 would disable it).
        """
        if axis < 1 or axis > self._num_axes:
            return False

        with self._lock:
            if enable:
                # Check if already enabled - skip disruptive sequence
                if self._motors_enabled[axis - 1]:
                    # Motor already enabled, just ensure output reg has enable bit
                    # Don't write 0x01 if we're in the middle of moving (0x11)
                    if self._output_reg[axis - 1] & self.BIT_ENABLE:
                        return True
                    # Re-enable if somehow lost
                    self._write_output_reg(axis, self.BIT_ENABLE)
                    return True

                # First-time enable: clear faults and enable
                self._write_output_reg(axis, self.BIT_CLEAR_ALERTS | self.BIT_CLEAR_MOTOR_FAULT)
                time.sleep(0.3)
                self._write_output_reg(axis, 0)
                time.sleep(0.1)

                # Enable the motor
                self._write_output_reg(axis, self.BIT_ENABLE)
                time.sleep(0.2)

                # Check shutdown register - should be clear or only have non-blocking faults
                shutdown = self._read_shutdown_reg(axis)
                # Bit 5 (0x20) = MotorDisabled - this is OK, just means we haven't started moving yet
                # Bit 10 (0x400) = MotorFaulted - this would block operation
                blocking_faults = shutdown & 0x400 if shutdown else 0

                if blocking_faults:
                    self._logger.warning(f"Axis {axis} has blocking faults: 0x{shutdown:08X}")
                    return False

                shutdown_hex = f"0x{shutdown:08X}" if shutdown else "0x00"
                self._logger.info(f"Axis {axis} enabled (shutdown={shutdown_hex})")
                self._motors_enabled[axis - 1] = True
                return True
            else:
                # Disable
                self._write_output_reg(axis, 0)
                self._motors_enabled[axis - 1] = False
                return True

    def set_velocity(self, axis: int, steps_per_sec: int, accel: int = 500000) -> bool:
        """Set velocity for continuous motion."""
        if axis < 1 or axis > self._num_axes:
            return False

        with self._lock:
            self._logger.info(f"[set_velocity] Axis {axis}: vel={steps_per_sec}, accel={accel}")

            # Set velocity limit
            self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                           self.ATTR_OUT_VEL_LIMIT, abs(steps_per_sec) + 100000)

            # Set acceleration/deceleration
            self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                           self.ATTR_OUT_ACCEL, accel)
            self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                           self.ATTR_OUT_DECEL, accel)

            # Set jog velocity (signed for direction)
            self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                           self.ATTR_OUT_JOG_VEL, steps_per_sec)

            # Read back to verify
            jog_vel = self._read_attr(self.MOTOR_OUTPUT_CLASS, axis, self.ATTR_OUT_JOG_VEL)
            self._logger.info(f"[set_velocity] Axis {axis}: jog_vel readback = {jog_vel}")

            return True

    def trigger_move(self, axis: int) -> bool:
        """Start or continue velocity movement.

        IMPORTANT: Before triggering a new move, must ensure any pending
        Load Vel Move Ack is cleared. Otherwise ClearLink ignores new commands.
        Also clears any shutdown faults that would block motion.
        """
        if axis < 1 or axis > self._num_axes:
            return False

        with self._lock:
            # Check for shutdown faults that would block motion
            shutdown_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                          self.ATTR_IN_SHUTDOWN_REG)
            # Non-blocking bits (0x421): MotorDisabled(0x20) + MotorFaulted(0x400) + TrackingError(0x01)
            # But 0x001 (Command While Shutdown) IS blocking, so we check for any faults
            if shutdown_reg is not None and shutdown_reg != 0:
                shutdown_hex = f"0x{shutdown_reg:04x}" if shutdown_reg is not None else "None"
                self._logger.info(f"[trigger_move] Axis {axis}: shutdown={shutdown_hex}, clearing faults")
                # Clear faults: 0xC0 = Clear Alerts (0x40) + Clear Motor Fault (0x80)
                self._write_output_reg(axis, self.BIT_CLEAR_ALERTS | self.BIT_CLEAR_MOTOR_FAULT)
                time.sleep(0.1)
                self._write_output_reg(axis, 0x00)
                time.sleep(0.05)
                # Re-enable
                self._write_output_reg(axis, self.BIT_ENABLE)
                time.sleep(0.05)

            # Check if there's a pending ack from previous move
            status_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                        self.ATTR_IN_STATUS_REG)
            ack_set = bool(status_reg & self.BIT_LOAD_VEL_MOVE_ACK) if status_reg is not None else False
            status_hex = f"0x{status_reg:08x}" if status_reg is not None else "None"
            self._logger.debug(f"[trigger_move] Axis {axis}: status_reg={status_hex}, ack={ack_set}")

            if status_reg is not None and (status_reg & self.BIT_LOAD_VEL_MOVE_ACK):
                self._logger.debug(f"[trigger_move] Axis {axis}: Clearing pending ack before new move")
                # Clear Load bit first
                self._write_output_reg(axis, self.BIT_ENABLE)
                # Wait for ack to clear (max 100ms)
                for i in range(10):
                    time.sleep(0.01)
                    status_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                                self.ATTR_IN_STATUS_REG)
                    if status_reg is not None and not (status_reg & self.BIT_LOAD_VEL_MOVE_ACK):
                        self._logger.debug(f"[trigger_move] Axis {axis}: Ack cleared after {i*10}ms")
                        break

            # Write Enable + Load Velocity Move
            result = self._write_output_reg(axis, self.BIT_ENABLE | self.BIT_LOAD_VEL_MOVE)

            return result

    def stop_motor(self, axis: int) -> bool:
        """Stop a motor but keep it enabled.

        Uses the Load Velocity Move Ack handshake per ClearLink protocol.
        Must clear any pending ack before triggering new move.

        CRITICAL: The Load bit (0x10) must be cleared after stop, otherwise
        subsequent jog commands will be ignored by ClearLink.
        """
        if axis < 1 or axis > self._num_axes:
            return False

        with self._lock:
            # Step 1: Clear any pending ack from previous move
            status_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                        self.ATTR_IN_STATUS_REG)
            if status_reg is not None and (status_reg & self.BIT_LOAD_VEL_MOVE_ACK):
                self._logger.info(f"[stop_motor] Axis {axis}: Clearing pending ack first")
                self._write_output_reg(axis, self.BIT_ENABLE)  # Clear load bit
                # Wait for ack to clear
                for i in range(50):
                    status_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                                self.ATTR_IN_STATUS_REG)
                    if status_reg is not None and not (status_reg & self.BIT_LOAD_VEL_MOVE_ACK):
                        self._logger.info(f"[stop_motor] Axis {axis}: Pending ack cleared after {i*10}ms")
                        break
                    time.sleep(0.01)

            # Step 2: Set velocity to 0
            self._logger.debug(f"[stop_motor] Axis {axis}: Setting velocity to 0")
            self._write_attr(self.MOTOR_OUTPUT_CLASS, axis,
                           self.ATTR_OUT_JOG_VEL, 0)

            # Step 3: Trigger the zero-velocity move
            self._logger.debug(f"[stop_motor] Axis {axis}: Triggering with 0x11")
            self._write_output_reg(axis, self.BIT_ENABLE | self.BIT_LOAD_VEL_MOVE)

            # Step 4: Wait for Load Vel Move Ack (bit 20) to become 1
            ack_found = False
            for i in range(50):  # Max 500ms
                status_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                            self.ATTR_IN_STATUS_REG)
                if status_reg is not None and (status_reg & self.BIT_LOAD_VEL_MOVE_ACK):
                    ack_found = True
                    self._logger.debug(f"[stop_motor] Axis {axis}: Ack found after {i*10}ms")
                    break
                time.sleep(0.01)
            if not ack_found:
                self._logger.warning(f"[stop_motor] Axis {axis}: Ack NOT found after 500ms")

            # Step 5: Clear load bit but keep enabled - WITH VERIFICATION
            # This is critical - if Load bit stays set, motor won't respond to new commands
            clear_success = False
            for attempt in range(3):  # Try up to 3 times
                self._logger.debug(f"[stop_motor] Axis {axis}: Clearing to 0x01 (attempt {attempt+1})")
                self._write_output_reg(axis, self.BIT_ENABLE)
                time.sleep(0.02)  # Short delay for write to take effect

                # Verify the write succeeded by reading back Output Register
                out_reg = self._read_attr(self.MOTOR_OUTPUT_CLASS, axis, self.ATTR_OUT_OUTPUT_REG)
                if out_reg is not None:
                    if not (out_reg & self.BIT_LOAD_VEL_MOVE):
                        # Load bit is cleared
                        clear_success = True
                        self._logger.debug(f"[stop_motor] Axis {axis}: Output reg verified as 0x{out_reg:02x}")
                        break
                    else:
                        self._logger.warning(f"[stop_motor] Axis {axis}: Output reg still 0x{out_reg:02x}, retrying")
                else:
                    self._logger.warning(f"[stop_motor] Axis {axis}: Failed to read output reg for verification")

            if not clear_success:
                self._logger.error(f"[stop_motor] Axis {axis}: FAILED to clear Load bit after 3 attempts!")

            # Step 6: Wait for ack bit to clear in Status Register
            ack_cleared = False
            for i in range(50):  # Max 500ms
                status_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                            self.ATTR_IN_STATUS_REG)
                if status_reg is not None and not (status_reg & self.BIT_LOAD_VEL_MOVE_ACK):
                    ack_cleared = True
                    self._logger.debug(f"[stop_motor] Axis {axis}: Ack cleared after {i*10}ms")
                    break
                time.sleep(0.01)

            if not ack_cleared:
                self._logger.warning(f"[stop_motor] Axis {axis}: Status Ack bit did not clear after 500ms")

            return clear_success

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

        with self._lock:
            # Read status register
            status_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                        self.ATTR_IN_STATUS_REG)
            if status_reg is not None:
                status.enabled = bool(status_reg & self.BIT_ENABLED)
                status.moving = bool(status_reg & self.BIT_STEPS_ACTIVE)
                # Homed bit - check if ReadyToHome bit is clear (bit 12)
                status.homed = not bool(status_reg & (1 << 12))

            # Read shutdown register - use this for fault detection
            shutdown_reg = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                          self.ATTR_IN_SHUTDOWN_REG)
            if shutdown_reg is not None:
                status.shutdown = shutdown_reg
                # Use shutdown register for fault detection instead of status register
                # These bits are normal/informational and don't block operation:
                # - 0x01: TrackingError - appears but doesn't prevent movement
                # - 0x20: MotorDisabled - normal when not moving
                # - 0x400: Bit 10 - informational
                non_blocking = 0x421  # TrackingError + MotorDisabled + bit 10
                blocking_faults = shutdown_reg & ~non_blocking
                status.fault = blocking_faults != 0

            # Read position
            pos = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                 self.ATTR_IN_CMD_POSITION)
            if pos is not None:
                status.position = pos

            # Read velocity
            vel = self._read_attr(self.MOTOR_INPUT_CLASS, axis,
                                 self.ATTR_IN_CMD_VELOCITY)
            if vel is not None:
                status.velocity = vel

            # Read torque (REAL value, -100 to +100%, -9999 = N/A)
            torque = self._read_attr_float(self.MOTOR_INPUT_CLASS, axis,
                                          self.ATTR_IN_TORQUE)
            if torque is not None:
                status.torque = torque

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
