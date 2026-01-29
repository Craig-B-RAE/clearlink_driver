"""
ClearLink Control Layer

Thread-safe control interface wrapping the EtherNet/IP communication layer.
Provides higher-level motor control methods with error handling.
"""

import threading
from dataclasses import dataclass
from typing import List, Optional, Tuple
import logging

from .clearlink_eip import ClearLinkEIP, ClearLinkStatus, AxisStatus


@dataclass
class ClearLinkDiagnostics:
    """Diagnostic data from ClearLink controller."""
    connected: bool = False
    connection_error: str = ""
    axis_faults: List[bool] = None
    total_commands_sent: int = 0
    total_errors: int = 0

    def __post_init__(self):
        if self.axis_faults is None:
            self.axis_faults = [False] * 4


class ClearLinkControl:
    """
    Thread-safe control layer for ClearLink motor controller.

    Provides higher-level motor control methods with:
    - Thread-safe access via RLock
    - Error handling and retry logic
    - Command tracking for diagnostics
    """

    def __init__(self, ip_address: str, port: int = 44818, num_axes: int = 4):
        """
        Initialize ClearLink control.

        Args:
            ip_address: IP address of ClearLink controller
            port: EtherNet/IP port (default 44818)
            num_axes: Number of motor axes (1-4)
        """
        self._eip = ClearLinkEIP(ip_address, port, num_axes)
        self._num_axes = num_axes
        self._lock = threading.RLock()
        self._logger = logging.getLogger(__name__)

        # Command tracking
        self._last_velocities = [0] * num_axes
        self._commands_sent = 0
        self._errors = 0

    @property
    def connected(self) -> bool:
        """Check if connected to ClearLink."""
        return self._eip.connected

    @property
    def connection_error(self) -> str:
        """Get last connection error message."""
        return self._eip.connection_error

    def initialize(self) -> bool:
        """
        Initialize connection to ClearLink.

        Returns:
            True if connection successful
        """
        with self._lock:
            success = self._eip.connect()
            if success:
                self._logger.info("ClearLink control initialized")
            return success

    def shutdown(self):
        """Shutdown ClearLink connection."""
        with self._lock:
            # Stop all motors before disconnecting
            self._eip.stop_all()
            self._eip.disconnect()
            self._logger.info("ClearLink control shutdown")

    def enable_motors(self, axes: List[int]) -> bool:
        """
        Enable specified motor axes.

        Args:
            axes: List of axis numbers (1-4) to enable

        Returns:
            True if all axes enabled successfully
        """
        with self._lock:
            success = True
            for axis in axes:
                if not self._eip.set_motor_enable(axis, True):
                    self._logger.error(f"Failed to enable axis {axis}")
                    self._errors += 1
                    success = False
                else:
                    self._commands_sent += 1
            return success

    def disable_motors(self, axes: List[int]) -> bool:
        """
        Disable specified motor axes.

        Args:
            axes: List of axis numbers (1-4) to disable

        Returns:
            True if all axes disabled successfully
        """
        with self._lock:
            success = True
            for axis in axes:
                if not self._eip.set_motor_enable(axis, False):
                    self._logger.error(f"Failed to disable axis {axis}")
                    self._errors += 1
                    success = False
                else:
                    self._commands_sent += 1
            return success

    def drive_velocity(self, axis_velocities: List[int], accel: int = 10000) -> bool:
        """
        Set velocities for all axes.

        Args:
            axis_velocities: List of velocities in steps/sec for each axis
                             (positive = forward, negative = reverse)
            accel: Acceleration in steps/sec^2

        Returns:
            True if all velocity commands successful
        """
        with self._lock:
            success = True
            print(f"[ClearLink] drive_velocity: incoming={axis_velocities}, last={self._last_velocities}", flush=True)

            for i, velocity in enumerate(axis_velocities):
                axis = i + 1
                if axis > self._num_axes:
                    break

                # Always process velocity=0 (stop) commands - don't skip even if unchanged
                # This ensures motors stop even if previous stop failed
                if velocity != 0 and velocity == self._last_velocities[i]:
                    self._logger.debug(f"Axis {axis}: velocity unchanged at {velocity}, skipping")
                    continue

                print(f"[ClearLink] Axis {axis}: velocity {self._last_velocities[i]} -> {velocity}", flush=True)

                # Use stop_motor for velocity=0 (requires full handshake)
                if velocity == 0:
                    print(f"[ClearLink] Axis {axis}: calling stop_motor", flush=True)
                    stop_ok = self._eip.stop_motor(axis)
                    if not stop_ok:
                        self._logger.error(f"Failed to stop axis {axis}")
                        self._errors += 1
                        success = False
                        continue
                    self._last_velocities[i] = 0
                    self._commands_sent += 1
                    continue

                # Set velocity
                vel_ok = self._eip.set_velocity(axis, velocity, accel)
                if not vel_ok:
                    self._logger.error(f"Failed to set velocity for axis {axis}")
                    self._errors += 1
                    success = False
                    continue

                # Trigger the move
                trig_ok = self._eip.trigger_move(axis)
                if not trig_ok:
                    self._logger.error(f"Failed to trigger move for axis {axis}")
                    self._errors += 1
                    success = False
                    continue

                self._last_velocities[i] = velocity
                self._commands_sent += 1

            return success

    def stop(self, decel: int = 50000) -> bool:
        """
        Stop all motors.

        Args:
            decel: Deceleration rate in steps/sec^2

        Returns:
            True if stop command successful
        """
        with self._lock:
            success = self._eip.stop_all()
            if success:
                self._last_velocities = [0] * self._num_axes
                self._commands_sent += 1
            else:
                self._errors += 1
            return success

    def clear_faults(self, axes: List[int]) -> Tuple[bool, str]:
        """
        Clear faults on specified axes.

        Args:
            axes: List of axis numbers (1-4) to clear

        Returns:
            Tuple of (success, message)
        """
        with self._lock:
            cleared = []
            failed = []

            for axis in axes:
                if self._eip.clear_faults(axis):
                    cleared.append(str(axis))
                    self._commands_sent += 1
                else:
                    failed.append(str(axis))
                    self._errors += 1

            if failed:
                return False, f"Failed to clear faults on axes: {', '.join(failed)}"
            return True, f"Cleared faults on axes: {', '.join(cleared)}"

    def home(self, axes: List[int], velocity: int, accel: int) -> Tuple[bool, str]:
        """
        Home specified axes.

        Note: Actual homing implementation depends on ClearPath servo configuration.
        This is a placeholder that would need to be customized for the specific
        homing sequence required.

        Args:
            axes: List of axis numbers (1-4) to home
            velocity: Homing velocity in steps/sec
            accel: Homing acceleration in steps/sec^2

        Returns:
            Tuple of (success, message)
        """
        with self._lock:
            # Placeholder - actual homing would be more complex
            self._logger.info(f"Homing axes {axes} at velocity {velocity}")
            return True, "Homing initiated (not fully implemented)"

    def read_status(self) -> ClearLinkStatus:
        """
        Read complete status from ClearLink.

        Returns:
            ClearLinkStatus with current state of all axes
        """
        with self._lock:
            return self._eip.get_all_status()

    def read_diagnostics(self) -> ClearLinkDiagnostics:
        """
        Read diagnostic information.

        Returns:
            ClearLinkDiagnostics with connection and error stats
        """
        diag = ClearLinkDiagnostics()
        diag.connected = self._eip.connected
        diag.connection_error = self._eip.connection_error
        diag.total_commands_sent = self._commands_sent
        diag.total_errors = self._errors

        # Get fault status for each axis
        status = self.read_status()
        diag.axis_faults = [status.axes[i].fault for i in range(self._num_axes)]

        return diag
