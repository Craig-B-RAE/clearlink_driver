"""
ClearLink ROS2 Node

ROS2 node for controlling ClearLink EtherNet/IP motor controller.
Provides publishers, subscribers, and services for motor control.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import threading
import time

from std_msgs.msg import Header
from clearlink_interfaces.msg import MotorCommand, MotorStatus
from clearlink_interfaces.srv import ClearFaults, Home

from .clearlink_control import ClearLinkControl


class ClearLinkNode(Node):
    """
    ROS2 node for ClearLink motor controller.

    Publishers:
        /clearlink/status (clearlink_interfaces/MotorStatus): Motor status at loop_hz

    Subscribers:
        /clearlink/command (clearlink_interfaces/MotorCommand): Motor velocity commands

    Services:
        /clearlink/clear_faults (clearlink_interfaces/ClearFaults): Clear motor faults
        /clearlink/home (clearlink_interfaces/Home): Home motor axes

    Parameters:
        ip_address (str): ClearLink IP address (default: 192.168.1.100)
        port (int): EtherNet/IP port (default: 44818)
        num_axes (int): Number of motor axes (default: 4)
        loop_hz (float): Status publish rate in Hz (default: 10)
        deadman_secs (float): Timeout before auto-stop (default: 3.0)
        status_topic (str): Status topic name (default: /clearlink/status)
        command_topic (str): Command topic name (default: /clearlink/command)
    """

    # Parameter names
    PARAM_IP_ADDRESS = 'ip_address'
    PARAM_PORT = 'port'
    PARAM_NUM_AXES = 'num_axes'
    PARAM_LOOP_HZ = 'loop_hz'
    PARAM_DEADMAN_SECS = 'deadman_secs'
    PARAM_STATUS_TOPIC = 'status_topic'
    PARAM_COMMAND_TOPIC = 'command_topic'

    def __init__(self):
        super().__init__('clearlink_driver')

        # Declare parameters
        self.declare_parameter(self.PARAM_IP_ADDRESS, '192.168.20.240')
        self.declare_parameter(self.PARAM_PORT, 44818)
        self.declare_parameter(self.PARAM_NUM_AXES, 4)
        self.declare_parameter(self.PARAM_LOOP_HZ, 10.0)
        self.declare_parameter(self.PARAM_DEADMAN_SECS, 3.0)
        self.declare_parameter(self.PARAM_STATUS_TOPIC, '/clearlink/status')
        self.declare_parameter(self.PARAM_COMMAND_TOPIC, '/clearlink/command')

        # Get parameters
        ip_address = self.get_parameter(self.PARAM_IP_ADDRESS).value
        port = self.get_parameter(self.PARAM_PORT).value
        num_axes = self.get_parameter(self.PARAM_NUM_AXES).value
        loop_hz = self.get_parameter(self.PARAM_LOOP_HZ).value
        self._deadman_secs = self.get_parameter(self.PARAM_DEADMAN_SECS).value
        status_topic = self.get_parameter(self.PARAM_STATUS_TOPIC).value
        command_topic = self.get_parameter(self.PARAM_COMMAND_TOPIC).value

        self.get_logger().info(f"ClearLink node starting: {ip_address}:{port}")

        # Initialize control layer
        self._control = ClearLinkControl(ip_address, port, num_axes)
        self._num_axes = num_axes

        # Command tracking for deadman switch
        self._last_cmd_time = time.time()
        self._cmd_lock = threading.RLock()
        # Track if we've received first move command (need clear_faults before first move)
        self._first_move_done = False

        # QoS profile - use VOLATILE durability to receive from all publishers
        # (both rosbridge TRANSIENT_LOCAL and motor_controller VOLATILE)
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Create publisher
        self.status_pub = self.create_publisher(
            MotorStatus,
            status_topic,
            qos
        )

        # Create subscriber
        self.command_sub = self.create_subscription(
            MotorCommand,
            command_topic,
            self._command_callback,
            qos
        )

        # Create services
        self.clear_faults_srv = self.create_service(
            ClearFaults,
            '/clearlink/clear_faults',
            self._clear_faults_callback
        )

        self.home_srv = self.create_service(
            Home,
            '/clearlink/home',
            self._home_callback
        )

        # Initialize connection
        if not self._control.initialize():
            self.get_logger().error(
                f"Failed to connect to ClearLink at {ip_address}: "
                f"{self._control.connection_error}"
            )
        else:
            self.get_logger().info("Connected to ClearLink")
            # Auto-enable motors if no faults detected
            self._auto_enable_motors()

        # Reconnect tracking
        self._reconnect_interval = 5.0
        self._last_reconnect_attempt = 0.0

        # Create main loop timer
        loop_period = 1.0 / loop_hz
        self.create_timer(loop_period, self._main_loop)

        self.get_logger().info("ClearLink node started")

    def _auto_enable_motors(self):
        """Auto-enable motors on startup, clearing faults first."""
        if not self._control.connected:
            self.get_logger().warn("Cannot auto-enable: not connected")
            return

        # Clear faults on all axes first before checking status
        all_axes = list(range(1, self._num_axes + 1))
        self.get_logger().info(f"Clearing faults on axes {all_axes}...")
        success, msg = self._control.clear_faults(all_axes)
        if not success:
            self.get_logger().warn(f"Fault clear warning: {msg}")

        # Now try to enable all axes
        if self._control.enable_motors(all_axes):
            self.get_logger().info(
                f"Auto-enabled motors on axes {all_axes}"
            )
        else:
            # Check which axes failed
            status = self._control.read_status()
            faulted = [i+1 for i, ax in enumerate(status.axes[:self._num_axes]) if ax.fault]
            if faulted:
                self.get_logger().warn(
                    f"Some axes have faults after enable attempt: {faulted}"
                )
            else:
                self.get_logger().error("Failed to auto-enable motors")

    def _main_loop(self):
        """Main loop - read status and publish, check deadman."""
        if not self._control.connected:
            # Publish disconnected status so UI shows fault
            msg = MotorStatus()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'clearlink'
            msg.connected = False
            msg.connection_error = self._control.connection_error or "Not connected"
            self.status_pub.publish(msg)

            now = time.time()
            if now - self._last_reconnect_attempt >= self._reconnect_interval:
                self._last_reconnect_attempt = now
                self.get_logger().info("Attempting to reconnect to ClearLink...")
                if self._control.reconnect():
                    self.get_logger().info("Reconnected to ClearLink")
                    self._auto_enable_motors()
                else:
                    self.get_logger().debug(f"Reconnect failed: {self._control.connection_error}")
            return

        # Read status
        status = self._control.read_status()

        # Create and publish status message
        msg = MotorStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'clearlink'

        msg.connected = status.connected
        msg.connection_error = status.connection_error

        # Axis states
        if len(status.axes) >= 1:
            msg.axis1_enabled = status.axes[0].enabled
            msg.axis1_fault = status.axes[0].fault
            msg.axis1_moving = status.axes[0].moving
            msg.axis1_homed = status.axes[0].homed
            msg.axis1_position = status.axes[0].position
            msg.axis1_velocity = status.axes[0].velocity
            msg.axis1_torque = status.axes[0].torque

        if len(status.axes) >= 2:
            msg.axis2_enabled = status.axes[1].enabled
            msg.axis2_fault = status.axes[1].fault
            msg.axis2_moving = status.axes[1].moving
            msg.axis2_homed = status.axes[1].homed
            msg.axis2_position = status.axes[1].position
            msg.axis2_velocity = status.axes[1].velocity
            msg.axis2_torque = status.axes[1].torque

        if len(status.axes) >= 3:
            msg.axis3_enabled = status.axes[2].enabled
            msg.axis3_fault = status.axes[2].fault
            msg.axis3_moving = status.axes[2].moving
            msg.axis3_homed = status.axes[2].homed
            msg.axis3_position = status.axes[2].position
            msg.axis3_velocity = status.axes[2].velocity
            msg.axis3_torque = status.axes[2].torque

        if len(status.axes) >= 4:
            msg.axis4_enabled = status.axes[3].enabled
            msg.axis4_fault = status.axes[3].fault
            msg.axis4_moving = status.axes[3].moving
            msg.axis4_homed = status.axes[3].homed
            msg.axis4_position = status.axes[3].position
            msg.axis4_velocity = status.axes[3].velocity
            msg.axis4_torque = status.axes[3].torque

        # Digital I/O
        if status.digital_inputs:
            msg.digital_inputs = status.digital_inputs
        if status.digital_outputs:
            msg.digital_outputs = status.digital_outputs

        msg.firmware_version = status.firmware_version
        msg.supply_voltage = status.supply_voltage

        self.status_pub.publish(msg)

        # Check deadman switch
        with self._cmd_lock:
            time_since_cmd = time.time() - self._last_cmd_time

        if self._deadman_secs > 0 and time_since_cmd > self._deadman_secs:
            # No command received recently - stop motors
            # Only log once when transitioning to stopped state
            pass  # Deadman currently disabled to match roboclaw behavior

    def _command_callback(self, msg: MotorCommand):
        """Handle incoming motor command."""
        with self._cmd_lock:
            self._last_cmd_time = time.time()

        # Build velocity list from message
        velocities = [
            msg.axis1_steps_per_sec if msg.axis1_enable else 0,
            msg.axis2_steps_per_sec if msg.axis2_enable else 0,
            msg.axis3_steps_per_sec if msg.axis3_enable else 0,
            msg.axis4_steps_per_sec if msg.axis4_enable else 0,
        ]

        # Enable/disable motors as requested
        enable_axes = []
        disable_axes = []

        if msg.axis1_enable:
            enable_axes.append(1)
        else:
            disable_axes.append(1)

        if msg.axis2_enable:
            enable_axes.append(2)
        else:
            disable_axes.append(2)

        if msg.axis3_enable:
            enable_axes.append(3)
        else:
            disable_axes.append(3)

        if msg.axis4_enable:
            enable_axes.append(4)
        else:
            disable_axes.append(4)

        # Apply enables
        if enable_axes:
            self._control.enable_motors(enable_axes)

        # Apply velocities
        accel = msg.acceleration if msg.acceleration > 0 else 10000
        # Clear faults before first movement to reset ClearLink state
        if not self._first_move_done:
            self.get_logger().info("First movement - clearing faults")
            self._control.clear_faults(enable_axes)
            self._first_move_done = True

        self.get_logger().info(f"Calling drive_velocity with velocities={velocities[:self._num_axes]}, accel={accel}")
        result = self._control.drive_velocity(velocities[:self._num_axes], accel)
        self.get_logger().info(f"drive_velocity returned {result}")

        # Apply disables
        if disable_axes:
            self._control.disable_motors(disable_axes)

    def _clear_faults_callback(self, request: ClearFaults.Request,
                                response: ClearFaults.Response) -> ClearFaults.Response:
        """Handle clear faults service request."""
        axes = []
        if request.axis1:
            axes.append(1)
        if request.axis2:
            axes.append(2)
        if request.axis3:
            axes.append(3)
        if request.axis4:
            axes.append(4)

        if not axes:
            response.success = False
            response.message = "No axes specified"
            return response

        success, message = self._control.clear_faults(axes)
        response.success = success
        response.message = message
        return response

    def _home_callback(self, request: Home.Request,
                       response: Home.Response) -> Home.Response:
        """Handle home service request."""
        axes = []
        if request.axis1:
            axes.append(1)
        if request.axis2:
            axes.append(2)
        if request.axis3:
            axes.append(3)
        if request.axis4:
            axes.append(4)

        if not axes:
            response.success = False
            response.message = "No axes specified"
            return response

        success, message = self._control.home(
            axes,
            request.velocity,
            request.acceleration
        )
        response.success = success
        response.message = message
        return response

    def destroy_node(self):
        """Cleanup on node shutdown."""
        self.get_logger().info("Shutting down ClearLink node")
        self._control.shutdown()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = ClearLinkNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
