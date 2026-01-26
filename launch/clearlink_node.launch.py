"""
ClearLink Driver Launch File

Launches the ClearLink motor controller node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.20.240',
        description='ClearLink IP address'
    )

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='44818',
        description='EtherNet/IP port'
    )

    num_axes_arg = DeclareLaunchArgument(
        'num_axes',
        default_value='4',
        description='Number of motor axes'
    )

    loop_hz_arg = DeclareLaunchArgument(
        'loop_hz',
        default_value='10.0',
        description='Status publish rate in Hz'
    )

    deadman_secs_arg = DeclareLaunchArgument(
        'deadman_secs',
        default_value='3.0',
        description='Deadman switch timeout (0 to disable)'
    )

    # Create node
    clearlink_node = Node(
        package='clearlink_driver',
        executable='clearlink_node',
        name='clearlink',
        parameters=[{
            'ip_address': LaunchConfiguration('ip_address'),
            'port': LaunchConfiguration('port'),
            'num_axes': LaunchConfiguration('num_axes'),
            'loop_hz': LaunchConfiguration('loop_hz'),
            'deadman_secs': LaunchConfiguration('deadman_secs'),
        }],
        output='screen'
    )

    return LaunchDescription([
        ip_address_arg,
        port_arg,
        num_axes_arg,
        loop_hz_arg,
        deadman_secs_arg,
        clearlink_node,
    ])
