from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rm_serial_driver',
            executable='send_packet_node',
            output='screen'
        )
    ])
