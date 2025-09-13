from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("network_bridge")
    udp1_config = config_dir + "/config/Udp1.yaml"
    udp2_config = config_dir + "/config/Udp2.yaml"

    return LaunchDescription(
        [
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="Udp1",
                output="screen",
                parameters=[udp1_config],
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="Udp2",
                output="screen",
                parameters=[udp2_config],
            ),
        ]
    )
