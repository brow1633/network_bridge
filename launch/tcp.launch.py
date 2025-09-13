from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("network_bridge")
    tcp1_config = config_dir + "/config/Tcp1.yaml"
    tcp2_config = config_dir + "/config/Tcp2.yaml"

    return LaunchDescription(
        [
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="Tcp1",
                output="screen",
                parameters=[tcp1_config],
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="Tcp2",
                output="screen",
                parameters=[tcp2_config],
            ),
        ]
    )
