from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = get_package_share_directory("network_bridge")
    zmq1_config = config_dir + "/config/Zmq1.yaml"
    zmq2_config = config_dir + "/config/Zmq2.yaml"

    return LaunchDescription(
        [
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="zmq_bridge_server",
                output="screen",
                parameters=[zmq1_config],
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="zmq_bridge_client",
                output="screen",
                parameters=[zmq2_config],
            ),
        ]
    )
