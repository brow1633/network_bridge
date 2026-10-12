from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = get_package_share_directory("network_bridge")
    udp_demo_client_config = config_dir + "/demo/udp_demo_client.yaml"

    receiver_address = LaunchConfiguration("receiver_address", default="127.0.0.1")
    receiver_port = LaunchConfiguration("receiver_port", default="5001")
    server_host = LaunchConfiguration("server_host", default="127.0.0.1")
    server_port = LaunchConfiguration("server_port", default="5000")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "receiver_address",
                default_value="127.0.0.1",
                description="Address to listen on (use 0.0.0.0 to bind on any interface)",
            ),
            DeclareLaunchArgument(
                "receiver_port",
                default_value="5001",
                description="UDP port of the server",
            ),
            DeclareLaunchArgument(
                "server_host",
                default_value="127.0.0.1",
                description="Hostname of the server",
            ),
            DeclareLaunchArgument(
                "server_port",
                default_value="5000",
                description="UDP port of the server",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="client_talker",
                namespace="/udp_client",
                output="screen",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="listener",
                name="client_listener",
                namespace="/udp_server",
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="local_tf",
                arguments=[
                    "--frame-id",
                    "base_link",
                    "--child-frame-id",
                    "client_link",
                ],
                remappings=[("/tf_static", "/udp_client/tf_static")],
                # namespace="/udp_client", # Not honored on kilted
                output="screen",
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="udp_demo_client",
                output="screen",
                parameters=[
                    udp_demo_client_config,
                    {"UdpInterface.local_address": receiver_address},
                    {"UdpInterface.receive_port": receiver_port},
                    {"UdpInterface.remote_address": server_host},
                    {"UdpInterface.send_port": server_port},
                ],
            ),
        ]
    )
