from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = get_package_share_directory("network_bridge")
    tcp_demo_client_config = config_dir + "/demo/tcp_demo_client.yaml"

    server_host = LaunchConfiguration("server_host", default="127.0.0.1")
    server_port = LaunchConfiguration("server_port", default="5000")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "server_host",
                default_value="127.0.0.1",
                description="Hostname of the server",
            ),
            DeclareLaunchArgument(
                "server_port",
                default_value="5000",
                description="TCP port of the server",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="client_talker",
                namespace="/tcp_client",
                output="screen",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="listener",
                name="client_listener",
                namespace="/tcp_server",
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
                remappings=[("/tf_static", "/tcp_client/tf_static")],
                # namespace="/tcp_client", # Not honored on kilted
                output="screen",
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="tcp_demo_client",
                output="screen",
                parameters=[
                    tcp_demo_client_config,
                    {"TcpInterface.remote_address": server_host},
                    {"TcpInterface.port": server_port},
                ],
            ),
        ]
    )
