from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    config_dir = get_package_share_directory("network_bridge")
    demo_server_config = config_dir + "/demo/demo_server.yaml"

    udp_receiver_host = LaunchConfiguration("udp_receiver_host", default="127.0.0.1")
    udp_receiver_port = LaunchConfiguration("udp_receiver_port", default="5001")
    udp_server_address = LaunchConfiguration("udp_server_address", default="127.0.0.1")
    udp_server_port = LaunchConfiguration("udp_server_port", default="5000")
    tcp_server_port = LaunchConfiguration("tcp_server_port", default="5000")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "udp_receiver_host",
                default_value="127.0.0.1",
                description="Hostname of the UDP remote host",
            ),
            DeclareLaunchArgument(
                "udp_receiver_port",
                default_value="5001",
                description="UDP port of the remote host",
            ),
            DeclareLaunchArgument(
                "udp_server_address",
                default_value="127.0.0.1",
                description="Address to listen to for UDP communication (use 0.0.0.0 to bind on any interface)",
            ),
            DeclareLaunchArgument(
                "udp_server_port",
                default_value="5000",
                description="Port to listen to for UDP communication",
            ),
            DeclareLaunchArgument(
                "tcp_server_port",
                default_value="5000",
                description="Port to listen to for TCP communication",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="talker",
                name="server_talker",
                namespace="/server",
                output="screen",
            ),
            Node(
                package="demo_nodes_cpp",
                executable="listener",
                name="server_listener",
                namespace="/client",
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
                    "server_link",
                ],
                remappings=[("/tf_static", "/server/tf_static")],
                # namespace="/server", # Not honored on kilted
                output="screen",
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="tcp_demo_server",
                output="screen",
                parameters=[
                    demo_server_config,
                    {"network_interface": "network_bridge::TcpInterface"},
                    {"TcpInterface.port": tcp_server_port},
                ],
            ),
            Node(
                package="network_bridge",
                executable="network_bridge",
                name="udp_demo_server",
                output="screen",
                parameters=[
                    demo_server_config,
                    {"network_interface": "network_bridge::UdpInterface"},
                    {"UdpInterface.local_address": udp_server_address},
                    {"UdpInterface.receive_port": udp_server_port},
                    {"UdpInterface.remote_address": udp_receiver_host},
                    {"UdpInterface.send_port": udp_receiver_port},
                ],
            ),
        ]
    )
