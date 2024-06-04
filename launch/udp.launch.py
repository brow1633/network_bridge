from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_dir = get_package_share_directory('ros2_tether')
    udp1_config = config_dir + '/config/Udp1.yaml'
    udp2_config = config_dir + '/config/Udp2.yaml'

    return LaunchDescription([
        Node(
            package='ros2_tether',
            executable='ros2_tether',
            name='Udp1',
            output='screen',
            parameters=[udp1_config],
        ),
        Node(
            package='ros2_tether',
            executable='ros2_tether',
            name='Udp2',
            output='screen',
            parameters=[udp2_config],
        )
    ])
