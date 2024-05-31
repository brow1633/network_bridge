import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_dir = get_package_share_directory('ros2_tether')
    Tcp1 = config_dir + '/config/Tcp1.yaml'
    Tcp2 = config_dir + '/config/Tcp2.yaml'
    return LaunchDescription([
        Node(
            package='ros2_tether',
            executable='ros2_tether',
            name='Tcp1',
            output='screen',
            parameters=[Tcp1],
        ),
        Node(
            package='ros2_tether',
            executable='ros2_tether',
            name='Tcp2',
            output='screen',
            parameters=[Tcp2],
        )
    ])