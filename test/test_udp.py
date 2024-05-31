import sys
import unittest
import launch
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
import rclpy
import time
from rclpy.node import Node
from launch_testing.asserts import assertInStdout
import launch_testing
from std_msgs.msg import String

from rclpy.task import Future

def generate_test_description():
    config = get_package_share_directory('ros2_tether') + '/config/'
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='ros2_tether',
            executable='ros2_tether',
            name='udp1',
            output='screen',
            parameters=[config + 'Udp1.yaml'],
            arguments=['--ros-args', '--log-level', 'debug', '--log-level', 'rcl:=info'],
        ),
        launch_ros.actions.Node(
            package='ros2_tether',
            executable='ros2_tether',
            name='udp2',
            output='screen',
            parameters=[config + 'Udp2.yaml'],
            arguments=['--ros-args', '--log-level', 'debug', '--log-level', 'rcl:=info'],
        ),
        launch_testing.actions.ReadyToTest()
    ])

class UdpTestNode(Node):
    def __init__(self):
        super().__init__('test_node')
        self.test_message_received = Future()
        self.received_msg = None
        self.publisher = self.create_publisher(String, '/MyDefaultTopic', 10)
        self.subscriber = self.create_subscription(String, '/udp2/MyDefaultTopic', self.listener_callback, 10)
 
    def publish(self, msg):
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        self.received_msg = msg
        self.test_message_received.set_result(True)

class TestUdp(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_output(self, proc_output):
        proc_output.assertWaitFor('Setting up local endpoint', timeout=0.5)
        proc_output.assertWaitFor('Setting up local endpoint', timeout=0.5)


        node = UdpTestNode()
        time.sleep(0.15)

        test_msg = String()
        test_msg.data = "Testing123"
        node.publish(test_msg)

        try:
            rclpy.spin_until_future_complete(node, node.test_message_received, timeout_sec=10.0)
            self.assertTrue(node.test_message_received.done(), "Timeout on message receival.")
            self.assertEqual(node.received_msg.data, "Testing123", "The received message did not match the expected output.")
        finally:
            node.destroy_node()

if __name__ == '__main__':
    import launch_testing
    launch_testing.main()
