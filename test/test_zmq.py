import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch_ros.actions
import launch_testing
import rclpy
from rclpy.node import Node
from rclpy.task import Future

from std_msgs.msg import String


def generate_test_description():
    config = get_package_share_directory("network_bridge") + "/config/"
    zmq1 = launch_ros.actions.Node(
        package="network_bridge",
        executable="network_bridge",
        name="zmq_bridge_server",
        output="screen",
        parameters=[config + "Zmq1.yaml"],
        arguments=["--ros-args", "--log-level", "debug", "--log-level", "rcl:=info"],
    )

    zmq2 = launch_ros.actions.Node(
        package="network_bridge",
        executable="network_bridge",
        name="zmq_bridge_client",
        output="screen",
        parameters=[config + "Zmq2.yaml"],
        arguments=["--ros-args", "--log-level", "debug", "--log-level", "rcl:=info"],
    )

    return launch.LaunchDescription(
        [
            zmq1,
            launch.actions.TimerAction(period=0.1, actions=[zmq2]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class ZmqTestNode(Node):

    def __init__(self):
        super().__init__("test_node")
        self.test_message_received = Future()
        self.received_msg = None
        self.publisher = self.create_publisher(String, "/zmq1/test_topic", 10)
        self.subscriber = self.create_subscription(
            String, "/zmq2/test_topic", self.listener_callback, 10
        )

    def publish(self, msg):
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        self.received_msg = msg
        self.test_message_received.set_result(True)


class TestZmq(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_output(self, proc_output):
        proc_output.assertWaitFor("Server bound", timeout=0.5)
        proc_output.assertWaitFor("Client connected", timeout=0.5)

        node = ZmqTestNode()
        time.sleep(0.15)

        test_msg = String()
        test_msg.data = "Testing123"
        node.publish(test_msg)

        try:
            rclpy.spin_until_future_complete(
                node, node.test_message_received, timeout_sec=10.0
            )
            self.assertTrue(
                node.test_message_received.done(), "Timeout on message receival."
            )
            self.assertEqual(
                node.received_msg.data,
                "Testing123",
                "The received message did not match the expected output.",
            )
        finally:
            node.destroy_node()


if __name__ == "__main__":
    launch_testing.main()
