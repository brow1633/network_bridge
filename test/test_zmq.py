import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.asserts
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

    zmq1_push_pull = launch_ros.actions.Node(
        package="network_bridge",
        executable="network_bridge",
        name="zmq_bridge_server_push",
        output="screen",
        parameters=[config + "Zmq1PushPull.yaml"],
        arguments=["--ros-args", "--log-level", "debug", "--log-level", "rcl:=info"],
    )

    zmq2_push_pull = launch_ros.actions.Node(
        package="network_bridge",
        executable="network_bridge",
        name="zmq_bridge_client_pull",
        output="screen",
        parameters=[config + "Zmq2PushPull.yaml"],
        arguments=["--ros-args", "--log-level", "debug", "--log-level", "rcl:=info"],
    )

    return launch.LaunchDescription(
        [
            zmq1,
            launch.actions.TimerAction(period=0.1, actions=[zmq2]),
            launch.actions.TimerAction(period=0.2, actions=[zmq1_push_pull]),
            launch.actions.TimerAction(period=0.3, actions=[zmq2_push_pull]),
            launch_testing.actions.ReadyToTest(),
        ]
    )


class ZmqTestNode(Node):

    def __init__(self, name, pub_topic, sub_topic):
        super().__init__(name)
        self.test_message_received = Future()
        self.received_msg = None
        self.expected_data = None
        self.publisher = self.create_publisher(String, pub_topic, 10)
        self.subscriber = self.create_subscription(
            String, sub_topic, self.listener_callback, 10
        )

    def publish(self, msg):
        self.publisher.publish(msg)

    def listener_callback(self, msg):
        if msg.data == self.expected_data and not self.test_message_received.done():
            self.received_msg = msg
            self.test_message_received.set_result(True)


class TestZmq(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def check_delivery(self, name, pub_topic, sub_topic):
        node = ZmqTestNode(name, pub_topic, sub_topic)
        try:
            # Warm up until both DDS discovery and the ZMQ connection deliver.
            node.expected_data = "warmup"
            warmup = String(data=node.expected_data)
            deadline = time.monotonic() + 15.0
            while not node.test_message_received.done() and time.monotonic() < deadline:
                node.publish(warmup)
                rclpy.spin_once(node, timeout_sec=0.1)
            self.assertTrue(
                node.test_message_received.done(), "Bridge never became ready"
            )

            # Send the actual assertion sample once; retries must not hide loss.
            node.expected_data = "Testing123:" + name
            node.test_message_received = Future()
            node.publish(String(data=node.expected_data))
            rclpy.spin_until_future_complete(
                node, node.test_message_received, timeout_sec=5.0
            )
            self.assertTrue(
                node.test_message_received.done(), "Test sample was not delivered"
            )
            self.assertEqual(node.received_msg.data, node.expected_data)
        finally:
            node.destroy_node()

    def test_pub_sub(self):
        self.check_delivery("zmq_test_node", "/zmq1/test_topic", "/zmq2/test_topic")

    def test_push_pull(self):
        self.check_delivery(
            "zmq_push_pull_test_node",
            "/zmq1/push_pull/test_topic",
            "/zmq2/push_pull/test_topic",
        )


@launch_testing.post_shutdown_test()
class TestZmqShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info):
        launch_testing.asserts.assertExitCodes(proc_info)


if __name__ == "__main__":
    launch_testing.main()
