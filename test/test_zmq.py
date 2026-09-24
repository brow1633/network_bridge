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

    zmq1_push_pull = launch_ros.actions.Node(
        package="network_bridge",
        executable="network_bridge",
        name="zmq_bridge_server_push",
        output="screen",
        parameters=[config + "Zmq1PushPull.yaml"],
        arguments=["--ros-args", "--log-level", "debug", "--log-level", "rcl:=info"]
    )

    zmq2_push_pull = launch_ros.actions.Node(
        package="network_bridge",
        executable="network_bridge",
        name="zmq_bridge_client_pull",
        output="screen",
        parameters=[config + "Zmq2PushPull.yaml"],
        arguments=["--ros-args", "--log-level", "debug", "--log-level", "rcl:=info"]
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
        self.publisher = self.create_publisher(String, pub_topic, 10)
        self.subscriber = self.create_subscription(
            String, sub_topic, self.listener_callback, 10
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

        node = ZmqTestNode(
            "zmq_test_node",
            "/zmq1/test_topic",
            "/zmq2/test_topic")
        time.sleep(1.5)

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

        node_push_pull = ZmqTestNode(
            "zmq_push_pull_test_node",
            "/zmq1/push_pull/test_topic",
            "/zmq2/push_pull/test_topic")
        time.sleep(1.5)

        node_push_pull.publish(test_msg)

        try:
            rclpy.spin_until_future_complete(
                node_push_pull,
                node_push_pull.test_message_received,
                timeout_sec=10.0
            )
            self.assertTrue(
                node_push_pull.test_message_received.done(), "Timeout on message receival."
            )
            self.assertEqual(
                node_push_pull.received_msg.data,
                "Testing123",
                "The received message did not match the expected output.",
            )
        finally:
            node_push_pull.destroy_node()

if __name__ == "__main__":
    launch_testing.main()
