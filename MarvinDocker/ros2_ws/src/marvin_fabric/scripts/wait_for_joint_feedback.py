#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node

from marvin_msgs.msg import Jointfeedback


class WaitForJointFeedback(Node):
    def __init__(self) -> None:
        super().__init__("wait_for_joint_feedback")
        self.declare_parameter("topic", "/info/joint_feedback")
        self.declare_parameter("timeout_sec", 20.0)

        self.topic = str(self.get_parameter("topic").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.got_message = False

        self.sub = self.create_subscription(
            Jointfeedback,
            self.topic,
            self._cb,
            10,
        )

    def _cb(self, msg: Jointfeedback) -> None:
        # First valid feedback is enough to release MoveIt startup.
        if len(msg.positions) >= 14:
            self.got_message = True


def main() -> None:
    rclpy.init()
    node = WaitForJointFeedback()
    node.get_logger().info(
        f"Waiting for first Jointfeedback on {node.topic} (timeout {node.timeout_sec:.1f}s)..."
    )

    deadline = time.monotonic() + max(0.1, node.timeout_sec)
    while rclpy.ok() and time.monotonic() < deadline and not node.got_message:
        rclpy.spin_once(node, timeout_sec=0.2)

    if node.got_message:
        node.get_logger().info("Received first real joint feedback, continue startup.")
    else:
        node.get_logger().warn(
            "Timeout waiting joint feedback. Continue startup anyway."
        )

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
