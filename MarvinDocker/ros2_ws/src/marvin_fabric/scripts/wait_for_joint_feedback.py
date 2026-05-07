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
        self.declare_parameter("log_interval_sec", 2.0)

        self.topic = str(self.get_parameter("topic").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.log_interval_sec = max(0.5, float(self.get_parameter("log_interval_sec").value))
        self.got_message = False
        self.invalid_feedback_count = 0

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
            return
        self.invalid_feedback_count += 1


def main() -> None:
    rclpy.init()
    node = WaitForJointFeedback()
    node.get_logger().info(
        f"Waiting for first Jointfeedback on {node.topic} (timeout {node.timeout_sec:.1f}s)..."
    )

    deadline = time.monotonic() + max(0.1, node.timeout_sec)
    start_time = time.monotonic()
    next_log_time = start_time + node.log_interval_sec
    while rclpy.ok() and time.monotonic() < deadline and not node.got_message:
        rclpy.spin_once(node, timeout_sec=0.2)
        now = time.monotonic()
        if now >= next_log_time and not node.got_message:
            elapsed = now - start_time
            remaining = max(0.0, deadline - now)
            node.get_logger().info(
                f"Still waiting on {node.topic}: elapsed={elapsed:.1f}s, remaining={remaining:.1f}s, "
                f"invalid_msgs={node.invalid_feedback_count}"
            )
            next_log_time = now + node.log_interval_sec

    if node.got_message:
        node.get_logger().info("Received first real joint feedback, continue startup.")
        exit_code = 0
    else:
        node.get_logger().warn(
            f"Timeout waiting joint feedback on {node.topic} after {node.timeout_sec:.1f}s "
            f"(invalid_msgs={node.invalid_feedback_count})."
        )
        exit_code = 1

    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()
