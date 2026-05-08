#!/usr/bin/env python3
import time

import rclpy
from marvin_msgs.srv import Int
from rclpy.node import Node
from std_msgs.msg import Bool
from std_msgs.msg import Int16MultiArray
from std_srvs.srv import Trigger


class RobotModeInitializer(Node):
    def __init__(self) -> None:
        super().__init__("robot_mode_initializer")

        self.declare_parameter("ready_service", "/control/set_ready")
        self.declare_parameter("mode_service", "/control/set_mode")
        self.declare_parameter("desired_mode", 3)
        self.declare_parameter("max_retries", 0)
        self.declare_parameter("retry_interval_sec", 1.0)
        self.declare_parameter("call_timeout_sec", 3.0)
        self.declare_parameter("wait_for_arm_state", True)
        self.declare_parameter("arm_state_topic", "/info/arm_state")
        self.declare_parameter("arm_state_timeout_sec", 20.0)
        self.declare_parameter("publish_grip_on_success", True)
        self.declare_parameter("grip_topic_left", "/control/gripL")
        self.declare_parameter("grip_topic_right", "/control/gripR")
        self.declare_parameter("grip_publish_count", 3)
        self.declare_parameter("grip_publish_interval_sec", 0.1)

        self.ready_service = str(self.get_parameter("ready_service").value)
        self.mode_service = str(self.get_parameter("mode_service").value)
        self.desired_mode = int(self.get_parameter("desired_mode").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.retry_interval_sec = float(self.get_parameter("retry_interval_sec").value)
        self.call_timeout_sec = float(self.get_parameter("call_timeout_sec").value)
        self.wait_for_arm_state = bool(self.get_parameter("wait_for_arm_state").value)
        self.arm_state_topic = str(self.get_parameter("arm_state_topic").value)
        self.arm_state_timeout_sec = float(self.get_parameter("arm_state_timeout_sec").value)
        self.publish_grip_on_success = bool(self.get_parameter("publish_grip_on_success").value)
        self.grip_topic_left = str(self.get_parameter("grip_topic_left").value)
        self.grip_topic_right = str(self.get_parameter("grip_topic_right").value)
        self.grip_publish_count = int(self.get_parameter("grip_publish_count").value)
        self.grip_publish_interval_sec = float(self.get_parameter("grip_publish_interval_sec").value)

        self._arm_state_ok = False
        self._arm_state_sub = self.create_subscription(
            Int16MultiArray, self.arm_state_topic, self._arm_state_cb, 10
        )
        self._ready_client = self.create_client(Trigger, self.ready_service)
        self._mode_client = self.create_client(Int, self.mode_service)
        self._grip_left_pub = self.create_publisher(Bool, self.grip_topic_left, 10)
        self._grip_right_pub = self.create_publisher(Bool, self.grip_topic_right, 10)

    def _arm_state_cb(self, msg: Int16MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self._arm_state_ok = (msg.data[0] >= 0) and (msg.data[1] >= 0)

    def _wait_for_service(self, client, name: str) -> None:
        while rclpy.ok():
            if client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Service ready: {name}")
                return
            self.get_logger().info(f"Waiting for service: {name}")

    def _wait_for_arm_state(self) -> None:
        if not self.wait_for_arm_state:
            return
        deadline = time.time() + self.arm_state_timeout_sec
        while rclpy.ok() and time.time() < deadline and not self._arm_state_ok:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self._arm_state_ok:
            self.get_logger().info("Arm state is ready.")
        else:
            self.get_logger().warn(
                "Arm state readiness timeout; continue init and rely on retries."
            )

    def _call_ready(self) -> bool:
        future = self._ready_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout_sec)
        if not future.done() or future.result() is None:
            self.get_logger().warn("set_ready call timed out or returned no result.")
            return False
        result = future.result()
        if not result.success:
            self.get_logger().warn(f"set_ready failed: {result.message}")
            return False
        self.get_logger().info(f"set_ready success: {result.message}")
        return True

    def _call_mode(self) -> bool:
        req = Int.Request()
        req.data = self.desired_mode
        future = self._mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.call_timeout_sec)
        if not future.done() or future.result() is None:
            self.get_logger().warn("set_mode call timed out or returned no result.")
            return False
        result = future.result()
        if not result.success:
            self.get_logger().warn(f"set_mode({self.desired_mode}) failed: {result.message}")
            return False
        self.get_logger().info(f"set_mode({self.desired_mode}) success: {result.message}")
        return True

    def _publish_grip(self) -> None:
        if not self.publish_grip_on_success:
            return
        msg = Bool()
        msg.data = True
        for _ in range(max(1, self.grip_publish_count)):
            self._grip_left_pub.publish(msg)
            self._grip_right_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(max(0.0, self.grip_publish_interval_sec))
        self.get_logger().info("Published grip enable messages.")

    def run(self) -> int:
        self._wait_for_service(self._ready_client, self.ready_service)
        self._wait_for_service(self._mode_client, self.mode_service)
        self._wait_for_arm_state()

        attempt = 0
        while rclpy.ok():
            attempt += 1
            self.get_logger().info(f"Robot init attempt {attempt}")
            ok_ready = self._call_ready()
            ok_mode = self._call_mode() if ok_ready else False
            if ok_ready and ok_mode:
                self._publish_grip()
                self.get_logger().info("Robot init completed.")
                return 0

            if self.max_retries > 0 and attempt >= self.max_retries:
                self.get_logger().error("Robot init failed: reached max retries.")
                return 1

            time.sleep(max(0.0, self.retry_interval_sec))

        return 1


def main() -> None:
    rclpy.init()
    node = RobotModeInitializer()
    code = 1
    try:
        code = node.run()
    except KeyboardInterrupt:
        code = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
