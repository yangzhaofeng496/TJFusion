#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import zmq
from zmq.error import ZMQError


SUPPORTED_JOINTS = {
    "Joint1_L",
    "Joint2_L",
    "Joint3_L",
    "Joint4_L",
    "Joint5_L",
    "Joint6_L",
    "Joint7_L",
    "Joint1_R",
    "Joint2_R",
    "Joint3_R",
    "Joint4_R",
    "Joint5_R",
    "Joint6_R",
    "Joint7_R",
}


class MarvinMujocoBridge(Node):
    def __init__(self) -> None:
        super().__init__("marvin_mujoco_bridge")

        self.declare_parameter("endpoint", "tcp://127.0.0.1:5555")
        self.declare_parameter("topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("timeout_ms", 2000)
        self.declare_parameter("relative", False)

        self.endpoint = str(self.get_parameter("endpoint").value)
        self.topic = str(self.get_parameter("topic").value)
        self.timeout_ms = int(self.get_parameter("timeout_ms").value)
        self.relative = bool(self.get_parameter("relative").value)

        self._zmq_context = zmq.Context.instance()

        self._sub = self.create_subscription(
            JointTrajectory, self.topic, self._on_trajectory, 10
        )

        self.get_logger().info(
            f"Bridge ready: topic={self.topic}, endpoint={self.endpoint}, timeout_ms={self.timeout_ms}, "
            f"relative={self.relative}"
        )

    @staticmethod
    def _duration_to_sec(msg_duration) -> float:
        return float(msg_duration.sec) + float(msg_duration.nanosec) * 1e-9

    def _send_request(self, payload: Dict) -> Tuple[bool, Optional[Dict]]:
        socket = self._zmq_context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        socket.connect(self.endpoint)

        try:
            socket.send_json(payload)
            reply = socket.recv_json()
            return True, reply if isinstance(reply, dict) else {"raw_reply": reply}
        except ZMQError as exc:
            self.get_logger().warning(f"ZMQ request failed: {exc}")
            return False, None
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f"Unexpected ZMQ error: {exc}")
            return False, None
        finally:
            socket.close(0)

    def _on_trajectory(self, msg: JointTrajectory) -> None:
        if not msg.points:
            self.get_logger().warning("Received empty JointTrajectory, skipping.")
            return

        if not msg.joint_names:
            self.get_logger().warning("JointTrajectory has no joint_names, skipping.")
            return

        name_to_index = {name: idx for idx, name in enumerate(msg.joint_names)}
        supported_in_msg = [name for name in msg.joint_names if name in SUPPORTED_JOINTS]
        unknown = [name for name in msg.joint_names if name not in SUPPORTED_JOINTS]

        if unknown:
            self.get_logger().warning(f"Ignoring unsupported joints: {unknown}")
        if not supported_in_msg:
            self.get_logger().warning("No supported MuJoCo joints found in trajectory, skipping.")
            return

        prev_time = 0.0
        for point_idx, point in enumerate(msg.points):
            current_time = self._duration_to_sec(point.time_from_start)
            duration = current_time - prev_time
            prev_time = current_time

            if duration < 0.0:
                self.get_logger().warning(
                    f"Point {point_idx} has non-monotonic time_from_start, clamping duration to 0.0."
                )
                duration = 0.0

            positions: Dict[str, float] = {}
            for joint_name in supported_in_msg:
                idx = name_to_index[joint_name]
                if idx >= len(point.positions):
                    self.get_logger().warning(
                        f"Point {point_idx} missing position for {joint_name}, skipping this joint."
                    )
                    continue
                positions[joint_name] = float(point.positions[idx])

            if not positions:
                self.get_logger().warning(f"Point {point_idx} has no valid positions, skipping point.")
                continue

            request = {
                "cmd": "set_positions",
                "positions": positions,
                "duration": float(duration),
                "relative": self.relative,
            }
            ok, reply = self._send_request(request)
            if ok and reply is not None and reply.get("ok") is False:
                self.get_logger().warning(f"MuJoCo rejected point {point_idx}: {reply}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MarvinMujocoBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
