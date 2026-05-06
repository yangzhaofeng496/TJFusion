#!/usr/bin/env python3

from __future__ import annotations

import json
import time
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import zmq
from zmq.error import Again, ZMQError


SUPPORTED_JOINT_ORDER = [
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
]


class MujocoStateBridge(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_state_bridge")

        self.declare_parameter("endpoint", "tcp://127.0.0.1:5555")
        self.declare_parameter("timeout_ms", 2000)
        self.declare_parameter("state_rate_hz", 50.0)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("zmq_topic", "")
        self.declare_parameter("publish_velocity", False)
        self.declare_parameter("log_on_receive", True)
        self.declare_parameter("log_raw_zmq", False)

        self.endpoint = str(self.get_parameter("endpoint").value)
        self.timeout_ms = int(self.get_parameter("timeout_ms").value)
        self.state_rate_hz = float(self.get_parameter("state_rate_hz").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.zmq_topic = str(self.get_parameter("zmq_topic").value)
        self.publish_velocity = bool(self.get_parameter("publish_velocity").value)
        self.log_on_receive = bool(self.get_parameter("log_on_receive").value)
        self.log_raw_zmq = bool(self.get_parameter("log_raw_zmq").value)

        if self.state_rate_hz <= 0.0:
            self.get_logger().warning("state_rate_hz <= 0, forcing to 10.0 Hz")
            self.state_rate_hz = 10.0

        self._zmq_context = zmq.Context.instance()
        self._sub = self._create_subscriber()
        self._joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self._last_warn_t = 0.0
        self._warn_interval_sec = 2.0

        self._timer = self.create_timer(1.0 / self.state_rate_hz, self._on_timer)
        self.get_logger().info(
            f"State bridge ready: endpoint={self.endpoint}, zmq_topic={self.zmq_topic or '<all>'}, "
            f"ros_topic={self.joint_state_topic}, rate={self.state_rate_hz}Hz, timeout_ms={self.timeout_ms}, "
            f"publish_velocity={self.publish_velocity}, log_on_receive={self.log_on_receive}, "
            f"log_raw_zmq={self.log_raw_zmq}"
        )

    def _warn_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self._last_warn_t >= self._warn_interval_sec:
            self._last_warn_t = now
            self.get_logger().warning(text)

    def _create_subscriber(self):
        sock = self._zmq_context.socket(zmq.SUB)
        sock.setsockopt(zmq.LINGER, 0)
        sock.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        sock.setsockopt(zmq.RCVHWM, 1)
        # Do not use CONFLATE here: remote side may publish multipart messages,
        # and CONFLATE with multipart can trigger libzmq assertions.
        sock.setsockopt(zmq.SUBSCRIBE, self.zmq_topic.encode("utf-8"))
        sock.connect(self.endpoint)
        return sock

    @staticmethod
    def _decode_frame(frame: bytes) -> str:
        return frame.decode("utf-8", errors="replace")

    def _parse_message(self, frames: List[bytes]) -> Tuple[Optional[str], Any]:
        if len(frames) == 1:
            text = self._decode_frame(frames[0]).strip()
            if text.startswith("[") and "]" in text and "{" in text:
                text = text[text.find("]") + 1 :].strip()
            return None, json.loads(text)

        topic = self._decode_frame(frames[0]).strip()
        payload = self._decode_frame(frames[-1]).strip()
        if payload.startswith("[") and "]" in payload and "{" in payload:
            payload = payload[payload.find("]") + 1 :].strip()
        return topic, json.loads(payload)

    def _recv_latest_state(self):
        latest_topic = None
        latest_state = None
        while True:
            try:
                parts = self._sub.recv_multipart(flags=zmq.NOBLOCK)
                if not parts:
                    continue
                if self.log_raw_zmq:
                    frame_preview = " | ".join(
                        self._decode_frame(frame)[:160].replace("\n", "\\n") for frame in parts
                    )
                    self.get_logger().info(
                        f"Raw ZMQ message received: frames={len(parts)} payload={frame_preview}"
                    )
                topic, msg = self._parse_message(parts)
                if isinstance(msg, dict):
                    latest_topic = topic
                    latest_state = msg
            except Again:
                break
            except ZMQError as exc:
                self._warn_throttled(f"ZMQ SUB receive failed: {exc}")
                break
            except Exception as exc:  # pylint: disable=broad-except
                raw_preview = ""
                if 'parts' in locals() and parts:
                    try:
                        raw_preview = " | ".join(
                            self._decode_frame(frame)[:160].replace("\n", "\\n") for frame in parts
                        )
                    except Exception:  # pylint: disable=broad-except
                        raw_preview = "<decode failed>"
                self._warn_throttled(
                    f"Unexpected SUB receive/parse error: {exc}"
                    + (f", raw={raw_preview}" if raw_preview else "")
                )
                break
        return latest_topic, latest_state

    @staticmethod
    def _pick_joint_names(qpos: Dict) -> List[str]:
        names = [name for name in SUPPORTED_JOINT_ORDER if name in qpos]
        if names:
            return names
        return [k for k, v in qpos.items() if isinstance(v, (int, float))]

    def _on_timer(self) -> None:
        topic, state = self._recv_latest_state()
        if state is None:
            return

        if state.get("ok") is False:
            self._warn_throttled(f"MuJoCo state returned ok=false: {state}")
            return

        qpos = state.get("qpos")
        if not isinstance(qpos, dict):
            self._warn_throttled("MuJoCo state has no valid qpos field.")
            return

        names = self._pick_joint_names(qpos)
        if not names:
            self._warn_throttled("MuJoCo qpos has no usable joint values.")
            return

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(qpos[name]) for name in names]

        if self.publish_velocity:
            qvel = state.get("qvel", {})
            if isinstance(qvel, dict):
                msg.velocity = [float(qvel.get(name, 0.0)) for name in names]

        if self.log_on_receive:
            preview = ", ".join(f"{name}={qpos[name]:.3f}" for name in names[:4])
            self.get_logger().info(
                f"Received MuJoCo state: topic={topic or '<none>'}, joints={len(names)}"
                + (f", {preview}" if preview else "")
            )

        self._joint_state_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoStateBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
