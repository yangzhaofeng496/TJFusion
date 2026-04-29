#!/usr/bin/env python3
import re
from typing import Dict, Set

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool
from visualization_msgs.msg import InteractiveMarkerFeedback, InteractiveMarkerUpdate


def _guess_side(marker_name: str) -> str:
    name = marker_name.lower()
    if re.search(r"(left|left_tool|_l\b|\bl_)", name):
        return "left"
    if re.search(r"(right|right_tool|_r\b|\br_)", name):
        return "right"
    return "unknown"


class MoveItGoalBridge(Node):
    def __init__(self) -> None:
        super().__init__("moveit_goal_bridge")
        self.declare_parameter(
            "feedback_topic",
            "",
        )
        self.declare_parameter("update_topic", "")
        self.declare_parameter("default_side", "right")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("output_frame_id", "base_link")

        self.feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.update_topic = str(self.get_parameter("update_topic").value)
        self.default_side = str(self.get_parameter("default_side").value).lower()
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)

        self.left_pub = self.create_publisher(PoseStamped, "control/target_poseL", 10)
        self.right_pub = self.create_publisher(PoseStamped, "control/target_poseR", 10)
        self.left_grip_pub = self.create_publisher(Bool, "control/gripL", 10)
        self.right_grip_pub = self.create_publisher(Bool, "control/gripR", 10)

        self.left_pose = None
        self.right_pose = None
        self._subs: Dict[str, object] = {}
        self._log_once: Set[str] = set()

        if self.feedback_topic:
            self._subscribe_feedback(self.feedback_topic)
        if self.update_topic:
            self._subscribe_update(self.update_topic)
        # Auto discover MoveIt interactive marker topics if explicit topics are not set.
        self.discovery_timer = self.create_timer(1.0, self._discover_topics)

        dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(dt, self._tick)
        self.get_logger().info(
            "MoveIt bridge started. "
            f"default_side={self.default_side}, explicit_feedback='{self.feedback_topic}', "
            f"explicit_update='{self.update_topic}'"
        )

    def _feedback_cb(self, msg: InteractiveMarkerFeedback) -> None:
        self._apply_pose(msg.marker_name, msg.header.frame_id, msg.pose)

    def _update_cb(self, msg: InteractiveMarkerUpdate) -> None:
        for p in msg.poses:
            self._apply_pose(p.name, p.header.frame_id, p.pose)

    def _apply_pose(self, marker_name: str, frame_id: str, pose_raw) -> None:
        pose = PoseStamped()
        pose.header.frame_id = frame_id if frame_id else self.output_frame_id
        pose.pose = pose_raw
        side = _guess_side(marker_name)
        if side == "unknown":
            side = self.default_side
            key = f"unknown::{marker_name}"
            if key not in self._log_once:
                self.get_logger().warn(
                    f"Unknown marker_name='{marker_name}', fallback to default_side='{self.default_side}'"
                )
                self._log_once.add(key)

        if side == "left":
            self.left_pose = pose
        else:
            self.right_pose = pose

    def _subscribe_feedback(self, topic: str) -> None:
        if topic in self._subs:
            return
        sub = self.create_subscription(InteractiveMarkerFeedback, topic, self._feedback_cb, 10)
        self._subs[topic] = sub
        self.get_logger().info(f"Subscribed feedback: {topic}")

    def _subscribe_update(self, topic: str) -> None:
        if topic in self._subs:
            return
        sub = self.create_subscription(InteractiveMarkerUpdate, topic, self._update_cb, 10)
        self._subs[topic] = sub
        self.get_logger().info(f"Subscribed update: {topic}")

    def _discover_topics(self) -> None:
        topic_map = dict(self.get_topic_names_and_types())
        for name, types in topic_map.items():
            if name in self._subs:
                continue
            if (
                name.endswith("robot_interaction_interactive_marker_topic/feedback")
                and "visualization_msgs/msg/InteractiveMarkerFeedback" in types
            ):
                self._subscribe_feedback(name)
            if (
                name.endswith("robot_interaction_interactive_marker_topic/update")
                and "visualization_msgs/msg/InteractiveMarkerUpdate" in types
            ):
                self._subscribe_update(name)

    def _tick(self) -> None:
        on = Bool()
        on.data = True
        self.left_grip_pub.publish(on)
        self.right_grip_pub.publish(on)

        if self.left_pose is not None:
            self.left_pose.header.stamp = self.get_clock().now().to_msg()
            self.left_pub.publish(self.left_pose)
        if self.right_pose is not None:
            self.right_pose.header.stamp = self.get_clock().now().to_msg()
            self.right_pub.publish(self.right_pose)


def main() -> None:
    rclpy.init()
    node = MoveItGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
