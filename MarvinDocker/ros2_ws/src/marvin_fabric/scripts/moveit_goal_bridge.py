#!/usr/bin/env python3
import re
from typing import Dict, Set

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
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
        self.declare_parameter("publish_on_execute_only", True)
        self.declare_parameter("publish_preview_on_plan", True)
        self.declare_parameter("publish_preview_always", True)
        self.declare_parameter("move_action_status_topic", "")
        self.declare_parameter("move_action_feedback_topic", "")
        self.declare_parameter("execute_status_topic", "")
        self.declare_parameter("preview_target_topic_left", "fabric_preview/target_poseL")
        self.declare_parameter("preview_target_topic_right", "fabric_preview/target_poseR")
        self.declare_parameter("preview_grip_topic_left", "fabric_preview/gripL")
        self.declare_parameter("preview_grip_topic_right", "fabric_preview/gripR")

        self.feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.update_topic = str(self.get_parameter("update_topic").value)
        self.default_side = str(self.get_parameter("default_side").value).lower()
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.publish_on_execute_only = bool(self.get_parameter("publish_on_execute_only").value)
        self.publish_preview_on_plan = bool(self.get_parameter("publish_preview_on_plan").value)
        self.publish_preview_always = bool(self.get_parameter("publish_preview_always").value)
        self.move_action_status_topic = str(self.get_parameter("move_action_status_topic").value)
        self.move_action_feedback_topic = str(self.get_parameter("move_action_feedback_topic").value)
        self.execute_status_topic = str(self.get_parameter("execute_status_topic").value)
        self.preview_target_topic_left = str(self.get_parameter("preview_target_topic_left").value)
        self.preview_target_topic_right = str(self.get_parameter("preview_target_topic_right").value)
        self.preview_grip_topic_left = str(self.get_parameter("preview_grip_topic_left").value)
        self.preview_grip_topic_right = str(self.get_parameter("preview_grip_topic_right").value)
        self.move_action_active = False
        self.execute_active = False
        self.move_execute_hint = False

        self.left_pub = self.create_publisher(PoseStamped, "control/target_poseL", 10)
        self.right_pub = self.create_publisher(PoseStamped, "control/target_poseR", 10)
        self.left_grip_pub = self.create_publisher(Bool, "control/gripL", 10)
        self.right_grip_pub = self.create_publisher(Bool, "control/gripR", 10)
        self.preview_left_pub = self.create_publisher(PoseStamped, self.preview_target_topic_left, 10)
        self.preview_right_pub = self.create_publisher(PoseStamped, self.preview_target_topic_right, 10)
        self.preview_left_grip_pub = self.create_publisher(Bool, self.preview_grip_topic_left, 10)
        self.preview_right_grip_pub = self.create_publisher(Bool, self.preview_grip_topic_right, 10)

        self.left_pose = None
        self.right_pose = None
        self._subs: Dict[str, object] = {}
        self._move_status_sub = None
        self._move_feedback_sub = None
        self._execute_status_sub = None
        self._log_once: Set[str] = set()
        self._action_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        if self.feedback_topic:
            self._subscribe_feedback(self.feedback_topic)
        if self.update_topic:
            self._subscribe_update(self.update_topic)
        if self.move_action_status_topic:
            self._subscribe_move_status(self.move_action_status_topic)
        if self.move_action_feedback_topic:
            self._subscribe_move_feedback(self.move_action_feedback_topic)
        if self.execute_status_topic:
            self._subscribe_execute_status(self.execute_status_topic)
        # Auto discover MoveIt interactive marker topics if explicit topics are not set.
        self.discovery_timer = self.create_timer(1.0, self._discover_topics)

        dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(dt, self._tick)
        self.get_logger().info(
            "MoveIt bridge started. "
            f"default_side={self.default_side}, explicit_feedback='{self.feedback_topic}', "
            f"explicit_update='{self.update_topic}', publish_on_execute_only={self.publish_on_execute_only}, "
            f"publish_preview_on_plan={self.publish_preview_on_plan}, "
            f"publish_preview_always={self.publish_preview_always}, "
            f"move_action_status_topic='{self.move_action_status_topic}', "
            f"move_action_feedback_topic='{self.move_action_feedback_topic}', "
            f"execute_status_topic='{self.execute_status_topic}'"
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

    def _subscribe_move_status(self, topic: str) -> None:
        if self._move_status_sub is not None:
            return
        self._move_status_sub = self.create_subscription(
            GoalStatusArray, topic, self._move_status_cb, self._action_status_qos
        )
        self.get_logger().info(f"Subscribed move action status: {topic}")

    def _move_status_cb(self, msg: GoalStatusArray) -> None:
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        active = any(st.status in active_states for st in msg.status_list)
        if active == self.move_action_active:
            return
        self.move_action_active = active
        if not self.move_action_active and self.move_execute_hint:
            self.move_execute_hint = False
            self.get_logger().info("Move execute hint -> False (move action inactive)")
        self.get_logger().info(f"Move action active -> {self.move_action_active}")

    def _subscribe_move_feedback(self, topic: str) -> None:
        if self._move_feedback_sub is not None:
            return
        self._move_feedback_sub = self.create_subscription(
            MoveGroup.FeedbackMessage, topic, self._move_feedback_cb, 10
        )
        self.get_logger().info(f"Subscribed move action feedback: {topic}")

    def _move_feedback_cb(self, msg: MoveGroup.FeedbackMessage) -> None:
        state = str(msg.feedback.state).strip().upper()
        execute_like_states = {"MONITOR", "EXECUTING"}
        hint = state in execute_like_states
        if hint == self.move_execute_hint:
            return
        self.move_execute_hint = hint
        self.get_logger().info(f"Move execute hint -> {self.move_execute_hint} (state='{state}')")

    def _subscribe_execute_status(self, topic: str) -> None:
        if self._execute_status_sub is not None:
            return
        self._execute_status_sub = self.create_subscription(
            GoalStatusArray, topic, self._execute_status_cb, self._action_status_qos
        )
        self.get_logger().info(f"Subscribed execute status: {topic}")

    def _execute_status_cb(self, msg: GoalStatusArray) -> None:
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        active = any(st.status in active_states for st in msg.status_list)
        if active == self.execute_active:
            return
        self.execute_active = active
        self.get_logger().info(f"Execute active -> {self.execute_active}")

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
            if self._move_status_sub is None and name.endswith("move_action/_action/status") and (
                "action_msgs/msg/GoalStatusArray" in types
            ):
                self._subscribe_move_status(name)
            if self._move_feedback_sub is None and name.endswith("move_action/_action/feedback") and (
                "moveit_msgs/action/MoveGroup_FeedbackMessage" in types
            ):
                self._subscribe_move_feedback(name)
            if self._execute_status_sub is None and name.endswith("execute_trajectory/_action/status") and (
                "action_msgs/msg/GoalStatusArray" in types
            ):
                self._subscribe_execute_status(name)

    def _publish_grips(self, left_pub, right_pub) -> None:
        on = Bool()
        on.data = True
        left_pub.publish(on)
        right_pub.publish(on)

    def _publish_targets(self, left_pose, right_pose, left_pub, right_pub) -> None:
        now = self.get_clock().now().to_msg()
        if left_pose is not None:
            left_pose.header.stamp = now
            left_pub.publish(left_pose)
        if right_pose is not None:
            right_pose.header.stamp = now
            right_pub.publish(right_pose)

    def _tick(self) -> None:
        # Preview path generation through Fabric for marker drag and planning.
        preview_active = self.publish_preview_on_plan and not self.execute_active
        if preview_active and not self.publish_preview_always:
            preview_active = self.move_action_active
        if preview_active:
            self._publish_grips(self.preview_left_grip_pub, self.preview_right_grip_pub)
            self._publish_targets(
                self.left_pose,
                self.right_pose,
                self.preview_left_pub,
                self.preview_right_pub,
            )

        # Real control path (to Fabric planner that drives hardware path).
        control_active = True
        if self.publish_on_execute_only:
            # Strict mode: only execute stage can drive real control outputs.
            control_active = self.execute_active or self.move_execute_hint
        if not control_active:
            return

        self._publish_grips(self.left_grip_pub, self.right_grip_pub)
        self._publish_targets(self.left_pose, self.right_pose, self.left_pub, self.right_pub)


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
