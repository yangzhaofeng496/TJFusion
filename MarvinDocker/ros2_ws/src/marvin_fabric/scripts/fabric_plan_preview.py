#!/usr/bin/env python3
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Duration
from marvin_msgs.msg import Jointcmd
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_NAMES = [
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


def _is_action_active(msg: GoalStatusArray) -> bool:
    active_states = {
        GoalStatus.STATUS_ACCEPTED,
        GoalStatus.STATUS_EXECUTING,
        GoalStatus.STATUS_CANCELING,
    }
    return any(st.status in active_states for st in msg.status_list)


class FabricPlanPreview(Node):
    def __init__(self) -> None:
        super().__init__("fabric_plan_preview")

        self.declare_parameter("plan_status_topic", "")
        self.declare_parameter("execute_status_topic", "")
        self.declare_parameter("joint_cmd_a_topic", "fabric_preview/joint_cmd_A")
        self.declare_parameter("joint_cmd_b_topic", "fabric_preview/joint_cmd_B")
        self.declare_parameter("display_topic", "/display_planned_path")
        self.declare_parameter("trajectory_topic", "/fabric_preview/trajectory")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("republish_cached_display_hz", 2.0)
        self.declare_parameter("model_id", "marvin_robot")

        self.plan_status_topic = str(self.get_parameter("plan_status_topic").value)
        self.execute_status_topic = str(self.get_parameter("execute_status_topic").value)
        self.joint_cmd_a_topic = str(self.get_parameter("joint_cmd_a_topic").value)
        self.joint_cmd_b_topic = str(self.get_parameter("joint_cmd_b_topic").value)
        self.display_topic = str(self.get_parameter("display_topic").value)
        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.republish_cached_display_hz = float(self.get_parameter("republish_cached_display_hz").value)
        self.model_id = str(self.get_parameter("model_id").value)

        self.plan_active = False
        self.execute_active = False
        self.capture_active = False

        self.latest_a: Optional[List[float]] = None
        self.latest_b: Optional[List[float]] = None
        self.samples: List[List[float]] = []
        self.last_fabric_display: Optional[DisplayTrajectory] = None
        self.last_fabric_rt: Optional[RobotTrajectory] = None
        self._last_republish_time = self.get_clock().now()

        self.display_pub = self.create_publisher(DisplayTrajectory, self.display_topic, 10)
        self.trajectory_pub = self.create_publisher(RobotTrajectory, self.trajectory_topic, 10)
        self.cmd_a_sub = self.create_subscription(Jointcmd, self.joint_cmd_a_topic, self._cmd_a_cb, 10)
        self.cmd_b_sub = self.create_subscription(Jointcmd, self.joint_cmd_b_topic, self._cmd_b_cb, 10)

        self.plan_status_sub = None
        self.exec_status_sub = None
        if self.plan_status_topic:
            self._subscribe_plan_status(self.plan_status_topic)
        if self.execute_status_topic:
            self._subscribe_execute_status(self.execute_status_topic)

        dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.dt = dt
        self.timer = self.create_timer(dt, self._tick)
        self.discovery_timer = self.create_timer(1.0, self._discover_status_topics)

        self.get_logger().info(
            "Fabric plan preview started. "
            f"joint_cmd_a='{self.joint_cmd_a_topic}', joint_cmd_b='{self.joint_cmd_b_topic}', "
            f"display_topic='{self.display_topic}'"
        )

    def _subscribe_plan_status(self, topic: str) -> None:
        if self.plan_status_sub is not None:
            return
        self.plan_status_sub = self.create_subscription(GoalStatusArray, topic, self._plan_status_cb, 10)
        self.get_logger().info(f"Subscribed plan status: {topic}")

    def _subscribe_execute_status(self, topic: str) -> None:
        if self.exec_status_sub is not None:
            return
        self.exec_status_sub = self.create_subscription(GoalStatusArray, topic, self._execute_status_cb, 10)
        self.get_logger().info(f"Subscribed execute status: {topic}")

    def _discover_status_topics(self) -> None:
        topic_map = dict(self.get_topic_names_and_types())
        for name, types in topic_map.items():
            if "action_msgs/msg/GoalStatusArray" not in types:
                continue
            if self.plan_status_sub is None and name.endswith("move_action/_action/status"):
                self._subscribe_plan_status(name)
            if self.exec_status_sub is None and name.endswith("execute_trajectory/_action/status"):
                self._subscribe_execute_status(name)

    def _plan_status_cb(self, msg: GoalStatusArray) -> None:
        active = _is_action_active(msg)
        if active != self.plan_active:
            self.plan_active = active
            self.get_logger().info(f"Plan action active -> {self.plan_active}")

    def _execute_status_cb(self, msg: GoalStatusArray) -> None:
        active = _is_action_active(msg)
        if active != self.execute_active:
            self.execute_active = active
            self.get_logger().info(f"Execute action active -> {self.execute_active}")

    def _cmd_a_cb(self, msg: Jointcmd) -> None:
        self.latest_a = [float(v) for v in list(msg.positions)[:7]]

    def _cmd_b_cb(self, msg: Jointcmd) -> None:
        self.latest_b = [float(v) for v in list(msg.positions)[:7]]

    def _tick(self) -> None:
        should_capture = self.plan_active and not self.execute_active
        if should_capture and not self.capture_active:
            self.capture_active = True
            self.samples = []
            self.get_logger().info("Start capturing Fabric preview trajectory.")
        elif not should_capture and self.capture_active:
            self.capture_active = False
            self._publish_preview_if_ready()

        if not self.capture_active:
            self._republish_cached_display_if_needed()
            return
        if self.latest_a is None or self.latest_b is None:
            return
        if len(self.latest_a) != 7 or len(self.latest_b) != 7:
            return
        self.samples.append(self.latest_a + self.latest_b)

    def _publish_preview_if_ready(self) -> None:
        if len(self.samples) < 2:
            self.get_logger().warn("Fabric preview capture has too few samples; skip publish.")
            return

        jt = JointTrajectory()
        jt.joint_names = list(JOINT_NAMES)

        for i, q in enumerate(self.samples):
            pt = JointTrajectoryPoint()
            pt.positions = list(q)
            sec = int(i * self.dt)
            nsec = int((i * self.dt - sec) * 1e9)
            pt.time_from_start = Duration(sec=sec, nanosec=nsec)
            jt.points.append(pt)

        rt = RobotTrajectory()
        rt.joint_trajectory = jt

        msg = DisplayTrajectory()
        msg.model_id = self.model_id
        msg.trajectory = [rt]
        msg.trajectory_start = JointState()
        msg.trajectory_start.name = list(JOINT_NAMES)
        msg.trajectory_start.position = list(self.samples[0])
        msg.trajectory_start.header.stamp = self.get_clock().now().to_msg()

        self.last_fabric_display = msg
        self.last_fabric_rt = rt
        self._last_republish_time = self.get_clock().now()
        self.display_pub.publish(msg)
        self.trajectory_pub.publish(rt)
        self.get_logger().info(
            f"Published Fabric preview trajectory to {self.display_topic} and {self.trajectory_topic} "
            f"with {len(self.samples)} points."
        )

    def _republish_cached_display_if_needed(self) -> None:
        if self.last_fabric_display is None or self.last_fabric_rt is None:
            return
        if self.execute_active:
            return
        hz = max(0.0, self.republish_cached_display_hz)
        if hz <= 0.0:
            return
        now = self.get_clock().now()
        period_sec = 1.0 / hz
        elapsed_sec = (now - self._last_republish_time).nanoseconds * 1e-9
        if elapsed_sec < period_sec:
            return

        # Keep MoveIt display topic pinned to latest Fabric trajectory.
        self.last_fabric_display.trajectory_start.header.stamp = now.to_msg()
        self.display_pub.publish(self.last_fabric_display)
        self.trajectory_pub.publish(self.last_fabric_rt)
        self._last_republish_time = now


def main() -> None:
    rclpy.init()
    node = FabricPlanPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
