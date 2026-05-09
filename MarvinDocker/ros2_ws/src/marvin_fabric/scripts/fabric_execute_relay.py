#!/usr/bin/env python3
from typing import List, Tuple

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from marvin_msgs.msg import Jointcmd
from moveit_msgs.msg import RobotTrajectory
from rclpy.node import Node
from std_msgs.msg import Bool


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


class FabricExecuteRelay(Node):
    def __init__(self) -> None:
        super().__init__("fabric_execute_relay")

        self.declare_parameter("trajectory_topic", "/fabric_preview/trajectory")
        self.declare_parameter("execute_status_topic", "")
        self.declare_parameter("out_joint_cmd_a_topic", "/control/joint_cmd_A")
        self.declare_parameter("out_joint_cmd_b_topic", "/control/joint_cmd_B")
        self.declare_parameter("out_grip_left_topic", "/control/gripL")
        self.declare_parameter("out_grip_right_topic", "/control/gripR")
        self.declare_parameter("tick_hz", 200.0)
        self.declare_parameter("default_dt_sec", 1.0 / 30.0)
        self.declare_parameter("replay_on_execute_rising_edge_only", True)

        self.trajectory_topic = str(self.get_parameter("trajectory_topic").value)
        self.execute_status_topic = str(self.get_parameter("execute_status_topic").value)
        self.out_joint_cmd_a_topic = str(self.get_parameter("out_joint_cmd_a_topic").value)
        self.out_joint_cmd_b_topic = str(self.get_parameter("out_joint_cmd_b_topic").value)
        self.out_grip_left_topic = str(self.get_parameter("out_grip_left_topic").value)
        self.out_grip_right_topic = str(self.get_parameter("out_grip_right_topic").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.default_dt_sec = float(self.get_parameter("default_dt_sec").value)
        self.replay_on_execute_rising_edge_only = bool(
            self.get_parameter("replay_on_execute_rising_edge_only").value
        )

        self.execute_active = False
        self._last_execute_active = False

        self._latest_traj_points: List[Tuple[float, List[float]]] = []
        self._replay_active = False
        self._replay_start_time = None
        self._next_index = 0

        self.traj_sub = self.create_subscription(
            RobotTrajectory, self.trajectory_topic, self._traj_cb, 10
        )
        self.exec_status_sub = None
        if self.execute_status_topic:
            self._subscribe_execute_status(self.execute_status_topic)

        self.cmd_a_pub = self.create_publisher(Jointcmd, self.out_joint_cmd_a_topic, 10)
        self.cmd_b_pub = self.create_publisher(Jointcmd, self.out_joint_cmd_b_topic, 10)
        self.grip_l_pub = self.create_publisher(Bool, self.out_grip_left_topic, 10)
        self.grip_r_pub = self.create_publisher(Bool, self.out_grip_right_topic, 10)

        self.discovery_timer = self.create_timer(1.0, self._discover_status_topics)
        dt = 1.0 / max(1.0, self.tick_hz)
        self.timer = self.create_timer(dt, self._tick)

        self.get_logger().info(
            "Fabric execute relay started. "
            f"trajectory_topic='{self.trajectory_topic}', out_a='{self.out_joint_cmd_a_topic}', "
            f"out_b='{self.out_joint_cmd_b_topic}'"
        )

    def _subscribe_execute_status(self, topic: str) -> None:
        if self.exec_status_sub is not None:
            return
        self.exec_status_sub = self.create_subscription(
            GoalStatusArray, topic, self._execute_status_cb, 10
        )
        self.get_logger().info(f"Subscribed execute status: {topic}")

    def _discover_status_topics(self) -> None:
        if self.exec_status_sub is not None:
            return
        topic_map = dict(self.get_topic_names_and_types())
        for name, types in topic_map.items():
            if "action_msgs/msg/GoalStatusArray" not in types:
                continue
            if name.endswith("execute_trajectory/_action/status"):
                self._subscribe_execute_status(name)
                return

    def _execute_status_cb(self, msg: GoalStatusArray) -> None:
        active = _is_action_active(msg)
        if active != self.execute_active:
            self.execute_active = active
            self.get_logger().info(f"Execute active -> {self.execute_active}")

    def _traj_cb(self, msg: RobotTrajectory) -> None:
        jt = msg.joint_trajectory
        if len(jt.points) < 2:
            self.get_logger().warn("Received trajectory has too few points; ignored.")
            return

        name_to_idx = {n: i for i, n in enumerate(jt.joint_names)}
        missing = [n for n in JOINT_NAMES if n not in name_to_idx]
        if missing:
            self.get_logger().warn(f"Trajectory missing joints: {missing}; ignored.")
            return

        points: List[Tuple[float, List[float]]] = []
        t_prev = -1.0
        for i, pt in enumerate(jt.points):
            q = [float(pt.positions[name_to_idx[n]]) for n in JOINT_NAMES]
            t = float(pt.time_from_start.sec) + float(pt.time_from_start.nanosec) * 1e-9
            if t <= t_prev:
                t = (i + 1) * max(1e-4, self.default_dt_sec)
            points.append((t, q))
            t_prev = t

        self._latest_traj_points = points
        self.get_logger().info(f"Cached Fabric trajectory with {len(points)} points.")

    def _start_replay(self) -> None:
        if not self._latest_traj_points:
            self.get_logger().warn("No cached Fabric trajectory to replay.")
            return
        self._replay_start_time = self.get_clock().now()
        self._next_index = 0
        self._replay_active = True
        self.get_logger().info("Start replay cached Fabric trajectory.")

    def _publish_grip_on(self) -> None:
        msg = Bool()
        msg.data = True
        self.grip_l_pub.publish(msg)
        self.grip_r_pub.publish(msg)

    def _publish_joint_cmd(self, q14: List[float]) -> None:
        now = self.get_clock().now().to_msg()

        msg_a = Jointcmd()
        msg_a.header.stamp = now
        msg_a.header.frame_id = "base_link"
        for i in range(7):
            msg_a.positions[i] = float(q14[i])
        self.cmd_a_pub.publish(msg_a)

        msg_b = Jointcmd()
        msg_b.header.stamp = now
        msg_b.header.frame_id = "base_link"
        for i in range(7):
            msg_b.positions[i] = float(q14[i + 7])
        self.cmd_b_pub.publish(msg_b)

    def _tick(self) -> None:
        rising_edge = self.execute_active and not self._last_execute_active
        self._last_execute_active = self.execute_active

        if self.replay_on_execute_rising_edge_only:
            if rising_edge:
                self._start_replay()
        elif self.execute_active and not self._replay_active:
            self._start_replay()

        if not self._replay_active or self._replay_start_time is None:
            return

        if not self.execute_active:
            self._replay_active = False
            self.get_logger().info("Execute inactive; stop replay.")
            return

        elapsed = (self.get_clock().now() - self._replay_start_time).nanoseconds * 1e-9
        last_pub = False
        while self._next_index < len(self._latest_traj_points):
            t, q = self._latest_traj_points[self._next_index]
            if elapsed < t:
                break
            self._publish_grip_on()
            self._publish_joint_cmd(q)
            self._next_index += 1
            last_pub = True

        if self._next_index >= len(self._latest_traj_points):
            self._replay_active = False
            self.get_logger().info("Replay finished.")
        elif not last_pub:
            # Keep grip state fresh while waiting for next point.
            self._publish_grip_on()


def main() -> None:
    rclpy.init()
    node = FabricExecuteRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
