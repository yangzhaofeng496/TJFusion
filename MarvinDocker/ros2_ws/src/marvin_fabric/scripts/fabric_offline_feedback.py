#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from marvin_msgs.msg import Jointfeedback
from sensor_msgs.msg import JointState
from std_msgs.msg import Int16MultiArray


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


class FabricOfflineFeedback(Node):
    def __init__(self) -> None:
        super().__init__("fabric_offline_feedback")
        self.declare_parameter("rate_hz", 100.0)
        self.declare_parameter("simulate_motion", False)
        self.declare_parameter("motion_amp_rad", 0.08)
        self.declare_parameter("motion_freq_hz", 0.15)
        self.declare_parameter("publish_joint_states", True)
        self.declare_parameter(
            "initial_positions",
            [
                1.57, -1.57, -1.57, -1.57, 0.0, 0.0, 0.0,
                -1.57, -1.57, 1.57, -1.57, 0.0, 0.0, 0.0,
            ],
        )

        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.simulate_motion = bool(self.get_parameter("simulate_motion").value)
        self.motion_amp_rad = float(self.get_parameter("motion_amp_rad").value)
        self.motion_freq_hz = float(self.get_parameter("motion_freq_hz").value)
        self.publish_joint_states = bool(self.get_parameter("publish_joint_states").value)
        self.initial_positions = [
            float(v) for v in self.get_parameter("initial_positions").value
        ]
        if len(self.initial_positions) != 14:
            self.initial_positions = [0.0] * 14

        self.arm_state_pub = self.create_publisher(Int16MultiArray, "info/arm_state", 10)
        self.joint_fb_pub = self.create_publisher(Jointfeedback, "info/joint_feedback", 10)
        self.joint_state_pub = self.create_publisher(JointState, "joint_states", 10)
        self.js_sub = self.create_subscription(
            JointState, "joint_states", self._joint_state_cb, 10
        )

        self.positions = list(self.initial_positions)
        self.velocities = [0.0] * 14
        self.efforts = [0.0] * 14

        self._t = 0.0
        dt = 1.0 / max(1.0, self.rate_hz)
        self.dt = dt
        self.timer = self.create_timer(dt, self._tick)
        self.get_logger().info(
            "Offline feedback node active. "
            f"simulate_motion={self.simulate_motion}, publish_joint_states={self.publish_joint_states}"
        )

    def _joint_state_cb(self, msg: JointState) -> None:
        # Map planner-published joint_states into marvin feedback layout.
        if not msg.name:
            return
        index_map = {name: i for i, name in enumerate(msg.name)}
        for i, name in enumerate(JOINT_NAMES):
            j = index_map.get(name)
            if j is None:
                continue
            if j < len(msg.position):
                self.positions[i] = float(msg.position[j])
            if j < len(msg.velocity):
                self.velocities[i] = float(msg.velocity[j])
            if j < len(msg.effort):
                self.efforts[i] = float(msg.effort[j])

    def _tick(self) -> None:
        if self.simulate_motion:
            # Produce smooth mock arm motion to emulate real joint feedback.
            w = 2.0 * math.pi * self.motion_freq_hz
            for i in range(14):
                phase = 0.35 * i
                self.positions[i] = self.initial_positions[i] + self.motion_amp_rad * math.sin(w * self._t + phase)
                self.velocities[i] = self.motion_amp_rad * w * math.cos(w * self._t + phase)
            self._t += self.dt

        state = Int16MultiArray()
        state.data = [3, 3]
        self.arm_state_pub.publish(state)

        fb = Jointfeedback()
        fb.positions = self.positions
        fb.velocities = self.velocities
        fb.efforts = self.efforts
        self.joint_fb_pub.publish(fb)

        if self.publish_joint_states:
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = JOINT_NAMES
            js.position = list(self.positions)
            js.velocity = list(self.velocities)
            js.effort = list(self.efforts)
            self.joint_state_pub.publish(js)


def main():
    rclpy.init()
    node = FabricOfflineFeedback()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
