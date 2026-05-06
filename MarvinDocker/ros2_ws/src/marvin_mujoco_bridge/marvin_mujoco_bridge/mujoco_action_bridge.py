#!/usr/bin/env python3

from __future__ import annotations

from typing import Dict, Optional, Tuple

from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
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


class MujocoActionBridge(Node):
    def __init__(self) -> None:
        super().__init__("mujoco_action_bridge")

        self.declare_parameter("endpoint", "tcp://127.0.0.1:5555")
        self.declare_parameter("timeout_ms", 2000)
        self.declare_parameter("relative", False)
        self.declare_parameter("send_hold_on_cancel", True)

        self.endpoint = str(self.get_parameter("endpoint").value)
        self.timeout_ms = int(self.get_parameter("timeout_ms").value)
        self.relative = bool(self.get_parameter("relative").value)
        self.send_hold_on_cancel = bool(self.get_parameter("send_hold_on_cancel").value)
        self._zmq_context = zmq.Context.instance()

        self._left_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/left_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self._right_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/right_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )
        self._both_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/both_arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
        )

        self.get_logger().info(
            f"Action bridge ready: endpoint={self.endpoint}, timeout_ms={self.timeout_ms}, "
            f"relative={self.relative}, send_hold_on_cancel={self.send_hold_on_cancel}"
        )

    def destroy_node(self):
        self._left_server.destroy()
        self._right_server.destroy()
        self._both_server.destroy()
        super().destroy_node()

    @staticmethod
    def _duration_to_sec(msg_duration) -> float:
        return float(msg_duration.sec) + float(msg_duration.nanosec) * 1e-9

    def _goal_cb(self, _goal_request):
        return GoalResponse.ACCEPT

    def _cancel_cb(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _send_request(self, payload: Dict) -> Tuple[bool, Optional[Dict]]:
        socket = self._zmq_context.socket(zmq.REQ)
        socket.setsockopt(zmq.LINGER, 0)
        socket.setsockopt(zmq.RCVTIMEO, self.timeout_ms)
        socket.setsockopt(zmq.SNDTIMEO, self.timeout_ms)
        socket.connect(self.endpoint)
        try:
            socket.send_json(payload)
            reply = socket.recv_json()
            if isinstance(reply, dict):
                return True, reply
            return True, {"raw_reply": reply}
        except ZMQError as exc:
            self.get_logger().warning(f"ZMQ request failed: {exc}")
            return False, None
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().warning(f"Unexpected ZMQ error: {exc}")
            return False, None
        finally:
            socket.close(0)

    def _execute_cb(self, goal_handle):
        result = FollowJointTrajectory.Result()
        traj = goal_handle.request.trajectory

        if not traj.joint_names or not traj.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "empty trajectory"
            return result

        name_to_index = {name: idx for idx, name in enumerate(traj.joint_names)}
        supported_in_msg = [name for name in traj.joint_names if name in SUPPORTED_JOINTS]
        if not supported_in_msg:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_JOINTS
            result.error_string = "no supported joints"
            return result

        prev_time = 0.0
        for point_idx, point in enumerate(traj.points):
            if goal_handle.is_cancel_requested:
                if self.send_hold_on_cancel:
                    self._send_request({"cmd": "hold"})
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "goal canceled"
                return result

            current_time = self._duration_to_sec(point.time_from_start)
            duration = current_time - prev_time
            prev_time = current_time
            if duration < 0.0:
                duration = 0.0

            positions: Dict[str, float] = {}
            for joint_name in supported_in_msg:
                idx = name_to_index[joint_name]
                if idx < len(point.positions):
                    positions[joint_name] = float(point.positions[idx])

            if not positions:
                continue

            request = {
                "cmd": "set_positions",
                "positions": positions,
                "duration": float(duration),
                "relative": self.relative,
            }
            ok, reply = self._send_request(request)
            if (not ok) or (reply is not None and reply.get("ok") is False):
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = f"set_positions failed at point {point_idx}: {reply}"
                return result

            feedback = FollowJointTrajectory.Feedback()
            feedback.joint_names = supported_in_msg
            feedback.desired = point
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = ""
        return result


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MujocoActionBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
