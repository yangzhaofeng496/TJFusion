#!/usr/bin/env python3
import sys
from functools import partial

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, ColorRGBA
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker

try:
    from interactive_markers.interactive_marker_server import InteractiveMarkerServer
except ImportError as exc:
    print(
        "[fabric_target_markers] Missing dependency 'interactive_markers'. "
        "Install: sudo apt install ros-humble-interactive-markers",
        file=sys.stderr,
    )
    raise SystemExit(2) from exc


class FabricTargetMarkers(Node):
    def __init__(self) -> None:
        super().__init__("fabric_target_markers")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("left_init", [0.45, 0.25, 1.10])
        self.declare_parameter("right_init", [0.45, -0.25, 1.10])

        self.frame_id = self.get_parameter("frame_id").value
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        left_init = [float(v) for v in self.get_parameter("left_init").value]
        right_init = [float(v) for v in self.get_parameter("right_init").value]

        self.left_pub = self.create_publisher(PoseStamped, "control/target_poseL", 10)
        self.right_pub = self.create_publisher(PoseStamped, "control/target_poseR", 10)
        self.left_grip_pub = self.create_publisher(Bool, "control/gripL", 10)
        self.right_grip_pub = self.create_publisher(Bool, "control/gripR", 10)

        self.server = InteractiveMarkerServer(self, "fabric_targets")

        self.left_pose = self._make_pose(left_init)
        self.right_pose = self._make_pose(right_init)

        self._insert_marker(
            name="target_left",
            description="Fabric Target Left",
            pose=self.left_pose,
            color=ColorRGBA(r=0.1, g=0.8, b=0.2, a=0.95),
            cb=partial(self._feedback_cb, side="left"),
        )
        self._insert_marker(
            name="target_right",
            description="Fabric Target Right",
            pose=self.right_pose,
            color=ColorRGBA(r=0.95, g=0.2, b=0.2, a=0.95),
            cb=partial(self._feedback_cb, side="right"),
        )
        self.server.applyChanges()

        dt = 1.0 / max(1.0, self.publish_rate_hz)
        self.timer = self.create_timer(dt, self._publish_targets)
        self.get_logger().info(
            "Fabric target markers ready. RViz topic: /fabric_targets/update"
        )

    @staticmethod
    def _make_pose(xyz):
        pose = Pose()
        pose.position.x = xyz[0]
        pose.position.y = xyz[1]
        pose.position.z = xyz[2]
        pose.orientation.w = 1.0
        return pose

    def _insert_marker(self, name, description, pose, color, cb):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = name
        marker.description = description
        marker.scale = 0.20
        marker.pose = pose

        sphere = Marker()
        sphere.type = Marker.SPHERE
        sphere.scale.x = 0.06
        sphere.scale.y = 0.06
        sphere.scale.z = 0.06
        sphere.color = color

        vis_control = InteractiveMarkerControl()
        vis_control.always_visible = True
        vis_control.markers.append(sphere)
        marker.controls.append(vis_control)

        self._append_6dof_controls(marker)
        self.server.insert(marker, cb)

    @staticmethod
    def _append_6dof_controls(int_marker):
        modes = [
            ("rotate_x", 1.0, 1.0, 0.0, 0.0, InteractiveMarkerControl.ROTATE_AXIS),
            ("move_x", 1.0, 1.0, 0.0, 0.0, InteractiveMarkerControl.MOVE_AXIS),
            ("rotate_y", 1.0, 0.0, 1.0, 0.0, InteractiveMarkerControl.ROTATE_AXIS),
            ("move_y", 1.0, 0.0, 1.0, 0.0, InteractiveMarkerControl.MOVE_AXIS),
            ("rotate_z", 1.0, 0.0, 0.0, 1.0, InteractiveMarkerControl.ROTATE_AXIS),
            ("move_z", 1.0, 0.0, 0.0, 1.0, InteractiveMarkerControl.MOVE_AXIS),
        ]
        for name, w, x, y, z, mode in modes:
            control = InteractiveMarkerControl()
            control.name = name
            control.orientation.w = w
            control.orientation.x = x
            control.orientation.y = y
            control.orientation.z = z
            control.interaction_mode = mode
            control.orientation_mode = InteractiveMarkerControl.FIXED
            int_marker.controls.append(control)

    def _feedback_cb(self, feedback, side):
        if side == "left":
            self.left_pose = feedback.pose
        else:
            self.right_pose = feedback.pose
        self._publish_targets()

    def _publish_targets(self):
        now = self.get_clock().now().to_msg()

        left = PoseStamped()
        left.header.frame_id = self.frame_id
        left.header.stamp = now
        left.pose = self.left_pose
        self.left_pub.publish(left)

        right = PoseStamped()
        right.header.frame_id = self.frame_id
        right.header.stamp = now
        right.pose = self.right_pose
        self.right_pub.publish(right)

        # Keep deadman switches enabled so planner_node accepts target pose updates.
        on = Bool()
        on.data = True
        self.left_grip_pub.publish(on)
        self.right_grip_pub.publish(on)


def main():
    rclpy.init()
    node = FabricTargetMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
