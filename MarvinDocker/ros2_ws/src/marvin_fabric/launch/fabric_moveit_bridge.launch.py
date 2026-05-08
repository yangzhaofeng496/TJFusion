from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xml.etree.ElementTree as ET


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


def _load_home_positions_from_srdf(moveit_share: str) -> list:
    srdf_path = os.path.join(moveit_share, "config", "marvin_robot.srdf")
    fallback = [0.0] * len(JOINT_NAMES)
    try:
        root = ET.parse(srdf_path).getroot()
        home_state = None
        for group_state in root.findall("group_state"):
            if (
                group_state.attrib.get("name") == "home"
                and group_state.attrib.get("group") == "both_arm"
            ):
                home_state = group_state
                break
        if home_state is None:
            return fallback

        value_map = {}
        for joint in home_state.findall("joint"):
            name = joint.attrib.get("name")
            value = joint.attrib.get("value")
            if name is None or value is None:
                continue
            value_map[name] = float(value)
        return [value_map.get(name, 0.0) for name in JOINT_NAMES]
    except Exception:
        return fallback


def _make_offline_feedback_node(simulate_motion_cfg, home_positions, *, condition=None) -> Node:
    return Node(
        condition=condition,
        package="marvin_fabric",
        executable="fabric_offline_feedback.py",
        name="fabric_offline_feedback",
        output="screen",
        parameters=[
            {
                "rate_hz": 100.0,
                "simulate_motion": simulate_motion_cfg,
                "publish_joint_states": True,
                "initial_positions": home_positions,
            }
        ],
    )


def generate_launch_description():
    fabric_share = get_package_share_directory("marvin_fabric")
    moveit_share = get_package_share_directory("moveit_m6")
    marvin_ros_control_share = get_package_share_directory("marvin_ros_control")
    config = os.path.join(fabric_share, "config", "robot_param_m6.yaml")
    real_hw_config = os.path.join(marvin_ros_control_share, "config", "robot_param_m6.yaml")
    home_positions = _load_home_positions_from_srdf(moveit_share)
    use_real_hardware_cfg = LaunchConfiguration("use_real_hardware")
    simulate_motion_cfg = LaunchConfiguration("simulate_robot_motion")

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "demo.launch.py")
        ),
        condition=UnlessCondition(use_real_hardware_cfg),
    )

    moveit_demo_after_wait = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "demo.launch.py")
        ),
    )

    planner_node = Node(
        condition=IfCondition(LaunchConfiguration("enable_fabric_control")),
        package="marvin_fabric",
        executable="planner_node",
        name="planner_node",
        parameters=[config],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    offline_feedback = _make_offline_feedback_node(
        simulate_motion_cfg,
        home_positions,
        condition=UnlessCondition(use_real_hardware_cfg),
    )

    real_hardware_node = Node(
        condition=IfCondition(use_real_hardware_cfg),
        package="marvin_ros_control",
        executable="marvin_robot_node",
        name="marvin_robot_node",
        parameters=[real_hw_config],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    wait_feedback_node = Node(
        condition=IfCondition(use_real_hardware_cfg),
        package="marvin_fabric",
        executable="wait_for_joint_feedback.py",
        name="wait_for_joint_feedback",
        output="screen",
        parameters=[{"topic": "/info/joint_feedback", "timeout_sec": 20.0}],
    )

    offline_feedback_fallback = _make_offline_feedback_node(
        simulate_motion_cfg,
        home_positions,
    )

    def _on_wait_feedback_exit(event, _context):
        return_code = getattr(event, "returncode", getattr(event, "return_code", 1))
        if return_code == 0:
            return [
                LogInfo(msg="[fabric_moveit_bridge] Real robot feedback detected, using hardware mode."),
                moveit_demo_after_wait,
            ]
        return [
            LogInfo(msg="[fabric_moveit_bridge] No robot feedback, falling back to offline mode."),
            offline_feedback_fallback,
            moveit_demo_after_wait,
        ]

    start_moveit_after_feedback = RegisterEventHandler(
        condition=IfCondition(use_real_hardware_cfg),
        event_handler=OnProcessExit(
            target_action=wait_feedback_node,
            on_exit=_on_wait_feedback_exit,
        ),
    )

    feedback_topic_arg = DeclareLaunchArgument(
        "feedback_topic",
        default_value="/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
        description="MoveIt interactive marker feedback topic",
    )

    use_real_hardware_arg = DeclareLaunchArgument(
        "use_real_hardware",
        default_value="true",
        description="true: try real hardware first and fallback to offline on timeout; false: force offline",
    )

    simulate_robot_motion_arg = DeclareLaunchArgument(
        "simulate_robot_motion",
        default_value="false",
        description="Only in offline mode: publish sinusoidal mock joint motion data",
    )

    enable_fabric_control_arg = DeclareLaunchArgument(
        "enable_fabric_control",
        default_value="true",
        description="true: run Fabric planner+bridge (robot can move); false: feedback-to-MoveIt only",
    )

    publish_on_execute_only_arg = DeclareLaunchArgument(
        "publish_on_execute_only",
        default_value="true",
        description="true: bridge publishes control only while MoveIt execute action is active",
    )
    enable_fabric_plan_preview_arg = DeclareLaunchArgument(
        "enable_fabric_plan_preview",
        default_value="true",
        description="true: run Fabric dry-run preview path generation for MoveIt Plan",
    )

    moveit_bridge = Node(
        condition=IfCondition(LaunchConfiguration("enable_fabric_control")),
        package="marvin_fabric",
        executable="moveit_goal_bridge.py",
        name="moveit_goal_bridge",
        output="screen",
        parameters=[
            {
                "feedback_topic": LaunchConfiguration("feedback_topic"),
                "default_side": "right",
                "publish_rate_hz": 30.0,
                "output_frame_id": "base_link",
                "publish_on_execute_only": LaunchConfiguration("publish_on_execute_only"),
                "publish_preview_on_plan": LaunchConfiguration("enable_fabric_plan_preview"),
                "publish_preview_always": True,
                "move_action_status_topic": "",
                "execute_status_topic": "",
                "preview_target_topic_left": "fabric_preview/target_poseL",
                "preview_target_topic_right": "fabric_preview/target_poseR",
                "preview_grip_topic_left": "fabric_preview/gripL",
                "preview_grip_topic_right": "fabric_preview/gripR",
            }
        ],
    )

    preview_planner_node = Node(
        condition=IfCondition(LaunchConfiguration("enable_fabric_plan_preview")),
        package="marvin_fabric",
        executable="planner_node",
        name="planner_node_preview",
        parameters=[config],
        remappings=[
            ("control/target_poseL", "fabric_preview/target_poseL"),
            ("control/target_poseR", "fabric_preview/target_poseR"),
            ("control/gripL", "fabric_preview/gripL"),
            ("control/gripR", "fabric_preview/gripR"),
            ("control/joint_cmd_A", "fabric_preview/joint_cmd_A"),
            ("control/joint_cmd_B", "fabric_preview/joint_cmd_B"),
            ("joint_states", "fabric_preview/joint_states"),
            ("eef_pose", "fabric_preview/eef_pose"),
            ("collision_spheres", "fabric_preview/collision_spheres"),
            ("fabric_markers", "fabric_preview/fabric_markers"),
            ("arm/eef_state", "fabric_preview/eef_state"),
            ("info/eef_left", "fabric_preview/eef_left"),
            ("info/eef_right", "fabric_preview/eef_right"),
            ("test_pub", "fabric_preview/test_pub"),
            ("reset_left_arm", "fabric_preview/reset_left_arm"),
            ("reset_right_arm", "fabric_preview/reset_right_arm"),
        ],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    preview_display_node = Node(
        condition=IfCondition(LaunchConfiguration("enable_fabric_plan_preview")),
        package="marvin_fabric",
        executable="fabric_plan_preview.py",
        name="fabric_plan_preview",
        output="screen",
        parameters=[
            {
                "plan_status_topic": "",
                "execute_status_topic": "",
                "joint_cmd_a_topic": "fabric_preview/joint_cmd_A",
                "joint_cmd_b_topic": "fabric_preview/joint_cmd_B",
                "display_topic": "/display_planned_path",
                "publish_rate_hz": 30.0,
                "model_id": "marvin_robot",
            }
        ],
    )

    return LaunchDescription(
        [
            use_real_hardware_arg,
            simulate_robot_motion_arg,
            enable_fabric_control_arg,
            publish_on_execute_only_arg,
            enable_fabric_plan_preview_arg,
            feedback_topic_arg,
            moveit_demo,
            planner_node,
            preview_planner_node,
            preview_display_node,
            real_hardware_node,
            wait_feedback_node,
            start_moveit_after_feedback,
            offline_feedback,
            moveit_bridge,
        ]
    )
