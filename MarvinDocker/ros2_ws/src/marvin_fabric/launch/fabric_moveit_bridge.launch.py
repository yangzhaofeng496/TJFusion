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
            }
        ],
    )

    return LaunchDescription(
        [
            use_real_hardware_arg,
            simulate_robot_motion_arg,
            enable_fabric_control_arg,
            feedback_topic_arg,
            moveit_demo,
            planner_node,
            real_hardware_node,
            wait_feedback_node,
            start_moveit_after_feedback,
            offline_feedback,
            moveit_bridge,
        ]
    )
