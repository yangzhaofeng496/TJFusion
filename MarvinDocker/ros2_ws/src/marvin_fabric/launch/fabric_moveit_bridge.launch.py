from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.actions import RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fabric_share = get_package_share_directory("marvin_fabric")
    moveit_share = get_package_share_directory("moveit_m6")
    marvin_ros_control_share = get_package_share_directory("marvin_ros_control")
    config = os.path.join(fabric_share, "config", "robot_param_m6.yaml")
    real_hw_config = os.path.join(marvin_ros_control_share, "config", "robot_param_m6.yaml")

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, "launch", "demo.launch.py")
        ),
        condition=UnlessCondition(LaunchConfiguration("use_real_hardware")),
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

    offline_feedback = Node(
        condition=UnlessCondition(LaunchConfiguration("use_real_hardware")),
        package="marvin_fabric",
        executable="fabric_offline_feedback.py",
        name="fabric_offline_feedback",
        output="screen",
        parameters=[
            {
                "rate_hz": 100.0,
                "simulate_motion": LaunchConfiguration("simulate_robot_motion"),
                "publish_joint_states": True,
            }
        ],
    )

    real_hardware_node = Node(
        condition=IfCondition(LaunchConfiguration("use_real_hardware")),
        package="marvin_ros_control",
        executable="marvin_robot_node",
        name="marvin_robot_node",
        parameters=[real_hw_config],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    wait_feedback_node = Node(
        condition=IfCondition(LaunchConfiguration("use_real_hardware")),
        package="marvin_fabric",
        executable="wait_for_joint_feedback.py",
        name="wait_for_joint_feedback",
        output="screen",
        parameters=[{"topic": "/info/joint_feedback", "timeout_sec": 20.0}],
    )

    start_moveit_after_feedback = RegisterEventHandler(
        condition=IfCondition(LaunchConfiguration("use_real_hardware")),
        event_handler=OnProcessExit(
            target_action=wait_feedback_node,
            on_exit=[moveit_demo_after_wait],
        ),
    )

    feedback_topic_arg = DeclareLaunchArgument(
        "feedback_topic",
        default_value="/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
        description="MoveIt interactive marker feedback topic",
    )

    use_real_hardware_arg = DeclareLaunchArgument(
        "use_real_hardware",
        default_value="false",
        description="true: start marvin_ros_control hardware node; false: use offline feedback",
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
