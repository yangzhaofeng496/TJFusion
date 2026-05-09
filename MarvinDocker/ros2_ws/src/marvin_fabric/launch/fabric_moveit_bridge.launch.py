from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fabric_share = get_package_share_directory("marvin_fabric")
    moveit_share = get_package_share_directory("moveit_m6")
    marvin_ros_control_share = get_package_share_directory("marvin_ros_control")

    fabric_config = os.path.join(fabric_share, "config", "robot_param_m6.yaml")
    real_hw_config = os.path.join(marvin_ros_control_share, "config", "robot_param_m6.yaml")

    feedback_topic_arg = DeclareLaunchArgument(
        "feedback_topic",
        default_value="/rviz_moveit_motion_planning_display/robot_interaction_interactive_marker_topic/feedback",
        description="MoveIt interactive marker feedback topic",
    )
    update_topic_arg = DeclareLaunchArgument(
        "update_topic",
        default_value="",
        description="MoveIt interactive marker update topic, empty means auto-discover",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="30.0",
        description="Rate to publish execute-stage target poses",
    )

    moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit_share, "launch", "demo.launch.py"))
    )

    real_hardware_node = Node(
        package="marvin_ros_control",
        executable="marvin_robot_node",
        name="marvin_robot_node",
        parameters=[real_hw_config],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    robot_mode_initializer = Node(
        package="marvin_fabric",
        executable="robot_mode_initializer.py",
        name="robot_mode_initializer",
        output="screen",
        parameters=[
            {
                "desired_mode": 3,
                "wait_for_arm_state": True,
                "max_retries": 0,
            }
        ],
    )

    planner_node = Node(
        package="marvin_fabric",
        executable="planner_node",
        name="planner_node",
        parameters=[fabric_config],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    moveit_bridge = Node(
        package="marvin_fabric",
        executable="moveit_goal_bridge.py",
        name="moveit_goal_bridge",
        output="screen",
        parameters=[
            {
                "feedback_topic": LaunchConfiguration("feedback_topic"),
                "update_topic": LaunchConfiguration("update_topic"),
                "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
                "publish_on_execute_only": True,
                "enable_execute_pose_stream": True,
                "move_action_feedback_topic": "/move_action/_action/feedback",
                "execute_status_topic": "/execute_trajectory/_action/status",
                "control_target_topic_left": "/control/target_poseL",
                "control_target_topic_right": "/control/target_poseR",
                "control_grip_topic_left": "/control/gripL",
                "control_grip_topic_right": "/control/gripR",
            }
        ],
    )

    return LaunchDescription(
        [
            feedback_topic_arg,
            update_topic_arg,
            publish_rate_arg,
            real_hardware_node,
            robot_mode_initializer,
            planner_node,
            moveit_demo,
            moveit_bridge,
        ]
    )
