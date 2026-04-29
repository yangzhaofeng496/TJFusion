from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    fabric_share = get_package_share_directory("marvin_fabric")
    marvin_desc_share = get_package_share_directory("marvin_description")
    config = os.path.join(fabric_share, "config", "robot_param_m6.yaml")

    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(marvin_desc_share, "launch", "description_only_m6.launch.py")
        ),
        launch_arguments={
            "use_rviz": "false",
            "use_gui": "false",
        }.items(),
    )

    planner_node = Node(
        package="marvin_fabric",
        executable="planner_node",
        name="planner_node",
        parameters=[config],
        output="screen",
        arguments=["--ros-args", "--log-level", "INFO"],
    )

    offline_feedback = Node(
        package="marvin_fabric",
        executable="fabric_offline_feedback.py",
        name="fabric_offline_feedback",
        output="screen",
        parameters=[{"rate_hz": 100.0}],
    )

    start_rviz_arg = DeclareLaunchArgument(
        "start_rviz",
        default_value="true",
        description="Start RViz2 for target dragging",
    )

    target_markers = Node(
        package="marvin_fabric",
        executable="fabric_target_markers.py",
        name="fabric_target_markers",
        output="screen",
        parameters=[
            {
                "frame_id": "base_link",
                "publish_rate_hz": 30.0,
                "left_init": [0.45, 0.25, 1.10],
                "right_init": [0.45, -0.25, 1.10],
            }
        ],
    )

    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("start_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
    )

    return LaunchDescription(
        [
            start_rviz_arg,
            description_launch,
            planner_node,
            offline_feedback,
            target_markers,
            rviz_node,
        ]
    )
