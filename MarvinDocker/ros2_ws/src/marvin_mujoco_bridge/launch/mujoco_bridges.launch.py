from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    state_endpoint_arg = DeclareLaunchArgument(
        "state_endpoint",
        default_value="tcp://192.168.1.99:5557",
        description="MuJoCo ZMQ PUB endpoint for state stream.",
    )
    action_endpoint_arg = DeclareLaunchArgument(
        "action_endpoint",
        default_value="tcp://192.168.1.99:5555",
        description="MuJoCo ZMQ REQ/REP endpoint for control commands.",
    )
    zmq_topic_arg = DeclareLaunchArgument(
        "zmq_topic",
        default_value="",
        description="ZMQ SUB topic filter for state bridge. Empty means receive all.",
    )
    log_raw_zmq_arg = DeclareLaunchArgument(
        "log_raw_zmq",
        default_value="false",
        description="Print raw ZMQ frames in state bridge.",
    )
    log_on_receive_arg = DeclareLaunchArgument(
        "log_on_receive",
        default_value="false",
        description="Print parsed state summary in state bridge.",
    )
    relative_arg = DeclareLaunchArgument(
        "relative",
        default_value="false",
        description="Send set_positions with relative=true/false in action bridge.",
    )
    send_hold_on_cancel_arg = DeclareLaunchArgument(
        "send_hold_on_cancel",
        default_value="true",
        description="Send hold command when trajectory goal is canceled.",
    )

    state_node = Node(
        package="marvin_mujoco_bridge",
        executable="mujoco_state_bridge",
        name="mujoco_state_bridge",
        output="screen",
        parameters=[
            {
                "endpoint": LaunchConfiguration("state_endpoint"),
                "zmq_topic": LaunchConfiguration("zmq_topic"),
                "log_raw_zmq": LaunchConfiguration("log_raw_zmq"),
                "log_on_receive": LaunchConfiguration("log_on_receive"),
            }
        ],
    )

    action_node = Node(
        package="marvin_mujoco_bridge",
        executable="mujoco_action_bridge",
        name="mujoco_action_bridge",
        output="screen",
        parameters=[
            {
                "endpoint": LaunchConfiguration("action_endpoint"),
                "relative": LaunchConfiguration("relative"),
                "send_hold_on_cancel": LaunchConfiguration("send_hold_on_cancel"),
            }
        ],
    )

    return LaunchDescription(
        [
            state_endpoint_arg,
            action_endpoint_arg,
            zmq_topic_arg,
            log_raw_zmq_arg,
            log_on_receive_arg,
            relative_arg,
            send_hold_on_cancel_arg,
            state_node,
            action_node,
        ]
    )
