from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("moveit_m6"), "launch", "demo.launch.py"])
        )
    )

    # Initialize fake joint state to SRDF "home" so RViz starts from home pose.
    set_home_once = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "topic",
                    "pub",
                    "--once",
                    "/joint_states",
                    "sensor_msgs/msg/JointState",
                    "{name: ['Joint1_L','Joint2_L','Joint3_L','Joint4_L','Joint5_L','Joint6_L','Joint7_L','Joint1_R','Joint2_R','Joint3_R','Joint4_R','Joint5_R','Joint6_R','Joint7_R','left_gripper_left_finger_joint','left_gripper_right_finger_joint','right_gripper_left_finger_joint','right_gripper_right_finger_joint'], position: [2.61479,-1.59871,-1.85433,-2.27467,-0.216093,-0.304625,-0.203511,-2.62467,-1.60188,1.82629,-2.28101,0.213259,-0.316744,0.226068,0.0,0.0,0.0,0.0]}",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription([demo_launch, set_home_once])
