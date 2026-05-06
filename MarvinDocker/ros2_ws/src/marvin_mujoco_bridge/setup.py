from setuptools import find_packages, setup

package_name = "marvin_mujoco_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/mujoco_bridges.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yang",
    maintainer_email="yang@todo.todo",
    description="Bridge MoveIt2 JointTrajectory commands to a MuJoCo ZMQ service.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "marvin_mujoco_bridge = marvin_mujoco_bridge.bridge_node:main",
            "mujoco_state_bridge = marvin_mujoco_bridge.mujoco_state_bridge:main",
            "mujoco_action_bridge = marvin_mujoco_bridge.mujoco_action_bridge:main",
        ],
    },
)
