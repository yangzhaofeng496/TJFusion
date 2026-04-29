# marvin_fabric

This package provides ROS 2 control functionalities for Marvin robots.

# control topics
1. left eef roation constraint
msg type:
```
 control/eef_constraint,  #example [1, 1, 1, 1, 1, 1] constrain rotation by three axis on each tcp.
```
2.
msg type:
```
 control/speed_scale, #[speed_scale_left, speed_scale_right] int value from 0 to 100.
```
control/target_poseL
control/target_poseR 
## Usage

Refer to the package documentation and launch files for usage instructions.

## MoveIt + Fabric Runtime Modes

The launch file:
```bash
ros2 launch marvin_fabric fabric_moveit_bridge.launch.py ...
```

supports the following runtime switches:

- `use_real_hardware`:
  - `true`: start `marvin_ros_control` hardware node and wait for real `/info/joint_feedback` before starting MoveIt.
  - `false`: use offline feedback node.
- `enable_fabric_control`:
  - `true`: start Fabric planner + MoveIt goal bridge (robot can be commanded through `/control/joint_cmd_A/B`).
  - `false`: feedback visualization only (no Fabric command publisher from this launch).
- `simulate_robot_motion` (only meaningful when `use_real_hardware:=false`):
  - `true`: publish sinusoidal mock joint feedback/state data.
  - `false`: publish static initial pose feedback/state data.

### Common launch commands

1. Offline, static feedback (visualization/debug):
```bash
ros2 launch marvin_fabric fabric_moveit_bridge.launch.py \
  use_real_hardware:=false \
  enable_fabric_control:=true \
  simulate_robot_motion:=false
```

2. Offline, dynamic mock robot data:
```bash
ros2 launch marvin_fabric fabric_moveit_bridge.launch.py \
  use_real_hardware:=false \
  enable_fabric_control:=true \
  simulate_robot_motion:=true
```

3. Real hardware, feedback-only (no control output from Fabric):
```bash
ros2 launch marvin_fabric fabric_moveit_bridge.launch.py \
  use_real_hardware:=true \
  enable_fabric_control:=false
```

4. Real hardware, Fabric control enabled:
```bash
ros2 launch marvin_fabric fabric_moveit_bridge.launch.py \
  use_real_hardware:=true \
  enable_fabric_control:=true
```

### Verify topic flow

Check command path:
```bash
ros2 topic info /control/joint_cmd_A -v
ros2 topic info /control/joint_cmd_B -v
```

Check feedback path:
```bash
ros2 topic echo /info/arm_state --once
ros2 topic hz /info/joint_feedback
ros2 topic hz /joint_states
```

Check MoveIt marker bridge inputs:
```bash
ros2 topic list | grep robot_interaction_interactive_marker_topic
ros2 topic hz /control/target_poseL
ros2 topic hz /control/target_poseR
```

## License

See [LICENSE](LICENSE) for details.


2025.10.18
