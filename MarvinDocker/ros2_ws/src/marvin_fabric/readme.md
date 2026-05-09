# marvin_fabric (Real Hardware Minimal Mode)

This package is now configured for a minimal real-hardware execution path with MoveIt.

## Kept Runtime Nodes

1. `marvin_ros_control/marvin_robot_node`  
2. `marvin_fabric/robot_mode_initializer.py`  
3. `marvin_fabric/planner_node`  
4. `moveit_m6` demo launch (RViz + MoveIt)  
5. `marvin_fabric/moveit_goal_bridge.py`  

## Launch

```bash
ros2 launch marvin_fabric fabric_moveit_bridge.launch.py
```

## Execution Chain

1. In RViz, click **Execute** in MoveIt.
2. `moveit_goal_bridge` detects execute stage and publishes:
   - `/control/target_poseL`
   - `/control/target_poseR`
   - `/control/gripL`
   - `/control/gripR`
3. `planner_node` consumes `/control/target_poseL/R` and publishes:
   - `/control/joint_cmd_A`
   - `/control/joint_cmd_B`
4. `marvin_robot_node` consumes `/control/joint_cmd_A/B` and sends motion to hardware.

## Quick Checks

```bash
ros2 topic hz /control/target_poseL
ros2 topic hz /control/target_poseR
ros2 topic hz /control/joint_cmd_A
ros2 topic hz /control/joint_cmd_B
ros2 topic echo /info/arm_state --once
```

## Notes

- `Plan` stage does not drive hardware control topics.
- Real control starts at `Execute` stage.
