#!/bin/bash
set -euo pipefail

WS="/ros2_ws"
SESSION_MAIN="marvin"
ACTION_ZMQ_ADDR="${ACTION_ZMQ_ADDR:-tcp://127.0.0.1:8899}"
ACTION_ZMQ_TOPIC="${ACTION_ZMQ_TOPIC:-/action}"

# ---------- 通用：每个 pane 先 source ----------
PRELUDE="cd ${WS} && source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash"
SET_ROS_DOMAIN_ID="export ROS_DOMAIN_ID=10"

# ---------- 修改 robot_ip（在 launch 前生效） ----------
YAML="${WS}/install/marvin_ros_control/share/marvin_ros_control/config/robot_param_m6.yaml"
NEW_IP="${1:-${ROBOT_IP:-}}"

if [[ -n "${NEW_IP}" ]]; then
  cp -a "${YAML}" "${YAML}.bak.$(date +%Y%m%d_%H%M%S)" 2>/dev/null || true
  sed -i -E "s/^([[:space:]]*robot_ip:[[:space:]]*).*/\1${NEW_IP}/" "${YAML}"
  echo "[OK] robot_ip updated -> ${NEW_IP}"
else
  echo "[INFO] robot_ip not changed (pass IP as arg1 or set ROBOT_IP)"
fi

# 关闭旧 session
tmux kill-session -t "${SESSION_MAIN}" 2>/dev/null || true

# ========== Session 1: marvin ==========
tmux new-session -d -s "${SESSION_MAIN}" -n "MAIN" bash

tmux set-option -t "${SESSION_MAIN}" -g pane-border-status top
tmux set-option -t "${SESSION_MAIN}" -g pane-border-format "#{pane_title}"

# 布局：左侧上下；左上分左右；右侧分上下
tmux split-window -v -t "${SESSION_MAIN}:0" bash
tmux select-pane -t "${SESSION_MAIN}:0.0"
tmux split-window -h -t "${SESSION_MAIN}:0.0" bash
tmux split-window -v -t "${SESSION_MAIN}:0.1" bash
tmux split-window -v -t "${SESSION_MAIN}:0.3" bash

tmux select-pane -t "${SESSION_MAIN}:0.0" -T "PLANNER"
tmux select-pane -t "${SESSION_MAIN}:0.1" -T "GRIPPER"
tmux select-pane -t "${SESSION_MAIN}:0.2" -T "CONTROL"
tmux select-pane -t "${SESSION_MAIN}:0.3" -T "TASK_MANAGER"
tmux select-pane -t "${SESSION_MAIN}:0.4" -T "ACTION_BRIDGE"

tmux send-keys -t "${SESSION_MAIN}:0.0" "bash -lc '${SET_ROS_DOMAIN_ID}; ${PRELUDE}; ros2 launch marvin_fabric planner_m6.launch.py'" C-m
tmux send-keys -t "${SESSION_MAIN}:0.1" "bash -lc '${SET_ROS_DOMAIN_ID}; ${PRELUDE}; sleep 5; ros2 launch dm_gripper_py dm_gripper.launch.py'" C-m
tmux send-keys -t "${SESSION_MAIN}:0.3" "bash -lc '${SET_ROS_DOMAIN_ID}; ${PRELUDE}; sleep 8 && python3 ${WS}/src/marvin_fabric/scripts/world/test_task_manager_dynamic0323.py'" C-m
tmux send-keys -t "${SESSION_MAIN}:0.2" "bash -lc '${SET_ROS_DOMAIN_ID}; ${PRELUDE}; ros2 run marvin_fabric robot_mode_initializer.py --ros-args -p desired_mode:=3 -p max_retries:=0'" C-m
tmux send-keys -t "${SESSION_MAIN}:0.4" "bash -lc '${SET_ROS_DOMAIN_ID}; ${PRELUDE}; python3 /scripts/zmq2ros.py --zmq_addr ${ACTION_ZMQ_ADDR} --zmq_topic ${ACTION_ZMQ_TOPIC}'" C-m

tmux select-layout -t "${SESSION_MAIN}:0" tiled

# 进入主 session
tmux attach -t "${SESSION_MAIN}"
