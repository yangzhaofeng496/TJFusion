#!/bin/bash
set -u

source /ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=10

READY_SRV="/control/set_ready"
MODE_SRV="/control/set_mode"
TARGET_MODE="${1:-3}"
MAX_WAIT_SEC="${MAX_WAIT_SEC:-120}"
SLEEP_SEC="${SLEEP_SEC:-1}"

wait_for_service() {
  local srv="$1"
  local t=0
  while true; do
    if ros2 service type "$srv" >/dev/null 2>&1; then
      echo "[ServiceCall] service ready: $srv"
      return 0
    fi
    sleep "$SLEEP_SEC"
    t=$((t + SLEEP_SEC))
    if (( t >= MAX_WAIT_SEC )); then
      echo "[ServiceCall] timeout waiting for service: $srv"
      return 1
    fi
  done
}

call_until_success() {
  local cmd="$1"
  local name="$2"
  local t=0
  while true; do
    local out
    out="$(eval "$cmd" 2>&1 || true)"
    echo "$out"
    if echo "$out" | grep -q "success=True"; then
      echo "[ServiceCall] $name success"
      return 0
    fi
    sleep "$SLEEP_SEC"
    t=$((t + SLEEP_SEC))
    if (( t >= MAX_WAIT_SEC )); then
      echo "[ServiceCall] timeout waiting for $name success"
      return 1
    fi
  done
}

wait_for_service "$READY_SRV" || exit 1
wait_for_service "$MODE_SRV" || exit 1

call_until_success "ros2 service call $READY_SRV std_srvs/srv/Trigger '{}'" "set_ready" || exit 1
call_until_success "ros2 service call $MODE_SRV marvin_msgs/srv/Int '{data: $TARGET_MODE}'" "set_mode($TARGET_MODE)" || exit 1

# publish a few times to avoid startup race on subscribers
for _ in 1 2 3; do
  ros2 topic pub -1 /control/gripL std_msgs/msg/Bool "{data: True}" >/dev/null 2>&1 || true
  ros2 topic pub -1 /control/gripR std_msgs/msg/Bool "{data: True}" >/dev/null 2>&1 || true
  sleep 0.1
done

echo "[ServiceCall] done"
