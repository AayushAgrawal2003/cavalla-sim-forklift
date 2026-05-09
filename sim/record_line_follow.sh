#!/bin/bash
# Records the closed-loop line-following demo to a single mp4.
#
# Brings up the full Cavalier control stack (safety_node, mux, adapter,
# line_follower_controller, activator) plus the bridge — then runs the
# recorder which subscribes to /safety/command and renders to mp4.
#
# Usage: ./sim/record_line_follow.sh [out.mp4] [--duration 30]

set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BRIDGE_WS="$ROOT/sim/bridge_ws"
LOG_DIR=/tmp/cavalier-sim-logs
mkdir -p "$LOG_DIR"

OUT="${1:-$ROOT/forklift_line_follow.mp4}"
[[ "$OUT" == --* ]] && OUT="$ROOT/forklift_line_follow.mp4"
DURATION=30
INIT_Y=0.5
INIT_YAW=0.0

while [ $# -gt 0 ]; do
  case "$1" in
    --duration) DURATION="$2"; shift 2;;
    --init-y) INIT_Y="$2"; shift 2;;
    --init-yaw) INIT_YAW="$2"; shift 2;;
    *) shift;;
  esac
done

source /opt/ros/jazzy/setup.bash
source "$BRIDGE_WS/install/setup.bash"

CFG="$BRIDGE_WS/install/drop_off/share/drop_off/config/line_follower.yaml"

cleanup() {
  pkill -9 -f "automation_command_mux|safety_node|line_follower_controller|automation_cmd_adapter|line_follow_runner|mujoco_driver_node" 2>/dev/null || true
}
trap cleanup EXIT INT TERM
cleanup
sleep 0.5

echo "== bridge"
SIM_INIT_Y="$INIT_Y" SIM_INIT_YAW_RAD="$INIT_YAW" MUJOCO_HEADLESS=1 \
  python3 -m mujoco_bridge.mujoco_driver_node > "$LOG_DIR/bridge.log" 2>&1 &
sleep 1.5

echo "== safety_node + mux + adapter + line_follower"
ros2 run forklift_driver safety_node > "$LOG_DIR/safety_node.log" 2>&1 &
ros2 run forklift_driver automation_command_mux --ros-args \
  -p source_topics:='[/automation/command_sources/nav2,/automation/command_sources/line_follower_cmd,/automation/command_sources/profiled_fork_height,/automation/command_sources/fork_height]' \
  > "$LOG_DIR/auto_mux.log" 2>&1 &
python3 -m mujoco_bridge.automation_cmd_adapter > "$LOG_DIR/adapter.log" 2>&1 &
ros2 run drop_off line_follower_controller --ros-args --params-file "$CFG" \
  > "$LOG_DIR/line_follower.log" 2>&1 &

echo "== heartbeat"
( while true; do
    ros2 topic pub --once /teleop/safety_status std_msgs/msg/UInt8 '{data: 0}' \
      > /dev/null 2>&1 || break
    sleep 0.3
  done ) > "$LOG_DIR/heartbeat.log" 2>&1 &

sleep 2

echo "== activator (background, $DURATION s)"
python3 "$ROOT/sim/line_follow_runner.py" --mode follow_line --duration "$DURATION" \
  > "$LOG_DIR/runner.log" 2>&1 &
sleep 1.5

echo "== recorder ($DURATION s) → $OUT"
python3 "$ROOT/sim/record_line_follow.py" \
  --duration "$DURATION" --init-y "$INIT_Y" --init-yaw "$INIT_YAW" \
  "$OUT"

echo "done — $OUT"
ls -la "$OUT"
