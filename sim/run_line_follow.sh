#!/bin/bash
# Launches the full closed-loop line-following stack on the MuJoCo sim.
#
# Components (each in its own process, logs in /tmp/cavalier-sim-logs/):
#   1. mujoco_bridge          — physics + /safety/command sub + sim feedback
#   2. safety_node            — Cavalier safety mux (UNMODIFIED)
#   3. automation_command_mux — Cavalier source mux (UNMODIFIED)
#   4. line_follower_controller — Cavalier PD line follower (UNMODIFIED)
#   5. line_follow_runner     — fakes /orchestrator/active_task + /drop_off/mode
#
# Usage:
#   ./sim/run_line_follow.sh [--init-y 0.5] [--init-yaw 0.0] [--duration 30]
#
# The MuJoCo viewer comes up on $DISPLAY (default :1). ESC the viewer
# to exit early; otherwise the script tears everything down after
# `duration` seconds.

set -eo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
BRIDGE_WS="$ROOT/sim/bridge_ws"
LOG_DIR=/tmp/cavalier-sim-logs
mkdir -p "$LOG_DIR"

INIT_Y="0.5"
INIT_YAW="0.0"
DURATION="30"
HEADLESS="${MUJOCO_HEADLESS:-0}"
while [ $# -gt 0 ]; do
  case "$1" in
    --init-y) INIT_Y="$2"; shift 2;;
    --init-yaw) INIT_YAW="$2"; shift 2;;
    --duration) DURATION="$2"; shift 2;;
    --headless) HEADLESS=1; shift;;
    *) echo "Unknown arg: $1" >&2; exit 2;;
  esac
done

source /opt/ros/jazzy/setup.bash
source "$BRIDGE_WS/install/setup.bash"

CFG="$BRIDGE_WS/install/drop_off/share/drop_off/config/line_follower.yaml"

PIDS=()
cleanup() {
  echo
  echo "tearing down..."
  for pid in "${PIDS[@]}"; do kill "$pid" 2>/dev/null || true; done
  sleep 0.3
  # `ros2 run` daemonizes the actual node — kill by pattern to catch orphans.
  pkill -9 -f "automation_command_mux|safety_node|line_follower_controller|automation_cmd_adapter|line_follow_runner|mujoco_driver_node" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "== starting mujoco_bridge (init_y=$INIT_Y, init_yaw=$INIT_YAW)"
SIM_INIT_Y="$INIT_Y" SIM_INIT_YAW_RAD="$INIT_YAW" MUJOCO_HEADLESS="$HEADLESS" DISPLAY="${DISPLAY:-:1}" \
  python3 -m mujoco_bridge.mujoco_driver_node \
  > "$LOG_DIR/bridge.log" 2>&1 &
PIDS+=($!)

# Wait for the bridge to come up (it advertises a topic when ready).
for _ in $(seq 1 50); do
  if ros2 topic list 2>/dev/null | grep -q '^/forklift/drive_feedback$'; then break; fi
  sleep 0.1
done

echo "== starting safety_node"
ros2 run forklift_driver safety_node > "$LOG_DIR/safety_node.log" 2>&1 &
PIDS+=($!)

echo "== starting automation_command_mux (routed through line_follower adapter)"
ros2 run forklift_driver automation_command_mux --ros-args \
  -p source_topics:='[/automation/command_sources/nav2,/automation/command_sources/line_follower_cmd,/automation/command_sources/profiled_fork_height,/automation/command_sources/fork_height]' \
  > "$LOG_DIR/auto_mux.log" 2>&1 &
PIDS+=($!)

echo "== starting automation_cmd_adapter (AutomationCommand -> ForkliftDirectCommand)"
python3 -m mujoco_bridge.automation_cmd_adapter > "$LOG_DIR/adapter.log" 2>&1 &
PIDS+=($!)

echo "== starting line_follower_controller"
ros2 run drop_off line_follower_controller --ros-args --params-file "$CFG" \
  > "$LOG_DIR/line_follower.log" 2>&1 &
PIDS+=($!)

echo "== starting safety heartbeat (status=Safe so safety_node permits AUTO)"
( while true; do
    ros2 topic pub --once /teleop/safety_status std_msgs/msg/UInt8 '{data: 0}' \
      > /dev/null 2>&1 || break
    sleep 0.3
  done ) > "$LOG_DIR/heartbeat.log" 2>&1 &
PIDS+=($!)

# Give the controller a moment to declare its parameters before activation.
sleep 1.5

echo "== starting line_follow_runner (activator)  — duration=${DURATION}s"
python3 "$ROOT/sim/line_follow_runner.py" --mode follow_line --duration "$DURATION"

echo "done"
