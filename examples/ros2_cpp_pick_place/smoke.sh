#!/usr/bin/env bash
# Run inside an environment with ROS 2 Jazzy and control_msgs installed.
set -eo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
if [[ $# != 2 || ! -f "$1/parameters.hpp" || -e "$2" ]]; then
  echo 'Usage: smoke.sh GENERATED_HEADER_DIRECTORY NEW_WORKSPACE_DIRECTORY' >&2
  exit 2
fi
header=$(cd "$1" && pwd)
mkdir -p "$2/src"
work=$(cd "$2" && pwd)
cp -R "$repo/examples/ros2_cpp_pick_place" "$work/src/dparams_cpp_pick_place"
source /opt/ros/jazzy/setup.bash
cd "$work"
colcon build --event-handlers console_direct+ --cmake-args "-DDPARAMS_HEADER_DIR=$header"
source "$work/install/setup.bash"
# Exercise the missing-server path, then prove the same controller can retry.
ros2 run dparams_cpp_pick_place pick_place --ros-args -p action_server_timeout_sec:=2.0 > "$work/controller.log" 2>&1 & controller_pid=$!
motor_pid=''
cleanup(){ if [[ -n "$motor_pid" ]]; then kill "$motor_pid" 2>/dev/null || true; fi; kill "$controller_pid" 2>/dev/null || true; }
trap cleanup EXIT
publish_target(){ timeout 20s ros2 topic pub --once /pick_target geometry_msgs/msg/PoseStamped '{header: {frame_id: base_link}, pose: {position: {x: 0.25, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}'; }
if ! publish_target; then cat "$work/controller.log"; exit 1; fi
for _ in $(seq 1 10); do
  if grep -q 'Action server unavailable after' "$work/controller.log"; then break; fi
  sleep 1
done
if ! grep -q 'Action server unavailable after' "$work/controller.log"; then cat "$work/controller.log"; exit 1; fi
if grep -q 'State: APPROACH' "$work/controller.log"; then cat "$work/controller.log"; exit 1; fi
ros2 run dparams_cpp_pick_place motor_sim > "$work/motor.log" 2>&1 & motor_pid=$!
if ! timeout 20s bash -c 'until ros2 action info /arm_controller/follow_joint_trajectory 2>/dev/null | grep -q "Action servers: 1"; do sleep 1; done'; then
  cat "$work/controller.log" "$work/motor.log"; exit 1
fi
if ! publish_target; then cat "$work/controller.log" "$work/motor.log"; exit 1; fi
for _ in $(seq 1 30); do
  if grep -q 'Pick and place sequence complete' "$work/controller.log"; then
    cat "$work/controller.log"
    exit 0
  fi
  sleep 1
done
cat "$work/controller.log" "$work/motor.log"
exit 1
