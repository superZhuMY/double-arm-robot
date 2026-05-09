#!/usr/bin/env bash
set -euo pipefail

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SETUP_FILE="${WS_DIR}/devel/setup.bash"
LOG_DIR="${WS_DIR}/runtime_logs"

run_ros() {
  bash -lc "cd '${WS_DIR}'; source '${SETUP_FILE}'; $*"
}

die() {
  echo "ERROR: $*" >&2
  exit 1
}

require_workspace() {
  [ -f "${SETUP_FILE}" ] || die "Missing ${SETUP_FILE}. Run catkin_make in ${WS_DIR} first."
  run_ros "rospack find jaka_controller_tcp >/dev/null" || die "Package jaka_controller_tcp not found. Check that the workspace is built and sourced."
  run_ros "rospack find Double_arm_robot_moveit >/dev/null" || die "Package Double_arm_robot_moveit not found. Check that the workspace is built and sourced."
  run_ros "rospack find moveit_simple_controller_manager >/dev/null" || \
    die "Missing moveit_simple_controller_manager. Install it with: sudo apt-get install ros-noetic-moveit-simple-controller-manager"
}

open_terminal() {
  local title="$1"
  local command="$2"
  local log_file="${LOG_DIR}/${title}.log"

  mkdir -p "${LOG_DIR}"
  : > "${log_file}"
  local shell_cmd="export LANG=C.UTF-8 LC_ALL=C.UTF-8; cd '${WS_DIR}'; source '${SETUP_FILE}'; stdbuf -oL -eL ${command} 2>&1 | stdbuf -oL -eL tee -a '${log_file}'; exec bash"

  if command -v gnome-terminal >/dev/null 2>&1; then
    if gnome-terminal --title="${title}" -- bash -lc "${shell_cmd}" >/dev/null 2>>"${log_file}"; then
      echo "Opened terminal: ${title}"
      return 0
    fi
  elif command -v xterm >/dev/null 2>&1; then
    if xterm -T "${title}" -e bash -lc "${shell_cmd}" >/dev/null 2>>"${log_file}" & then
      echo "Opened terminal: ${title}"
      return 0
    fi
  elif command -v konsole >/dev/null 2>&1; then
    if konsole --new-tab --title "${title}" -e bash -lc "${shell_cmd}" >/dev/null 2>>"${log_file}" & then
      echo "Opened terminal: ${title}"
      return 0
    fi
  fi

  echo "Could not open a terminal, starting in background: ${title}"
  echo "Log file: ${log_file}"
  nohup bash -lc "export LANG=C.UTF-8 LC_ALL=C.UTF-8; cd '${WS_DIR}'; source '${SETUP_FILE}'; stdbuf -oL -eL ${command}" >>"${log_file}" 2>&1 &
  echo "${title} PID: $!"
}

stop_old_system() {
  echo "Cleaning old double-arm ROS processes..."

  ps -eo pid=,cmd= | while read -r pid cmd; do
    case "${cmd}" in
      *roslaunch*jaka_controller_tcp*double_arm*|*roslaunch*Double_arm_robot_moveit*|*roslaunch*moveit_exe*manual_demo.launch*)
        echo "  Stopping roslaunch ${pid}: ${cmd}"
        kill "${pid}" >/dev/null 2>&1 || true
        ;;
    esac
  done

  sleep 1

  local nodes
  nodes="$(run_ros "rosnode list 2>/dev/null" || true)"
  [ -n "${nodes}" ] || return 0

  printf '%s\n' "${nodes}" | while IFS= read -r node; do
    case "${node}" in
      /move_group|/joint_state_publisher|/robot_state_publisher|/double_arm_robot_state_publisher|/jaka_controller_L|/jaka_controller_R|/jaka_send_read_node_zong_L|/jaka_send_read_node_zong_R|/rviz_*)
        echo "  Stopping node ${node}"
        run_ros "rosnode kill '${node}' >/dev/null 2>&1" || true
        ;;
    esac
  done
}

wait_for_ros_master() {
  local timeout="${1:-20}"
  local elapsed=0

  while [ "${elapsed}" -lt "${timeout}" ]; do
    if run_ros "rosnode list >/dev/null 2>&1"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  die "Timed out waiting for ROS master. Check whether roslaunch started successfully."
}

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-20}"
  local elapsed=0

  while [ "${elapsed}" -lt "${timeout}" ]; do
    if run_ros "rostopic list 2>/dev/null | grep -qx '${topic}'"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  die "Timed out waiting for topic: ${topic}"
}

wait_for_subscriber() {
  local topic="$1"
  local timeout="${2:-20}"
  local elapsed=0

  while [ "${elapsed}" -lt "${timeout}" ]; do
    if run_ros "rostopic info '${topic}' 2>/dev/null | awk '/Subscribers:/ {seen=1; next} seen && /^ \\* / {found=1} END {exit found ? 0 : 1}'"; then
      return 0
    fi
    sleep 1
    elapsed=$((elapsed + 1))
  done

  die "${topic} has no subscribers. The hardware bridge may not be running, or serial/Modbus initialization failed."
}

print_status() {
  echo
  echo "Current key ROS status:"
  run_ros "rosnode list | grep -E 'move_group|jaka_controller|jaka_send|robot_state|joint_state|rviz' || true"
  echo
  run_ros "rostopic info /command_L || true"
  echo
  run_ros "rostopic info /command_R || true"
  echo
  run_ros "rosparam get /move_group/controller_list 2>/dev/null || true"
}

require_workspace
stop_old_system

open_terminal "double_arm_hardware_bridge" \
  "roslaunch jaka_controller_tcp double_arm_hardware_bridge.launch"

wait_for_ros_master 20
wait_for_topic "/Double_arm_robot/l_arm/follow_joint_trajectory/status" 30
wait_for_topic "/Double_arm_robot/r_arm/follow_joint_trajectory/status" 30
wait_for_topic "/command_L" 30
wait_for_topic "/command_R" 30
wait_for_subscriber "/command_L" 30
wait_for_subscriber "/command_R" 30
wait_for_topic "/joint_states" 30

open_terminal "double_arm_moveit" \
  "roslaunch jaka_controller_tcp double_arm_moveit.launch moveit_controller_manager:=simple"

sleep 3
print_status

echo
echo "Startup complete. After planning/execution, check the hardware_bridge window for motion-command logs or motor write errors."
