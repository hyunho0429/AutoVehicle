#!/usr/bin/env bash
set -euo pipefail
WS="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-monitor}"
if [[ $# -gt 0 ]]; then shift; fi
case "$MODE" in
  monitor) CONTROL=false ;;
  drive) CONTROL=true ;;
  *) printf 'Usage: bash RUN_INTEGRATED.sh [monitor|drive] [name:=value ...]\n' >&2; exit 2 ;;
esac
if [[ ! -f /opt/ros/noetic/setup.bash || ! -f "$WS/devel/setup.bash" ]]; then
  printf 'Use the ROS Noetic environment and build this workspace with catkin_make first: %s\n' "$WS" >&2
  exit 2
fi
set +u
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
set -u
exec roslaunch morai_bringup integrated_obstacle_v61.launch \
  workspace_path:="$WS" enable_control:="$CONTROL" "$@"
