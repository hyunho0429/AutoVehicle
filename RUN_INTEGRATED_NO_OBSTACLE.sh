#!/usr/bin/env bash
set -e
WS="$(cd -- "$(dirname -- "$0")" && pwd)"
source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"
MODEL="$WS/src/detection/camera_perception/models/highway_best.pt"
if [ ! -f "$MODEL" ]; then
  echo 'Missing highway_best.pt. Run: python3 docker/final_ws/fetch_highway_model.py'
  exit 1
fi
exec roslaunch morai_bringup integrated_no_obstacle_v61.launch workspace_path:="$WS" "$@"
