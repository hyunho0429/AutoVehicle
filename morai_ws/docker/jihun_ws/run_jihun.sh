#!/usr/bin/env bash
# Jihun's fixed-route driving entry point; reuse the common implementations.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
COMMON="$SCRIPT_DIR/../final_ws"
ACTION="${1:-show}"
[[ $# -eq 0 ]] || shift
usage() {
  echo 'Usage: bash run_jihun.sh {show|monitor|drive|diagnose|start|stop|install [--config]|approach smooth|speed KMH|clearance METERS|lane on/off|turn safe/mission|models|view|rviz}'
}
case "$ACTION" in
  show|monitor|drive|diagnose)
    [[ $# -eq 0 ]] || { usage >&2; exit 2; }
    exec bash "$COMMON/run_test.sh" "$ACTION" 2
    ;;
  start|stop)
    [[ $# -eq 0 ]] || { usage >&2; exit 2; }
    exec bash "$COMMON/run_highway.sh" "$ACTION"
    ;;
  install)
    exec bash "$COMMON/install_curvature_signal.sh" "$@"
    ;;
  approach|speed|clearance|lane|turn|models|view|rviz)
    exec bash "$COMMON/run_test.sh" "$ACTION" "$@"
    ;;
  help|-h|--help) usage ;;
  *) usage >&2; exit 2 ;;
esac
