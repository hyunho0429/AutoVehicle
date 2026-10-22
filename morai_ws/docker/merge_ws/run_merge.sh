#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
COMMON="$SCRIPT_DIR/../final_ws"
ACTION="${1:-show}"
[[ $# -eq 0 ]] || shift
if [[ "$ACTION" == init ]]; then
  [[ $# -eq 0 ]] || exit 2
  for name in highway.env highway-test.env; do
    if [[ ! -f "$SCRIPT_DIR/$name" ]]; then
      if [[ -f "$COMMON/$name" ]]; then
        cp -p "$COMMON/$name" "$SCRIPT_DIR/$name"
      else
        cp "$SCRIPT_DIR/$name.example" "$SCRIPT_DIR/$name"
      fi
      if [[ "$name" == highway.env ]]; then
        sed -i '/^CONTAINER_NAME=/d; /^ROUNDABOUT_ENABLED=/d' "$SCRIPT_DIR/$name"
        printf '\nCONTAINER_NAME=morai-merge-ws-net10\nROUNDABOUT_ENABLED=false\n' >> "$SCRIPT_DIR/$name"
      fi
    fi
  done
  echo 'merge_ws settings created. Existing IP/sensor/speed settings were inherited when available.'
  echo 'Container: morai-merge-ws-net10. Roundabout: off until its CSV is supplied.'
  exit 0
fi
[[ -f "$SCRIPT_DIR/highway.env" && -f "$SCRIPT_DIR/highway-test.env" ]] || {
  echo 'Run bash run_merge.sh init first.' >&2; exit 2;
}
export MORAI_SETTINGS_DIR="$SCRIPT_DIR"
export MORAI_PROFILE2_LAUNCH=merge_ws_drive.launch
source "$SCRIPT_DIR/highway.env"
case "$ACTION" in
  show|monitor|drive|diagnose)
    [[ $# -eq 0 ]] || exit 2
    exec bash "$COMMON/run_test.sh" "$ACTION" 2
    ;;
  start|stop|shell|build)
    [[ $# -eq 0 ]] || exit 2
    exec bash "$COMMON/run_highway.sh" "$ACTION"
    ;;
  install) exec bash "$COMMON/install_curvature_signal.sh" "$@" ;;
  speed|clearance|approach|lane|turn|models|view|rviz)
    exec bash "$COMMON/run_test.sh" "$ACTION" "$@" ;;
  validate)
    exec python3 "$SCRIPT_DIR/validate_roundabout.py" --allow-empty ;;
  roundabout)
    [[ $# -eq 1 && "$1" =~ ^(on|off)$ ]] || { echo 'Usage: bash run_merge.sh roundabout on/off' >&2; exit 2; }
    value=false
    if [[ "$1" == on ]]; then
      python3 "$SCRIPT_DIR/validate_roundabout.py"
      value=true
    fi
    cp -p "$SCRIPT_DIR/highway.env" "$SCRIPT_DIR/highway.env.bak"
    sed -i '/^ROUNDABOUT_ENABLED=/d' "$SCRIPT_DIR/highway.env"
    printf '\nROUNDABOUT_ENABLED=%s\n' "$value" >> "$SCRIPT_DIR/highway.env"
    echo "ROUNDABOUT_ENABLED=$value saved. Restart the launch to apply."
    ;;
  avoidance|avoidance-policy)
    key=OBSTACLE_AVOIDANCE_ENABLED
    if [[ "$ACTION" == avoidance ]]; then
      [[ $# -eq 1 && "$1" =~ ^(on|off)$ ]] || { echo 'Usage: bash run_merge.sh avoidance on/off' >&2; exit 2; }
      value=false; [[ "$1" == off ]] || value=true
    else
      key=AVOIDANCE_FAILURE_POLICY
      [[ $# -eq 1 && "$1" =~ ^(reference|stop)$ ]] || { echo 'Usage: bash run_merge.sh avoidance-policy reference/stop' >&2; exit 2; }
      value="$1"
    fi
    cp -p "$SCRIPT_DIR/highway.env" "$SCRIPT_DIR/highway.env.bak"
    sed -i "/^${key}=/d" "$SCRIPT_DIR/highway.env"
    printf '\n%s=%s\n' "$key" "$value" >> "$SCRIPT_DIR/highway.env"
    echo "$key=$value saved. Restart the launch to apply."
    ;;
  *) echo 'Usage: bash run_merge.sh {init|show|monitor|drive|diagnose|start|stop|install --config|shell|validate|roundabout on/off|avoidance on/off|avoidance-policy reference/stop|speed KMH|approach smooth|lane on/off}' >&2; exit 2 ;;
esac
