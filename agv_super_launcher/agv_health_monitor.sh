#!/usr/bin/env bash
set -u

CONTAINER="ros_agv_serial"
HOST_SHARE="$HOME/agv_share"
JSON_FILE="$HOST_SHARE/latest.json"

check_container() {
  if docker ps --format "{{.Names}}" | grep -q "^$CONTAINER$"; then
    echo "CONTAINER: OK"
  else
    echo "CONTAINER: FAIL"
  fi
}

check_json() {
  if [[ -f "$JSON_FILE" ]]; then
    AGE=$(( $(date +%s) - $(stat -c %Y "$JSON_FILE") ))
    if [[ "$AGE" -le 2 ]]; then
      echo "latest.json: OK"
    else
      echo "latest.json: STALE"
    fi
  else
    echo "latest.json: MISSING"
  fi
}

check_scan_alive() {
  if docker exec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /scan --once >/dev/null" 2>/dev/null; then
    echo "/scan: OK"
  else
    echo "/scan: FAIL"
    FAIL=1
  fi
}

check_topic() {
  local topic=$1
  if docker exec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && ros2 topic list" 2>/dev/null | grep -q "$topic"; then
    echo "$topic: OK"
  else
    echo "$topic: FAIL"; [[ "$topic" == "/tf" ]] && FAIL=1
  fi
}


FAIL=0

echo "==== AGV HEALTH MONITOR ===="

check_container
check_json

check_scan_alive
check_topic "/detections"
check_topic "/cmd_vel_safe"
check_topic "/odometry/filtered"
check_topic "/tf"

echo "==== END ===="

exit $FAIL
