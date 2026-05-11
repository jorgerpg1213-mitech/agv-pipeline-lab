#!/usr/bin/env bash
set -u

CONTAINER="ros_agv_serial"

check_scan_alive() {
  if docker exec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && timeout 5 ros2 topic echo /scan --once >/dev/null"; then
    echo "/scan: ALIVE"
  else
    echo "/scan: DEAD"
    return 1
  fi
}

echo "==== AGV HEALTH MONITOR V2 ===="

check_scan_alive || FAIL=1

echo "==== END ===="

exit ${FAIL:-0}
