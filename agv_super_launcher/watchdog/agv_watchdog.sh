#!/usr/bin/env bash
set -u

CONTAINER="ros_agv_serial"
SHARE_DIR="$HOME/agv_share"
STATUS_FILE="$SHARE_DIR/agv_status.json"
LOG_DIR="$HOME/agv_logs/watchdog"
LOG_FILE="$LOG_DIR/watchdog.log"
JSON_FILE="$SHARE_DIR/latest.json"

INTERVAL=2
YOLO_MAX_AGE=10

mkdir -p "$LOG_DIR" "$SHARE_DIR"

log() {
  echo "$(date '+%F %T') $*" | tee -a "$LOG_FILE"
}

json_status() {
  local state="$1"
  local fault="$2"
  local message="$3"
  local lidar="$4"
  local esp32="$5"
  local yolo="$6"
  local tf="$7"
  local can_restart="$8"

  cat > "$STATUS_FILE" <<JSON
{
  "state": "$state",
  "fault": $fault,
  "message": "$message",
  "can_restart": $can_restart,
  "timestamp": "$(date -Iseconds)",
  "components": {
    "lidar": "$lidar",
    "esp32": "$esp32",
    "yolo": "$yolo",
    "tf": "$tf"
  }
}
JSON
}

topic_alive() {
  local topic="$1"
  docker exec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash; source /root/agv_ws/install/setup.bash; timeout 2 ros2 topic echo $topic --once >/dev/null 2>&1"
}

json_fresh() {
  [[ -f "$JSON_FILE" ]] || return 1
  local age
  age=$(( $(date +%s) - $(stat -c %Y "$JSON_FILE") ))
  [[ "$age" -le "$YOLO_MAX_AGE" ]]
}

log "AGV watchdog pasivo iniciado. No apaga, no reinicia, no mata procesos."

while true; do
  LIDAR="OK"
  ESP32="OK"
  YOLO="OK"
  TF="OK"
  STATE="RUNNING"
  FAULT="null"
  MESSAGE="Sistema AGV operando normalmente."
  CAN_RESTART="true"

  topic_alive "/scan" || LIDAR="FAIL"
  topic_alive "/imu" || ESP32="FAIL"
  topic_alive "/odom" || ESP32="FAIL"
  topic_alive "/tf" || TF="FAIL"
  json_fresh || YOLO="FAIL"

  if [[ "$LIDAR" == "FAIL" || "$ESP32" == "FAIL" || "$YOLO" == "FAIL" || "$TF" == "FAIL" ]]; then
    STATE="DEGRADED"
    CAN_RESTART="false"
    MESSAGE="Watchdog pasivo detectó fallo. No se ejecutó recovery ni apagado."

    if [[ "$LIDAR" == "FAIL" ]]; then
      FAULT='"LIDAR_NO_SCAN"'
    elif [[ "$ESP32" == "FAIL" ]]; then
      FAULT='"ESP32_NO_DATA"'
    elif [[ "$YOLO" == "FAIL" ]]; then
      FAULT='"YOLO_STALE"'
    elif [[ "$TF" == "FAIL" ]]; then
      FAULT='"TF_NO_DATA"'
    fi

    log "WARN state=$STATE fault=$FAULT lidar=$LIDAR esp32=$ESP32 yolo=$YOLO tf=$TF"
  else
    log "INFO state=$STATE lidar=$LIDAR esp32=$ESP32 yolo=$YOLO tf=$TF"
  fi

  json_status "$STATE" "$FAULT" "$MESSAGE" "$LIDAR" "$ESP32" "$YOLO" "$TF" "$CAN_RESTART"
  sleep "$INTERVAL"
done
