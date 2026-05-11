#!/usr/bin/env bash
set -u

CONTAINER="ros_agv_serial"
SHARE_DIR="$HOME/agv_share"
YOLO_FILE="$SHARE_DIR/latest.json"
ROS_STATUS_FILE="$SHARE_DIR/agv_status.json"
HOST_STATUS_FILE="$SHARE_DIR/agv_host_status.json"
ESCALATION_FLAG="$SHARE_DIR/escalation_required.flag"


LOG_DIR="$HOME/agv_logs/watchdog"
LOG_FILE="$LOG_DIR/host_watchdog.log"

INTERVAL=2
YOLO_MAX_AGE=10
ROS_STATUS_MAX_AGE=10

mkdir -p "$LOG_DIR" "$SHARE_DIR"

log() {
  echo "$(date '+%F %T') $*" | tee -a "$LOG_FILE"
}

file_age_ok() {
  local file="$1"
  local max_age="$2"

  [[ -f "$file" ]] || return 1

  local age
  age=$(( $(date +%s) - $(stat -c %Y "$file") ))

  [[ "$age" -le "$max_age" ]]
}

container_running() {
  docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true
}

yolo_running() {
  pgrep -af "yolo_gst_csi.py" >/dev/null 2>&1
}

write_status() {
  local state="$1"
  local fault="$2"
  local message="$3"
  local docker_state="$4"
  local yolo_proc="$5"
  local yolo_json="$6"
  local ros_json="$7"

  cat > "$HOST_STATUS_FILE" <<JSON
{
  "state": "$state",
  "fault": $fault,
  "message": "$message",
  "can_restart": false,
  "timestamp": "$(date -Iseconds)",
  "components": {
    "docker": "$docker_state",
    "yolo_process": "$yolo_proc",
    "latest_json": "$yolo_json",
    "ros_status_json": "$ros_json"
  }
}
JSON
}

log "AGV host watchdog pasivo iniciado. No apaga, no reinicia, no mata procesos."

while true; do
  STATE="RUNNING"
  FAULT="null"
  MESSAGE="Host watchdog pasivo monitoreando."
  DOCKER_STATE="OK"
  YOLO_PROC="OK"
  YOLO_JSON="OK"
  ROS_JSON="OK"

  container_running || DOCKER_STATE="FAIL"
  yolo_running || YOLO_PROC="FAIL"
  file_age_ok "$YOLO_FILE" "$YOLO_MAX_AGE" || YOLO_JSON="FAIL"
  file_age_ok "$ROS_STATUS_FILE" "$ROS_STATUS_MAX_AGE" || ROS_JSON="FAIL"

  if [[ "$DOCKER_STATE" == "FAIL" || "$YOLO_PROC" == "FAIL" || "$YOLO_JSON" == "FAIL" || "$ROS_JSON" == "FAIL" ]]; then
    STATE="DEGRADED"
    MESSAGE="Host watchdog detectó fallo externo. No ejecutó recovery, apagado ni kill."

    if [[ "$DOCKER_STATE" == "FAIL" ]]; then
      FAULT='"DOCKER_NOT_RUNNING"'
    elif [[ "$YOLO_PROC" == "FAIL" ]]; then
      FAULT='"YOLO_PROCESS_NOT_RUNNING"'
    elif [[ "$YOLO_JSON" == "FAIL" ]]; then
      FAULT='"YOLO_JSON_STALE_OR_MISSING"'
    elif [[ "$ROS_JSON" == "FAIL" ]]; then
      FAULT='"ROS_STATUS_STALE_OR_MISSING"'
    fi

    log "WARN state=$STATE fault=$FAULT docker=$DOCKER_STATE yolo_proc=$YOLO_PROC yolo_json=$YOLO_JSON ros_json=$ROS_JSON"
  else
    log "INFO state=$STATE docker=$DOCKER_STATE yolo_proc=$YOLO_PROC yolo_json=$YOLO_JSON ros_json=$ROS_JSON"
  fi
  if [[ -f "$ESCALATION_FLAG" ]]; then

    log "CRITICAL escalation flag detectada. Ejecutando safe shutdown."

    pkill -TERM -f "mega_super_launcher_host.sh"

    rm -f "$ESCALATION_FLAG"

  fi


  write_status "$STATE" "$FAULT" "$MESSAGE" "$DOCKER_STATE" "$YOLO_PROC" "$YOLO_JSON" "$ROS_JSON"
  sleep "$INTERVAL"
done
