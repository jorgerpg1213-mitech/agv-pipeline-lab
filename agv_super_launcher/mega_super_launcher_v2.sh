#!/usr/bin/env bash
set -u

CONTAINER="ros_agv_serial"
YOLO_SCRIPT="$HOME/yolo_gst_csi.py"
YOLO_VENV="$HOME/venvs/yolo_track"
HOST_SHARE="$HOME/agv_share"
JSON_FILE="$HOST_SHARE/latest.json"
LOG_ROOT="$HOME/agv_logs"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$LOG_ROOT/run_$RUN_ID"

YOLO_PID=""
ROS_PID=""
RVIZ_PID=""

RVIZ_CONFIG="$HOME/agv_share/agv_nav2_slam.rviz"

mkdir -p "$RUN_DIR"

LAUNCHER_LOG="$RUN_DIR/launcher.log"
YOLO_LOG="$RUN_DIR/yolo.log"
ROS_LOG="$RUN_DIR/ros.log"

log() {
  echo "[$(date '+%F %T')] $*" | tee -a "$LAUNCHER_LOG"
}

cleanup() {
  trap - INT TERM EXIT
  log "Cierre solicitado. Deteniendo procesos..."

  if [[ -n "${RVIZ_PID:-}" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi

  if [[ -n "${YOLO_PID:-}" ]] && kill -0 "$YOLO_PID" 2>/dev/null; then
    kill "$YOLO_PID" 2>/dev/null || true
    wait "$YOLO_PID" 2>/dev/null || true
  fi

  if [[ -n "${ROS_PID:-}" ]] && kill -0 "$ROS_PID" 2>/dev/null; then
    kill "$ROS_PID" 2>/dev/null || true
    wait "$ROS_PID" 2>/dev/null || true
  fi

  log "Limpieza ROS sin reiniciar contenedor."
  docker exec "$CONTAINER" bash -lc "pkill -INT -f 'ros2 launch agv_bringup system_nav2_slam.launch.py' || true" >>"$LAUNCHER_LOG" 2>&1 || true
  docker exec "$CONTAINER" bash -lc "pkill -TERM -f 'rplidar_node|slam_toolbox|nav2|ekf_node|esp32_bridge_node|odom_node|perception_node|safety_policy_node' || true" >>"$LAUNCHER_LOG" 2>&1 || true

  log "Launcher cerrado."
}

check_env() {
  if ! docker ps >/dev/null 2>&1; then
    log "ERROR: Docker no responde sin sudo."
    exit 1
  fi

  if [[ ! -f "$YOLO_SCRIPT" ]]; then
    log "ERROR: No existe YOLO_SCRIPT: $YOLO_SCRIPT"
    exit 1
  fi

  if [[ ! -d "$YOLO_VENV" ]]; then
    log "ERROR: No existe YOLO_VENV: $YOLO_VENV"
    exit 1
  fi

  if [[ ! -d "$HOST_SHARE" ]]; then
    log "ERROR: No existe HOST_SHARE: $HOST_SHARE"
    exit 1
  fi
}

start_container() {
  log "Arrancando/verificando contenedor: $CONTAINER"
  docker start "$CONTAINER" >>"$LAUNCHER_LOG" 2>&1 || {
    log "ERROR: No se pudo arrancar contenedor $CONTAINER"
    exit 1
  }
}

start_yolo() {
  log "Limpiando latest.json viejo si existe."
  rm -f "$JSON_FILE"

  log "Arrancando YOLO en host."
  (
    cd "$YOLO_VENV" || exit 1
    source bin/activate
    python "$YOLO_SCRIPT"
  ) >>"$YOLO_LOG" 2>&1 &
  YOLO_PID=$!
}

wait_for_yolo_json() {
  log "Esperando latest.json fresco..."
  READY=0
  for i in {1..30}; do
    if [[ -f "$JSON_FILE" ]]; then
      AGE=$(( $(date +%s) - $(stat -c %Y "$JSON_FILE") ))
      if [[ "$AGE" -le 2 ]]; then
        READY=1
        break
      fi
    fi
    sleep 1
  done

  if [[ "$READY" -ne 1 ]]; then
    log "ERROR: YOLO no generó latest.json fresco dentro del timeout."
    exit 1
  fi
}

start_ros() {
  log "latest.json fresco detectado. Arrancando ROS superlauncher dentro de Docker."

  docker exec "$CONTAINER" bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/agv_ws/install/setup.bash
  ros2 launch agv_bringup system_nav2_slam.launch.py
  ' >>"$ROS_LOG" 2>&1 &
  ROS_PID=$!
}

start_rviz() {
  log "Esperando 5 segundos antes de abrir RViz."
  sleep 5

  if [[ -f "$RVIZ_CONFIG" ]]; then
    log "Abriendo RViz con configuración: $RVIZ_CONFIG"
    (
      set +u
      source /opt/ros/humble/setup.bash
      source "$HOME/agv_ws/install/setup.bash"
      set -u
      rviz2 -d "$RVIZ_CONFIG"
    ) >>"$RUN_DIR/rviz.log" 2>&1 &
    RVIZ_PID=$!
  else
    log "WARN: No existe RVIZ_CONFIG: $RVIZ_CONFIG. RViz no se abrirá."
  fi
}

runtime_watchdog() {
  local interval=5
  log "Watchdog iniciado (intervalo=${interval}s)"
  while true; do
    sleep "$interval"
    if ! ~/agv_super_launcher/agv_health_monitor.sh >/dev/null 2>&1; then
      log "ERROR: Watchdog detectó fallo crítico (/scan o /tf). Deteniendo sistema."
      
if [[ -n "${ROS_PID:-}" ]]; then
  kill "$ROS_PID" 2>/dev/null
fi

      break
    fi
  done
}

main() {
  trap cleanup INT TERM EXIT

  log "Mega Super Launcher Host v2 iniciado."
  log "Logs: $RUN_DIR"

  check_env
  start_container
  start_yolo
  wait_for_yolo_json
  start_ros
  start_rviz

  log "ROS superlauncher lanzado. PID ROS=$ROS_PID | PID YOLO=$YOLO_PID | PID RVIZ=$RVIZ_PID"
  log "Esperando estabilización de ROS (10s)..."
  sleep 10
  log "Ejecutando health monitor inicial..."

  HEALTH_OUTPUT="$(~/agv_super_launcher/agv_health_monitor.sh)"
  echo "$HEALTH_OUTPUT" | tee -a "$LAUNCHER_LOG"

  log "Health check inicial no bloqueante (modo arranque)."

  log "Sistema vivo. Usa Ctrl+C para cerrar."


  wait "$ROS_PID"
}

main
