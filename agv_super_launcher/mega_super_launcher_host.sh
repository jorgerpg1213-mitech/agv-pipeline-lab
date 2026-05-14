#!/usr/bin/env bash
set -u

CONTAINER="agv_ros2_lab"
YOLO_SCRIPT="$HOME/yolo_gst_csi.py"
YOLO_VENV="$HOME/venvs/yolo_track"
HOST_SHARE="$HOME/agv_share"
JSON_FILE="$HOST_SHARE/latest.json"
LOG_ROOT="$HOME/agv_logs"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$LOG_ROOT/run_$RUN_ID"

YOLO_PID=""
ROS_PID=""
HOST_WATCHDOG_PID=""

RVIZ_CONFIG="$HOME/agv_share/agv_nav2_slam.rviz"
RVIZ_PID=""

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

  log "Cerrando RViz si está activo."
  if [[ -n "${RVIZ_PID:-}" ]] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -TERM "$RVIZ_PID" 2>/dev/null || true
    sleep 2
    kill -KILL "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  pkill -TERM -f "rviz2 -d $RVIZ_CONFIG" 2>/dev/null || true

  log "Cerrando YOLO si está activo."
  if [[ -n "${YOLO_PID:-}" ]] && kill -0 "$YOLO_PID" 2>/dev/null; then
    kill -TERM "$YOLO_PID" 2>/dev/null || true
    sleep 2
    kill -KILL "$YOLO_PID" 2>/dev/null || true
    wait "$YOLO_PID" 2>/dev/null || true
  fi
  pkill -TERM -f "$YOLO_SCRIPT" 2>/dev/null || true

  log "Cerrando ROS launch por PGID si está activo."
  docker exec "$CONTAINER" bash -lc 'if [[ -f /tmp/agv_ros_pgid ]]; then PGID="$(cat /tmp/agv_ros_pgid)"; if [[ "$PGID" =~ ^[0-9]+$ ]]; then kill -TERM -"$PGID" 2>/dev/null || true; sleep 3; kill -KILL -"$PGID" 2>/dev/null || true; fi; rm -f /tmp/agv_ros_pgid; fi' >>"$LAUNCHER_LOG" 2>&1 || true

  if [[ -n "${ROS_PID:-}" ]] && kill -0 "$ROS_PID" 2>/dev/null; then
    wait "$ROS_PID" 2>/dev/null || true
  fi

  log "Verificando procesos huérfanos."
  ps aux | egrep 'ros2|rviz2|yolo_gst_csi|mega_super_launcher|agv_watchdog' | grep -v egrep >>"$LAUNCHER_LOG" 2>&1 || true

  log "Launcher cerrado."
}

trap cleanup INT TERM EXIT

log "Mega Super Launcher Host v1 iniciado."
log "Logs: $RUN_DIR"

if ! docker ps >/dev/null 2>&1; then
  log "ERROR: Docker no responde sin sudo."
  exit 1
fi

#if [[ ! -f "$YOLO_SCRIPT" ]]; then
#  log "ERROR: No existe YOLO_SCRIPT: $YOLO_SCRIPT"
#  exit 1
#fi
#
#if [[ ! -d "$YOLO_VENV" ]]; then
#  log "ERROR: No existe YOLO_VENV: $YOLO_VENV"
#  exit 1
#fi

if [[ ! -d "$HOST_SHARE" ]]; then
  log "ERROR: No existe HOST_SHARE: $HOST_SHARE"
  exit 1
fi

log "Arrancando/verificando contenedor: $CONTAINER"
docker start "$CONTAINER" >>"$LAUNCHER_LOG" 2>&1 || {
  log "ERROR: No se pudo arrancar contenedor $CONTAINER"
  exit 1
}

log "Limpiando latest.json viejo si existe."
rm -f "$JSON_FILE"

#log "Arrancando YOLO en host."
#(
#  cd "$YOLO_VENV" || exit 1
#  source bin/activate
#  python "$YOLO_SCRIPT"
#) >>"$YOLO_LOG" 2>&1 &
#YOLO_PID=$!
#
#log "Esperando latest.json fresco..."
#READY=0
#for i in {1..30}; do
#  if [[ -f "$JSON_FILE" ]]; then
#    AGE=$(( $(date +%s) - $(stat -c %Y "$JSON_FILE") ))
#    if [[ "$AGE" -le 2 ]]; then
#      READY=1
#      break
#    fi
#  fi
#  sleep 1
#done
#
#if [[ "$READY" -ne 1 ]]; then
#  log "ERROR: YOLO no generó latest.json fresco dentro del timeout."
#  exit 1
#fi

log "latest.json fresco detectado. Arrancando ROS superlauncher dentro de Docker."

docker exec "$CONTAINER" bash -lc '
rm -f /tmp/agv_ros_pgid
setsid bash -c '\''
echo $$ >/tmp/agv_ros_pgid
source /opt/ros/humble/setup.bash
source /root/agv_ws/install/setup.bash
ros2 launch agv_bringup system_nav2_slam.launch.py
'\'' &
CHILD_PID=$!
wait "$CHILD_PID"
' >>"$ROS_LOG" 2>&1 &
ROS_PID=$!

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

log "ROS superlauncher lanzado. PID ROS=$ROS_PID | PID YOLO=$YOLO_PID | PID RVIZ=$RVIZ_PID"
log "Sistema vivo. Usa Ctrl+C para cerrar."

wait "$ROS_PID"
