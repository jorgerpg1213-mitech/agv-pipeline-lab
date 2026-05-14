#!/usr/bin/env bash
set -u

CONTAINER="agv_ros2_lab"
ENABLE_YOLO=false
YOLO_SCRIPT="$HOME/yolo_gst_csi.py"
YOLO_VENV="$HOME/venvs/yolo_track"
HOST_SHARE="$HOME/pipeline_lab_x86_review/agv-pipeline-lab/agv_share"
JSON_FILE="$HOST_SHARE/latest.json"
LOG_ROOT="$HOME/agv_logs"
RUN_ID="$(date +%Y%m%d_%H%M%S)"
RUN_DIR="$LOG_ROOT/run_$RUN_ID"

YOLO_PID=""
ROS_PID=""
HOST_WATCHDOG_PID=""

RVIZ_CONFIG="$HOME/pipeline_lab_x86_review/agv-pipeline-lab/src/rplidar_ros/rviz/rplidar_ros.rviz"
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
  if [[ -n "${RVIZ_PID:-}" ]]; then
    docker exec "$CONTAINER" bash -lc "kill -TERM $RVIZ_PID 2>/dev/null; sleep 2; kill -KILL $RVIZ_PID 2>/dev/null" 2>/dev/null || true
    log "RViz detenido."
  fi

  log "Cerrando YOLO si está activo."
  if [[ -n "${YOLO_PID:-}" ]] && kill -0 "$YOLO_PID" 2>/dev/null; then
    kill -TERM "$YOLO_PID" 2>/dev/null || true
    sleep 2
    kill -KILL "$YOLO_PID" 2>/dev/null || true
    wait "$YOLO_PID" 2>/dev/null || true
  fi
  pkill -TERM -f "$YOLO_SCRIPT" 2/dev/null || true
  docker exec "$CONTAINER" bash -lc "pkill -f agv_recovery_manager" 2>/dev/null || true

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

if [[ "$ENABLE_YOLO" == true ]]; then
if [[ ! -f "$YOLO_SCRIPT" ]]; then
  log "ERROR: No existe YOLO_SCRIPT: $YOLO_SCRIPT"
  exit 1
fi

if [[ ! -d "$YOLO_VENV" ]]; then
  log "ERROR: No existe YOLO_VENV: $YOLO_VENV"
  exit 1
fi

fi
if [[ ! -d "$HOST_SHARE" ]]; then
  log "ERROR: No existe HOST_SHARE: $HOST_SHARE"
  exit 1
fi

log "Arrancando/verificando contenedor: $CONTAINER"
docker restart "$CONTAINER" >>"$LAUNCHER_LOG" 2>&1 || {

docker exec "$CONTAINER" bash -lc "pkill -f agv_recovery_manager 2>/dev/null; pkill -f rplidar_node 2>/dev/null; true"
  log "ERROR: No se pudo arrancar contenedor $CONTAINER"
  exit 1
}

if [[ "$ENABLE_YOLO" == true ]]; then
log "Limpiando latest.json viejo si existe."
rm -f "$JSON_FILE"

log "Arrancando YOLO en host."
(
  cd "$YOLO_VENV" || exit 1
  source bin/activate
  python "$YOLO_SCRIPT"
) >>"$YOLO_LOG" 2>&1 &
YOLO_PID=$!

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

log "latest.json fresco detectado. Arrancando ROS superlauncher dentro de Docker."
fi

docker exec "$CONTAINER" bash -lc '
rm -f /tmp/agv_ros_pgid
setsid bash -c '\''
echo $$ >/tmp/agv_ros_pgid
source /opt/ros/humble/setup.bash
source /root/agv_ws/install/setup.bash
exec ros2 launch agv_bringup system_nav2_slam.launch.py
'\'' &
CHILD_PID=$!
wait "$CHILD_PID"
' >>"$ROS_LOG" 2>&1 &
ROS_PID=$!

log "Esperando 5 segundos antes de abrir RViz."
sleep 10

log "Arrancando motor RPLidar..."
docker exec "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && ros2 service call /start_motor std_srvs/srv/Empty" >>"$LAUNCHER_LOG" 2>&1 || true
log "Motor RPLidar lanzado."


if [[ -f "$RVIZ_CONFIG" ]]; then
  log "Abriendo RViz desde contenedor con X11..."
  xhost +local:docker >>"$LAUNCHER_LOG" 2>&1 || true
  docker exec -d -e DISPLAY="$DISPLAY" "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && source /root/agv_ws/install/setup.bash && rviz2 -d /root/agv_ws/src/rplidar_ros/rviz/rplidar_ros.rviz"
  sleep 3
  RVIZ_PID=$(docker exec "$CONTAINER" pgrep -f rviz2 2>/dev/null || echo "")
  log "RViz lanzado desde contenedor. PID=$RVIZ_PID"
else
  log "WARN: No existe RVIZ_CONFIG: $RVIZ_CONFIG. RViz no se abrirá."
fi

log "ROS superlauncher lanzado. PID ROS=$ROS_PID | PID YOLO=$YOLO_PID | PID RVIZ=$RVIZ_PID"
log "Sistema vivo. Usa Ctrl+C para cerrar."

log "Monitoreando stack ROS..."
while docker exec "$CONTAINER" bash -lc "test -f /tmp/agv_ros_pgid" 2>/dev/null; do
  sleep 5
done
log "Stack ROS terminado."
