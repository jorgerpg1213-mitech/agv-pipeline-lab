import json
import time
import subprocess
from pathlib import Path

STATUS_FILE = Path('/mnt/agv_share/agv_status.json')
ESCALATION_FLAG = Path('/mnt/agv_share/escalation_required.flag')

CHECK_INTERVAL = 2.0
MAX_RECOVERY_ATTEMPTS = 3
MAX_FAULT_DURATION = 30.0

recovery_attempts = {}
fault_start_times = {}
RECOVERY_COOLDOWN = 15.0
recovery_in_progress_until = 0.0
last_recovery_time = 0.0


def load_status():
    try:
        with open(STATUS_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f'[RECOVERY] Error leyendo status: {e}')
        return None


def recover_esp32_bridge():
    print('[RECOVERY] Ejecutando recovery ESP32 bridge...')
    subprocess.run('pkill -f esp32_bridge_node', shell=True)
    time.sleep(10)
    subprocess.Popen(
        'source /opt/ros/humble/setup.bash && '
        'source /root/agv_ws/install/setup.bash && '
        'nohup ros2 run esp32_bridge_node esp32_bridge_node '
        '>/tmp/esp32_bridge_recovery.log 2>&1 &',
        shell=True,
        executable='/bin/bash'
    )
    print('[RECOVERY] Recovery ESP32 lanzado.')


def recover_lidar():
    print('[RECOVERY] Ejecutando recovery RPLidar...')
    subprocess.run('pkill -f rplidar_node', shell=True)
    time.sleep(20)
    subprocess.Popen(
        ['bash', '-lc',
         'source /opt/ros/humble/setup.bash && '
         'source /root/agv_ws/install/setup.bash && '
         'exec ros2 launch rplidar_ros rplidar_a1_launch.py'],
        start_new_session=True
    )
    time.sleep(5)
    subprocess.Popen(
        ['bash', '-lc',
         'source /opt/ros/humble/setup.bash && '
         'ros2 service call /start_motor std_srvs/srv/Empty'],
        start_new_session=True
    )
    print('[RECOVERY] Recovery RPLidar lanzado + start_motor enviado.')


def main():
    global last_recovery_time, recovery_in_progress_until

    print('[RECOVERY] AGV Recovery Manager iniciado.')

    if ESCALATION_FLAG.exists():
        ESCALATION_FLAG.unlink()
        print('[RECOVERY] ESCALATION_FLAG limpiado al inicio.')

    while True:
        status = load_status()

        if status is None:
            time.sleep(CHECK_INTERVAL)
            continue

        state = status.get('state', 'UNKNOWN')
        fault = status.get('fault', 'NONE')
        components = status.get('components', {})
        faults = [f.strip() for f in fault.split(',')]

        print(f'[RECOVERY] state={state} fault={fault}')

        now = time.time()

        if state == 'FAULT_PASSIVE':
            fault_start_times.setdefault(fault, now)
        else:
            fault_start_times.pop(fault, None)
            recovery_attempts[fault] = 0

        if state == 'FAULT_PASSIVE':
            fault_duration = now - fault_start_times.get(fault, now)
            if recovery_attempts.get(fault, 0) >= MAX_RECOVERY_ATTEMPTS and fault_duration >= MAX_FAULT_DURATION:
                ESCALATION_FLAG.touch(exist_ok=True)
                print(f'[RECOVERY] ESCALATION_REQUIRED fault={fault}')

        if now < recovery_in_progress_until:
            print(f'[RECOVERY] grace period activa hasta {recovery_in_progress_until}')
            time.sleep(CHECK_INTERVAL)
            continue

        if now - last_recovery_time < RECOVERY_COOLDOWN:
            time.sleep(CHECK_INTERVAL)
            continue

        if state == 'FAULT_PASSIVE' and 'esp32_imu' in faults:
            recovery_in_progress_until = now + 30.0
            recover_esp32_bridge()
            recovery_attempts[fault] = recovery_attempts.get(fault, 0) + 1
            print(f'[RECOVERY] attempts={recovery_attempts[fault]}')
            last_recovery_time = now

        if ESCALATION_FLAG.exists():
            print('[RECOVERY] Escalation activa, no se ejecuta recover_lidar()')
        else:
            if state == 'FAULT_PASSIVE' and 'lidar' in faults and components.get('lidar') == 'FAIL':
                recovery_in_progress_until = now + 45.0
                recover_lidar()
                recovery_attempts[fault] = recovery_attempts.get(fault, 0) + 1
                print(f'[RECOVERY] attempts={recovery_attempts[fault]}')
                last_recovery_time = now

        time.sleep(CHECK_INTERVAL)


if __name__ == '__main__':
    main()
