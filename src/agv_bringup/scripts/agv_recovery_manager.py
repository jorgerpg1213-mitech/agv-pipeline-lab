
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

    subprocess.run(
        'pkill -f esp32_bridge_node',
        shell=True
    )

    time.sleep(2)

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

    subprocess.run(
        'pkill -f rplidar_node',
        shell=True
    )

    time.sleep(2)

    subprocess.Popen(
        'source /opt/ros/humble/setup.bash && '
        'source /root/agv_ws/install/setup.bash && '
        'nohup /root/agv_ws/install/rplidar_ros/lib/rplidar_ros/rplidar_node '
        '--ros-args -r __node:=rplidar_node '
        '--params-file /tmp/launch_params_r_u41w1i '
        '>/tmp/rplidar_recovery.log 2>&1 &',
        shell=True,
        executable='/bin/bash'
    )


    print('[RECOVERY] Recovery RPLidar lanzado.')



def main():
    global last_recovery_time

    print('[RECOVERY] AGV Recovery Manager iniciado.')

    while True:
        status = load_status()

        if status is None:
            time.sleep(CHECK_INTERVAL)
            continue

        state = status.get('state', 'UNKNOWN')
        fault = status.get('fault', 'NONE')


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

                print(f"[RECOVERY] ESCALATION_REQUIRED fault={fault}")


        if now - last_recovery_time < RECOVERY_COOLDOWN:
            time.sleep(CHECK_INTERVAL)
            continue

        if state == 'FAULT_PASSIVE' and fault == 'esp32_imu':
            recover_esp32_bridge()
            recovery_attempts[fault] = recovery_attempts.get(fault, 0) + 1

            print(f"[RECOVERY] attempts={recovery_attempts[fault]}")

            last_recovery_time = now

        if state == 'FAULT_PASSIVE' and fault == 'lidar':
            recover_lidar()
            recovery_attempts[fault] = recovery_attempts.get(fault, 0) + 1

            print(f"[RECOVERY] attempts={recovery_attempts[fault]}")

            last_recovery_time = now

        time.sleep(CHECK_INTERVAL)


if __name__ == '__main__':
    main()
