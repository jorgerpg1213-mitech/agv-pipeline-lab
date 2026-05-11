#!/usr/bin/env python3
import json
import os
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError

from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from tf2_msgs.msg import TFMessage
from agv_msgs.msg import DetectionArray


class AgvWatchdogNode(Node):
    def __init__(self):
        super().__init__('agv_watchdog_node')

        self.status_path = '/mnt/agv_share/agv_status.json'

        self.thresholds = {
            'lidar': 5.0,
            'esp32_imu': 5.0,
            'odom': 5.0,
            'tf': 5.0,
            'perception': 10.0,
        }

        self.fail_counts = {key: 0 for key in self.thresholds}
        self.last_seen = {key: None for key in self.thresholds}

        self.create_subscription(LaserScan, '/scan', lambda msg: self.mark_seen('lidar'), 10)
        self.create_subscription(Imu, '/imu', lambda msg: self.mark_seen('esp32_imu'), 10)
        self.create_subscription(Odometry, '/odom', lambda msg: self.mark_seen('odom'), 10)
        self.create_subscription(TFMessage, '/tf', lambda msg: self.mark_seen('tf'), 10)
        self.create_subscription(DetectionArray, '/detections', lambda msg: self.mark_seen('perception'), 10)

        self.timer = self.create_timer(1.0, self.evaluate)

        self.get_logger().info('AGV ROS2 watchdog pasivo iniciado. No reinicia, no mata, no apaga.')

    def mark_seen(self, component):
        self.last_seen[component] = time.monotonic()
        self.fail_counts[component] = 0

    def evaluate(self):
        now = time.monotonic()
        components = {}
        failed = []

        for component, threshold in self.thresholds.items():
            last = self.last_seen[component]

            if last is None:
                self.fail_counts[component] += 1
                components[component] = 'WAITING'
                failed.append(component)
                continue

            age = now - last
            if age > threshold:
                self.fail_counts[component] += 1
                components[component] = 'FAIL'
                failed.append(component)
            else:
                components[component] = 'OK'
                self.fail_counts[component] = 0

        max_fail = max(self.fail_counts.values())

        if max_fail >= 3:
            state = 'FAULT_PASSIVE'
            fault = ','.join(failed) if failed else 'NONE'
            can_restart = False
            level = 'error'
            message = 'Watchdog ROS2 pasivo confirmó fallo por contador. No ejecutó acciones destructivas.'
        elif max_fail >= 2:
            state = 'DEGRADED'
            fault = ','.join(failed) if failed else 'NONE'
            can_restart = False
            level = 'warn'
            message = 'Watchdog ROS2 pasivo detectó degradación temporal.'
        else:
            state = 'RUNNING' if not failed else 'STARTING'
            fault = 'NONE' if not failed else ','.join(failed)
            can_restart = True
            level = 'info'
            message = 'Watchdog ROS2 pasivo monitoreando.'

        payload = {
            'state': state,
            'fault': fault,
            'message': message,
            'can_restart': can_restart,
            'timestamp': datetime.now(timezone.utc).isoformat(),
            'components': components,
            'fail_counts': self.fail_counts,
        }

        self.write_status(payload)

        log_line = f"state={state} fault={fault} components={components}"
        if level == 'error':
            self.get_logger().error(log_line)
        elif level == 'warn':
            self.get_logger().warn(log_line)
        else:
            self.get_logger().info(log_line)

    def write_status(self, payload):
        tmp_path = self.status_path + '.tmp'
        try:
            with open(tmp_path, 'w') as f:
                json.dump(payload, f, indent=2)
            os.replace(tmp_path, self.status_path)
        except Exception as exc:
            self.get_logger().error(f'No se pudo escribir status JSON: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = AgvWatchdogNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, RCLError):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
