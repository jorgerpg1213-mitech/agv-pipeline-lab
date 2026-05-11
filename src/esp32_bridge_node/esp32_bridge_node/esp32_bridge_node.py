import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


class ESP32Bridge(Node):
    """
    Nodo serial único para puente ROS <-> ESP32.

    Responsabilidades:
    - Suscribirse a /cmd_vel
    - Convertir cmd_vel a referencias de rueda
    - Enviar $CMD al ESP32
    - Solicitar feedback con $REQFBK*
    - Leer y parsear $FBK
    - Publicar /wheel_vel como twist del chasis en SI:
        linear.x  [m/s]
        angular.z [rad/s]
    - Publicar /imu

    Protocolo esperado en ESP32:
    - RX CMD:    $CMD,seq,t_ms,enable,wl_ref,wr_ref*
    - RX REQFBK: $REQFBK*
    - TX FBK:    $FBK,seq,t_ms,left_ticks,right_ticks,wl_meas,wr_meas,ax,ay,az,gx,gy,gz,status*

    Convenciones:
    - wl_meas y wr_meas llegan desde el ESP32 en rad/s de cada rueda
    - ax, ay, az en m/s^2
    - gx, gy, gz en rad/s
    """

    def __init__(self):
        super().__init__('esp32_bridge_node')

        # -------------------------
        # Topics
        # -------------------------
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_vel_topic', '/wheel_vel')
        self.declare_parameter('imu_topic', '/imu')

        # -------------------------
        # Parámetros serial
        # -------------------------
        self.declare_parameter('serial_port', '/dev/esp32_agv')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_timeout_s', 0.05)
        self.declare_parameter('serial_write_timeout_s', 0.10)
        self.declare_parameter('serial_retry_interval_s', 1.0)
        self.declare_parameter('clear_buffers_on_open', True)

        # -------------------------
        # Parámetros de ciclo
        # -------------------------
        self.declare_parameter('cycle_rate_hz', 20.0)
        self.declare_parameter('fbk_read_attempts_per_cycle', 3)

        # -------------------------
        # Parámetros cinemáticos
        # -------------------------
        self.declare_parameter('wheel_radius', 0.05)   # m
        self.declare_parameter('wheel_base', 0.30)     # m

        # -------------------------
        # Timeout de comando
        # -------------------------
        self.declare_parameter('cmd_timeout_ms', 200)

        # -------------------------
        # Parámetros de publicación IMU
        # -------------------------
        self.declare_parameter('imu_frame_id', 'imu_link')
        self.declare_parameter('imu_orientation_available', False)
        self.declare_parameter('imu_angular_velocity_covariance_diagonal', [0.02, 0.02, 0.02])
        self.declare_parameter('imu_linear_acceleration_covariance_diagonal', [0.10, 0.10, 0.10])

        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.wheel_vel_topic = str(self.get_parameter('wheel_vel_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)

        self.serial_port_name = str(self.get_parameter('serial_port').value)
        self.baudrate = int(self.get_parameter('baudrate').value)
        self.serial_timeout_s = float(self.get_parameter('serial_timeout_s').value)
        self.serial_write_timeout_s = float(self.get_parameter('serial_write_timeout_s').value)
        self.serial_retry_interval_s = float(self.get_parameter('serial_retry_interval_s').value)
        self.clear_buffers_on_open = bool(self.get_parameter('clear_buffers_on_open').value)

        self.cycle_rate_hz = float(self.get_parameter('cycle_rate_hz').value)
        self.fbk_read_attempts_per_cycle = int(self.get_parameter('fbk_read_attempts_per_cycle').value)

        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_base = float(self.get_parameter('wheel_base').value)
        self.cmd_timeout_ms = int(self.get_parameter('cmd_timeout_ms').value)

        self.imu_frame_id = str(self.get_parameter('imu_frame_id').value)
        self.imu_orientation_available = bool(self.get_parameter('imu_orientation_available').value)

        imu_ang_cov_diag = list(self.get_parameter('imu_angular_velocity_covariance_diagonal').value)
        imu_lin_cov_diag = list(self.get_parameter('imu_linear_acceleration_covariance_diagonal').value)

        self._validate_parameters()

        self.imu_angular_velocity_covariance = self._diag3_to_cov9(imu_ang_cov_diag)
        self.imu_linear_acceleration_covariance = self._diag3_to_cov9(imu_lin_cov_diag)

        # -------------------------
        # Publishers
        # -------------------------
        self.twist_pub = self.create_publisher(Twist, self.wheel_vel_topic, 10)
        self.imu_pub = self.create_publisher(Imu, self.imu_topic, 10)

        # -------------------------
        # Subscriber
        # -------------------------
        self.cmd_sub = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_callback,
            10
        )

        # -------------------------
        # Estado de comando
        # -------------------------
        self.last_cmd_time_ns = 0
        self.last_linear = 0.0
        self.last_angular = 0.0
        self.cmd_seq = 0

        # -------------------------
        # Estado serial
        # -------------------------
        self.serial_port = None
        self.last_serial_retry_ns = 0
        self.consecutive_serial_errors = 0
        self.last_good_fbk_time_ns = 0

        self.open_serial()

        # -------------------------
        # Timer único de ciclo serial
        # -------------------------
        timer_period = 1.0 / self.cycle_rate_hz
        self.timer = self.create_timer(timer_period, self.serial_cycle)

        self.get_logger().info(
            f'esp32_bridge_node listo | cmd_vel={self.cmd_vel_topic} | '
            f'wheel_vel={self.wheel_vel_topic} | imu={self.imu_topic}'
        )

    # =========================================================
    # Validación
    # =========================================================
    def _validate_parameters(self):
        if self.cycle_rate_hz <= 0.0:
            raise ValueError('cycle_rate_hz debe ser > 0')
        if self.fbk_read_attempts_per_cycle <= 0:
            raise ValueError('fbk_read_attempts_per_cycle debe ser > 0')
        if self.wheel_radius <= 0.0:
            raise ValueError('wheel_radius debe ser > 0')
        if self.wheel_base <= 0.0:
            raise ValueError('wheel_base debe ser > 0')
        if self.cmd_timeout_ms < 0:
            raise ValueError('cmd_timeout_ms no puede ser negativo')

    def _diag3_to_cov9(self, diag):
        if len(diag) != 3:
            raise ValueError('La diagonal de covarianza IMU debe tener 3 elementos')
        cov = [0.0] * 9
        cov[0] = float(diag[0])
        cov[4] = float(diag[1])
        cov[8] = float(diag[2])
        return cov

    # =========================================================
    # Serial
    # =========================================================
    def open_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baudrate,
                timeout=self.serial_timeout_s,
                write_timeout=self.serial_write_timeout_s,
                inter_byte_timeout=self.serial_timeout_s,
                rtscts=False,
                dsrdtr=False,
                xonxoff=False,
            )

            if self.clear_buffers_on_open:
                try:
                    self.serial_port.reset_input_buffer()
                except Exception:
                    pass
                try:
                    self.serial_port.reset_output_buffer()
                except Exception:
                    pass

            try:
                _ = self.serial_port.readline()
            except Exception:
                pass

            self.consecutive_serial_errors = 0
            self.get_logger().info(
                f'Serial abierta en {self.serial_port_name} @ {self.baudrate}'
            )

        except Exception as e:
            self.serial_port = None
            self.get_logger().error(f'No se pudo abrir serial: {e}')

    def close_serial(self):
        if self.serial_port is not None:
            try:
                if self.serial_port.is_open:
                    self.serial_port.close()
            except Exception:
                pass
        self.serial_port = None

    def handle_serial_fault(self, context: str, exc: Exception):
        self.consecutive_serial_errors += 1
        self.get_logger().error(f'{context}: {exc}')
        self.close_serial()

    def try_reopen_serial_if_needed(self, now_ns: int):
        if self.serial_port is not None and self.serial_port.is_open:
            return

        retry_interval_ns = int(self.serial_retry_interval_s * 1_000_000_000)
        if (now_ns - self.last_serial_retry_ns) < retry_interval_ns:
            return

        self.last_serial_retry_ns = now_ns
        self.open_serial()

    # =========================================================
    # ROS CMD
    # =========================================================
    def cmd_callback(self, msg: Twist):
        self.last_linear = float(msg.linear.x)
        self.last_angular = float(msg.angular.z)
        self.last_cmd_time_ns = self.get_clock().now().nanoseconds

    def compute_wheel_refs(self, linear: float, angular: float):
        # v_l = v - w*L/2
        # v_r = v + w*L/2
        v_l = linear - (angular * self.wheel_base / 2.0)
        v_r = linear + (angular * self.wheel_base / 2.0)

        # rad/s
        wl_ref = v_l / self.wheel_radius
        wr_ref = v_r / self.wheel_radius
        return wl_ref, wr_ref

    def command_is_valid(self, now_ns: int):
        if self.last_cmd_time_ns <= 0:
            return False

        age_ms = (now_ns - self.last_cmd_time_ns) / 1_000_000.0
        return age_ms <= self.cmd_timeout_ms

    def build_cmd_frame(self, now_ns: int):
        now_ms = int(now_ns / 1_000_000)

        if self.command_is_valid(now_ns):
            enable = 1
            wl_ref, wr_ref = self.compute_wheel_refs(self.last_linear, self.last_angular)
        else:
            enable = 0
            wl_ref = 0.0
            wr_ref = 0.0

        frame = f"$CMD,{self.cmd_seq},{now_ms},{enable},{wl_ref:.4f},{wr_ref:.4f}*\n"
        self.cmd_seq += 1
        return frame

    # =========================================================
    # Parsing
    # =========================================================
    def parse_fbk_line(self, line: str):
        if not line:
            raise ValueError('línea vacía')

        if not (line.startswith('$FBK,') and line.endswith('*')):
            raise ValueError(f'trama inválida: {line}')

        payload = line[5:-1]
        fields = payload.split(',')

        if len(fields) != 13:
            raise ValueError(
                f'FBK inválida: se esperaban 13 campos y llegaron {len(fields)}'
            )

        seq = int(fields[0])
        t_ms = int(fields[1])
        left_ticks = int(fields[2])
        right_ticks = int(fields[3])
        wl_meas = float(fields[4])
        wr_meas = float(fields[5])
        ax = float(fields[6])
        ay = float(fields[7])
        az = float(fields[8])
        gx = float(fields[9])
        gy = float(fields[10])
        gz = float(fields[11])
        status = int(fields[12])

        return {
            'seq': seq,
            't_ms': t_ms,
            'left_ticks': left_ticks,
            'right_ticks': right_ticks,
            'wl_meas': wl_meas,
            'wr_meas': wr_meas,
            'ax': ax,
            'ay': ay,
            'az': az,
            'gx': gx,
            'gy': gy,
            'gz': gz,
            'status': status,
        }

    # =========================================================
    # Publish
    # =========================================================
    def publish_feedback(self, fbk: dict):
        wl_meas = fbk['wl_meas']   # rad/s rueda izquierda
        wr_meas = fbk['wr_meas']   # rad/s rueda derecha

        # -----------------------------------------------------
        # Chassis twist en SI
        # v = R * (wl + wr) / 2
        # w = R * (wr - wl) / L
        # -----------------------------------------------------
        v_chassis = self.wheel_radius * (wl_meas + wr_meas) / 2.0
        w_chassis = self.wheel_radius * (wr_meas - wl_meas) / self.wheel_base

        twist = Twist()
        twist.linear.x = v_chassis
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = w_chassis
        self.twist_pub.publish(twist)

        imu = Imu()
        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = self.imu_frame_id

        # No tenemos orientación estimada real desde este bridge
        imu.orientation.x = 0.0
        imu.orientation.y = 0.0
        imu.orientation.z = 0.0
        imu.orientation.w = 1.0

        if self.imu_orientation_available:
            imu.orientation_covariance = [0.0] * 9
        else:
            imu.orientation_covariance = [-1.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0,
                                          0.0, 0.0, 0.0]

        imu.linear_acceleration.x = fbk['ax']
        imu.linear_acceleration.y = fbk['ay']
        imu.linear_acceleration.z = fbk['az']
        imu.linear_acceleration_covariance = self.imu_linear_acceleration_covariance

        imu.angular_velocity.x = fbk['gx']
        imu.angular_velocity.y = fbk['gy']
        imu.angular_velocity.z = fbk['gz']
        imu.angular_velocity_covariance = self.imu_angular_velocity_covariance

        self.imu_pub.publish(imu)

    # =========================================================
    # Ciclo serial único
    # =========================================================
    def serial_cycle(self):
        now_ns = self.get_clock().now().nanoseconds
        self.try_reopen_serial_if_needed(now_ns)

        if self.serial_port is None or not self.serial_port.is_open:
            return

        try:
            cmd_frame = self.build_cmd_frame(now_ns)
            self.serial_port.write(cmd_frame.encode('utf-8'))

            self.serial_port.write(b"$REQFBK*\n")

            fbk = None
            for _ in range(self.fbk_read_attempts_per_cycle):
                raw = self.serial_port.readline()
                if not raw:
                    continue

                line = raw.decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                try:
                    fbk = self.parse_fbk_line(line)
                    break
                except ValueError:
                    self.get_logger().warning(f'Trama ignorada: {line}')
                    continue

            if fbk is None:
                return

            self.publish_feedback(fbk)
            self.last_good_fbk_time_ns = now_ns
            self.consecutive_serial_errors = 0

        except serial.SerialException as e:
            self.handle_serial_fault('Error serial en ciclo', e)
        except OSError as e:
            self.handle_serial_fault('Error OS en ciclo serial', e)
        except Exception as e:
            self.handle_serial_fault('Error inesperado en ciclo serial', e)

    # =========================================================
    # Cierre limpio
    # =========================================================
    def destroy_node(self):
        self.close_serial()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ESP32Bridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
