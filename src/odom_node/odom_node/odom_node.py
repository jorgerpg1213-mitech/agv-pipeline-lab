import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Quaternion, TransformStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class OdomNode(Node):
    def __init__(self):
        super().__init__('odom_node')

        # Topics / frames
        self.declare_parameter('wheel_vel_topic', '/wheel_vel')
        self.declare_parameter('imu_topic', '/imu')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        # Behavior
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('use_imu_yaw_rate', False)
        self.declare_parameter('update_rate_hz', 20.0)
        self.declare_parameter('wheel_vel_timeout_sec', 0.25)
        self.declare_parameter('imu_timeout_sec', 0.25)

        # Plausibility limits
        self.declare_parameter('max_linear_speed', 5.0)
        self.declare_parameter('max_angular_speed', 20.0)

        # Covariances
        self.declare_parameter(
            'pose_covariance_diagonal',
            [0.02, 0.02, 99999.0, 99999.0, 99999.0, 0.05]
        )
        self.declare_parameter(
            'twist_covariance_diagonal',
            [0.02, 0.02, 99999.0, 99999.0, 99999.0, 0.05]
        )

        self.wheel_vel_topic = str(self.get_parameter('wheel_vel_topic').value)
        self.imu_topic = str(self.get_parameter('imu_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.odom_frame = str(self.get_parameter('odom_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)

        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        self.use_imu_yaw_rate = bool(self.get_parameter('use_imu_yaw_rate').value)
        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.wheel_vel_timeout_sec = float(self.get_parameter('wheel_vel_timeout_sec').value)
        self.imu_timeout_sec = float(self.get_parameter('imu_timeout_sec').value)
        self.max_linear_speed = float(self.get_parameter('max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)

        pose_diag = list(self.get_parameter('pose_covariance_diagonal').value)
        twist_diag = list(self.get_parameter('twist_covariance_diagonal').value)
        self.pose_covariance = self._diag6_to_cov36(pose_diag)
        self.twist_covariance = self._diag6_to_cov36(twist_diag)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.wheel_sub = self.create_subscription(
            Twist,
            self.wheel_vel_topic,
            self.wheel_vel_callback,
            qos_profile_sensor_data,
        )
        self.imu_sub = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            qos_profile_sensor_data,
        )

        # Internal state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.v_meas = 0.0
        self.w_wheel_meas = 0.0
        self.w_imu_meas = 0.0
        self.w_used = 0.0

        self.last_update_time = self.get_clock().now()
        self.last_wheel_rx_time = None
        self.last_imu_rx_time = None

        self.warned_wheel_timeout = False
        self.warned_imu_timeout = False

        period = 1.0 / self.update_rate_hz if self.update_rate_hz > 0.0 else 0.05
        self.timer = self.create_timer(period, self.update)

        self.get_logger().info(
            f'odom_node started | wheel_vel={self.wheel_vel_topic} | '
            f'imu={self.imu_topic} | odom={self.odom_topic} | '
            f'use_imu_yaw_rate={self.use_imu_yaw_rate} | publish_tf={self.publish_tf}'
        )

    def _diag6_to_cov36(self, diag):
        if len(diag) != 6:
            raise ValueError('Covariance diagonal must have 6 elements.')
        cov = [0.0] * 36
        cov[0] = float(diag[0])
        cov[7] = float(diag[1])
        cov[14] = float(diag[2])
        cov[21] = float(diag[3])
        cov[28] = float(diag[4])
        cov[35] = float(diag[5])
        return cov

    def _clamp(self, value, limit):
        return max(-limit, min(limit, value))

    def _normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def _is_fresh(self, stamp, timeout_sec):
        if stamp is None:
            return False
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        return age <= timeout_sec

    def wheel_vel_callback(self, msg):
        v = float(msg.linear.x)
        w = float(msg.angular.z)

        self.v_meas = self._clamp(v, self.max_linear_speed)
        self.w_wheel_meas = self._clamp(w, self.max_angular_speed)
        self.last_wheel_rx_time = self.get_clock().now()

        if self.warned_wheel_timeout:
            self.get_logger().info('wheel_vel stream recovered')
            self.warned_wheel_timeout = False

    def imu_callback(self, msg):
        wz = float(msg.angular_velocity.z)
        self.w_imu_meas = self._clamp(wz, self.max_angular_speed)
        self.last_imu_rx_time = self.get_clock().now()

        if self.warned_imu_timeout:
            self.get_logger().info('imu stream recovered')
            self.warned_imu_timeout = False

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        if dt <= 0.0 or dt > 1.0:
            return

        wheel_fresh = self._is_fresh(self.last_wheel_rx_time, self.wheel_vel_timeout_sec)
        imu_fresh = self._is_fresh(self.last_imu_rx_time, self.imu_timeout_sec)

        if wheel_fresh:
            v = self.v_meas
            w_wheel = self.w_wheel_meas
        else:
            v = 0.0
            w_wheel = 0.0
            if not self.warned_wheel_timeout:
                self.get_logger().warn('wheel_vel timeout: forcing v=0 and w_wheel=0')
                self.warned_wheel_timeout = True

        if self.use_imu_yaw_rate:
            if imu_fresh:
                w = self.w_imu_meas
            else:
                w = w_wheel
                if not self.warned_imu_timeout:
                    self.get_logger().warn('imu timeout: falling back to wheel angular rate')
                    self.warned_imu_timeout = True
        else:
            w = w_wheel

        self.w_used = w

        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = self._normalize_angle(self.theta + w * dt)

        q = self._yaw_to_quaternion(self.theta)

        if self.publish_tf:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = now.to_msg()
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = self.x
            tf_msg.transform.translation.y = self.y
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = q
            self.tf_broadcaster.sendTransform(tf_msg)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        odom.pose.covariance = self.pose_covariance

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        odom.twist.covariance = self.twist_covariance

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
