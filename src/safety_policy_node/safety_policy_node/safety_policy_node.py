import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from agv_msgs.msg import DetectionArray


class SafetyPolicyNode(Node):
    def __init__(self):
        super().__init__('safety_policy_node')

        self.declare_parameter('detections_topic', '/detections')
        self.declare_parameter('cmd_in_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_out_topic', '/cmd_vel')

        self.declare_parameter('image_width_px', 1280.0)
        self.declare_parameter('risk_zone_min_ratio', 0.35)
        self.declare_parameter('risk_zone_max_ratio', 0.65)
        self.declare_parameter('frames_required', 3)
        self.declare_parameter('hysteresis_time_s', 1.5)
        self.declare_parameter('detection_timeout_s', 0.7)
        self.declare_parameter('caution_scale', 0.35)
        self.declare_parameter('min_approach_speed_px_s', 20.0)

        self.detections_topic = str(self.get_parameter('detections_topic').value)
        self.cmd_in_topic = str(self.get_parameter('cmd_in_topic').value)
        self.cmd_out_topic = str(self.get_parameter('cmd_out_topic').value)

        self.image_width_px = float(self.get_parameter('image_width_px').value)
        self.risk_zone_min_ratio = float(self.get_parameter('risk_zone_min_ratio').value)
        self.risk_zone_max_ratio = float(self.get_parameter('risk_zone_max_ratio').value)
        self.frames_required = int(self.get_parameter('frames_required').value)
        self.hysteresis_time_s = float(self.get_parameter('hysteresis_time_s').value)
        self.detection_timeout_s = float(self.get_parameter('detection_timeout_s').value)
        self.caution_scale = float(self.get_parameter('caution_scale').value)
        self.min_approach_speed_px_s = float(self.get_parameter('min_approach_speed_px_s').value)

        self.last_cmd_in = Twist()
        self.last_detection_time = 0.0
        self.last_risk_time = 0.0
        self.consecutive_risk_frames = 0
        self.state = 'CLEAR'

        self.create_subscription(DetectionArray, self.detections_topic, self.detections_callback, 10)
        self.create_subscription(Twist, self.cmd_in_topic, self.cmd_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_out_topic, 10)

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info(
            f'Safety filter ready | detections={self.detections_topic} | '
            f'in={self.cmd_in_topic} | out={self.cmd_out_topic} | image_width_px={self.image_width_px}'
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds / 1e9

    def detections_callback(self, msg):
        now = self.now_s()
        self.last_detection_time = now

        risk_now = False
        caution_now = False

        risk_min = self.image_width_px * self.risk_zone_min_ratio
        risk_max = self.image_width_px * self.risk_zone_max_ratio
        risk_center = self.image_width_px * 0.5

        for det in msg.detections:
            if det.class_name != 'person':
                continue

            cx = float(det.center_x)
            speed = abs(float(det.speed))
            direction = str(det.direction).lower().strip()

            inside_roi = risk_min <= cx <= risk_max

            moving_toward_roi = (
                speed >= self.min_approach_speed_px_s and
                ((cx < risk_center and direction == 'right') or
                 (cx > risk_center and direction == 'left'))
            )

            if inside_roi or moving_toward_roi:
                risk_now = True
                break

            if speed >= self.min_approach_speed_px_s:
                caution_now = True

        if risk_now:
            self.consecutive_risk_frames += 1
        else:
            self.consecutive_risk_frames = 0

        if self.consecutive_risk_frames >= self.frames_required:
            self.last_risk_time = now

        if (now - self.last_risk_time) <= self.hysteresis_time_s:
            self.set_state('STOP')
        elif caution_now:
            self.set_state('CAUTION')
        else:
            self.set_state('CLEAR')

    def cmd_callback(self, msg):
        self.last_cmd_in = msg
        self.publish_filtered_cmd()

    def timer_callback(self):
        now = self.now_s()

        if self.last_detection_time > 0.0:
            if (now - self.last_detection_time) > self.detection_timeout_s:
                self.set_state('STOP')

        self.publish_filtered_cmd()

    def set_state(self, new_state):
        if new_state != self.state:
            self.state = new_state
            self.get_logger().info(f'SAFETY STATE: {self.state}')

    def publish_filtered_cmd(self):
        out = Twist()

        if self.state == 'STOP':
            pass
        elif self.state == 'CAUTION':
            out.linear.x = self.last_cmd_in.linear.x * self.caution_scale
            out.angular.z = self.last_cmd_in.angular.z * self.caution_scale
        else:
            out = self.last_cmd_in

        self.cmd_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = SafetyPolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
