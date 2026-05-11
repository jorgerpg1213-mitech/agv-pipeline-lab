import rclpy
from rclpy.node import Node
import json
import os

from agv_msgs.msg import Detection
from agv_msgs.msg import DetectionArray


SNAPSHOT_PATH = "/mnt/agv_share/latest.json"


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        # ahora publica DetectionArray en lugar de String
        self.publisher_ = self.create_publisher(DetectionArray, '/detections', 10)

        # timer 20 Hz
        self.timer = self.create_timer(0.05, self.timer_callback)

        self.last_frame_id = None

    def timer_callback(self):

        if not os.path.exists(SNAPSHOT_PATH):
            return

        try:
            with open(SNAPSHOT_PATH, "r") as f:
                data = json.load(f)
        except Exception:
            # archivo puede estar siendo escrito
            return

        frame_id = data.get("frame_id", None)

        if frame_id == self.last_frame_id:
            return

        self.last_frame_id = frame_id

        msg = DetectionArray()

        msg.timestamp = float(data.get("timestamp", 0.0))
        msg.frame_id = int(frame_id if frame_id is not None else 0)

        detections = data.get("detections", [])

        for det in detections:

            d = Detection()

            d.track_id = int(det.get("track_id", 0))
            d.class_name = str(det.get("class", ""))
            d.confidence = float(det.get("conf", 0.0))

            center = det.get("center")

            if center is not None and len(center) == 2:
                d.center_x = float(center[0])
                d.center_y = float(center[1])
            else:
                d.center_x = 0.0
                d.center_y = 0.0

            d.speed = float(det.get("speed_px_s", 0.0))
            d.direction = str(det.get("direction", ""))

            msg.detections.append(d)

        self.publisher_.publish(msg)

        self.get_logger().info(f"Published frame {frame_id} with {len(msg.detections)} detections")


def main(args=None):

    rclpy.init(args=args)

    node = PerceptionNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
