import os
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .common import to_json, now_ts


class PerceptionNode(Node):
    def __init__(self) -> None:
        super().__init__("perception_node")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("save_dir", "/home/talay/records")

        self.camera_index = int(self.get_parameter("camera_index").value)
        self.save_dir = str(self.get_parameter("save_dir").value)

        os.makedirs(self.save_dir, exist_ok=True)

        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.video_path = os.path.join(self.save_dir, f"camera_processed_{stamp}.mp4")
        self.writer = cv2.VideoWriter(
            self.video_path,
            cv2.VideoWriter_fourcc(*"mp4v"),
            10.0,
            (640, 480),
        )

        self.hint_pub = self.create_publisher(String, "/perception/corridor_hint", 10)
        self.timer = self.create_timer(0.1, self.loop)

    def loop(self) -> None:
        ok, frame = self.cap.read()
        if not ok:
            return

        h, w = frame.shape[:2]
        image_center_x = w // 2

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blur, 50, 150)

        lines = cv2.HoughLinesP(
            edges,
            rho=1,
            theta=cv2.cv2.PI / 180.0,
            threshold=60,
            minLineLength=40,
            maxLineGap=20,
        )

        left_x = []
        right_x = []

        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                mid_x = (x1 + x2) / 2.0
                if mid_x < image_center_x:
                    left_x.append(mid_x)
                else:
                    right_x.append(mid_x)
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        heading_bias_deg = 0.0
        if left_x and right_x:
            corridor_center = (max(left_x) + min(right_x)) / 2.0
            pixel_error = corridor_center - image_center_x
            heading_bias_deg = -pixel_error * 0.03
            cv2.line(frame, (int(corridor_center), 0), (int(corridor_center), h), (255, 0, 0), 2)

        cv2.line(frame, (image_center_x, 0), (image_center_x, h), (0, 0, 255), 2)
        cv2.putText(
            frame,
            f"bias_deg={heading_bias_deg:.2f}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (255, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            datetime.fromtimestamp(now_ts()).strftime("%H:%M:%S.%f")[:-3],
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        self.writer.write(frame)
        self.hint_pub.publish(String(data=to_json({"heading_bias_deg": heading_bias_deg})))

    def destroy_node(self):
        try:
            self.cap.release()
            self.writer.release()
            cv2.destroyAllWindows()
        finally:
            super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()