import subprocess
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__("camera_publisher_node")

        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.bridge = CvBridge()

        # Frame skipping settings
        self.frame_count = 0
        self.publish_frames = 3
        self.skip_frames = 3
        self.cycle_frames = self.publish_frames + self.skip_frames

        self.get_logger().info("Starting rpicam-vid camera stream...")
        self.get_logger().info("Frame skipping: publish 3 frames, skip 3 frames")

        self.process = subprocess.Popen(
            [
                "rpicam-vid",
                "-t", "0",
                "--width", "640",
                "--height", "480",
                "--codec", "mjpeg",
                "--nopreview",
                "-o", "-"
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0
        )

        self.buffer = bytearray()
        self.timer = self.create_timer(0.01, self.read_frame)

    def read_frame(self):
        chunk = self.process.stdout.read(4096)

        if not chunk:
            return

        self.buffer.extend(chunk)

        start = self.buffer.find(b"\xff\xd8")
        end = self.buffer.find(b"\xff\xd9")

        if start != -1 and end != -1 and end > start:
            jpg = self.buffer[start:end + 2]
            self.buffer = self.buffer[end + 2:]

            np_arr = np.frombuffer(jpg, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            if frame is not None:
                cycle_position = self.frame_count % self.cycle_frames

                if cycle_position < self.publish_frames:
                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "camera_frame"

                    self.publisher.publish(msg)

                self.frame_count += 1

    def destroy_node(self):
        if self.process:
            self.process.terminate()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = CameraPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()