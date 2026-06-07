import subprocess
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_camera_node')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 10)

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)

        cmd = [
            'rpicam-vid',
            '--nopreview',
            '-t', '0',
            '--width', str(self.width),
            '--height', str(self.height),
            '--framerate', str(self.fps),
            '--codec', 'mjpeg',
            '-o', '-'
        ]

        self.get_logger().info('Starting rpicam-vid...')
        self.proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            bufsize=0
        )

        self.buffer = bytearray()
        self.timer = self.create_timer(1.0 / self.fps, self.publish_frame)

        self.get_logger().info(
            f'CSI camera started: {self.width}x{self.height} @ {self.fps} FPS'
        )
        self.get_logger().info('Publishing on /camera/image_raw')

    def publish_frame(self):
        try:
            if self.proc.stdout is None:
                self.get_logger().error('Camera process stdout is None')
                return

            chunk = self.proc.stdout.read(4096)
            if not chunk:
                self.get_logger().warning('No data from camera process')
                return

            self.buffer.extend(chunk)

            start = self.buffer.find(b'\xff\xd8')
            end = self.buffer.find(b'\xff\xd9')

            if start != -1 and end != -1 and end > start:
                jpg = self.buffer[start:end + 2]
                del self.buffer[:end + 2]

                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    self.get_logger().warning('Failed to decode JPEG frame')
                    return

                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera_frame'
                self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().error(f'Failed to capture/publish frame: {e}')

    def destroy_node(self):
        try:
            if hasattr(self, 'proc') and self.proc is not None:
                self.proc.terminate()
                self.proc.wait(timeout=2)
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CSICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()