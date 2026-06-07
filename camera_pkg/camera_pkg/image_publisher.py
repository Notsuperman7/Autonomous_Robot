import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture('/home/pi5/ws2/test.mp4')

        if not self.cap.isOpened():
            self.get_logger().error('Failed to open video file')
        else:
            self.get_logger().info('Video file opened successfully')

        self.timer = self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        if not self.cap.isOpened():
            self.get_logger().error('Video is not opened')
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info('Reached end of video, restarting...')
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)
        self.get_logger().info('Publishing video frame')

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()