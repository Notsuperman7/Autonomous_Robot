

import os
import cv2

import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO

from camera_interfaces.msg import Detection, DetectionArray

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.bridge = CvBridge()
        self.conf_threshold = 0.75
        self.frame_count = 0
        self.process_every_n_frames = 3

        package_share=get_package_share_directory('camera_pkg')
        model_path = os.path.join(package_share, 'models', 'best_yolov8n_ncnn_model')
        self.model = YOLO(model_path)

        self.image_sub=self.create_subscription(
            Image,
            '/camera/image_raw',   
            self.image_callback,
            10
        )

        self.annotated_image_pub = self.create_publisher(
            Image,
            '/detection_image',
            10
        )
        

        self.detection_pub=self.create_publisher(
            DetectionArray,
            '/detections',
            10
        )
        self.get_logger().info('Detector node started.')
        self.get_logger().info(f'Using model: {model_path}')
        self.get_logger().info(f'Confidence threshold: {self.conf_threshold}')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as error:
            self.get_logger().error(f'Failed to convert image: {error}')
            return
        
        self.frame_count += 1
        if self.frame_count % self.process_every_n_frames != 0:
            return

        results = self.model(frame, verbose=False)

        detection_array = DetectionArray()
        detection_array.header = msg.header

        annotated_frame = frame.copy()

        for result in results:
            for box in result.boxes:
                confidence = float(box.conf[0].item())

                if confidence < self.conf_threshold:
                    continue

                class_id = int(box.cls[0].item())
                class_name = self.model.names[class_id]

                x_min, y_min, x_max, y_max = box.xyxy[0].tolist()

                x_min = int(x_min)
                y_min = int(y_min)
                x_max = int(x_max)
                y_max = int(y_max)

                detection = Detection()
                detection.class_name = str(class_name)
                detection.confidence = confidence
                detection.x_min = x_min
                detection.y_min = y_min
                detection.x_max = x_max
                detection.y_max = y_max

                detection_array.detections.append(detection)

                label = f'{class_name} {confidence:.2f}'

                cv2.rectangle(
                    annotated_frame,
                    (x_min, y_min),
                    (x_max, y_max),
                    (0, 255, 0),
                    2
                )

                cv2.putText(
                    annotated_frame,
                    label,
                    (x_min, max(y_min - 10, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        self.detection_pub.publish(detection_array)

        annotated_msg = self.bridge.cv2_to_imgmsg(
            annotated_frame,
            encoding='bgr8'
        )
        annotated_msg.header = msg.header
        self.annotated_image_pub.publish(annotated_msg)

        if detection_array.detections:
            summary = ', '.join(
                f'{det.class_name} ({det.confidence:.2f})'
                for det in detection_array.detections
            )
            self.get_logger().info(f'Detections: {summary}')


def main(args=None):
    rclpy.init(args=args)
    node= DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



