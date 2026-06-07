#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Vector3
from cv_bridge import CvBridge
import os 
import cv2
import numpy as np


class LaneDetectionNode(Node):
    def __init__(self):
        super().__init__('lane_detection_node')

        self.bridge = CvBridge()
        self.save_counter = 0
        self.save_folder = "/home/pi5/autonomouse_ws/src/camera_pkg/live_debug"
        os.makedirs(self.save_folder, exist_ok=True)

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.lane_pub = self.create_publisher(
            Vector3,
            '/lane_detection/lane_info',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/lane_detection/debug_image',
            10
        )

        self.crop_ratio = 0.60

        # --- RED TAPE HSV RANGES (hue wraps around in OpenCV) ---
        self.lower_red1 = np.array([0, 120, 100])
        self.upper_red1 = np.array([10, 255, 255])

        self.lower_red2 = np.array([170, 120, 100])
        self.upper_red2 = np.array([180, 255, 255])

        # Lower floor for thin distant lanes
        self.min_pixels = 150

        self.get_logger().info('Lane Detection Node Started')

    def estimate_lane_width_px(self, y_in_roi, roi_height):
        """Expected lane-center-to-lane-center width in pixels at a given Y."""
        width_at_bottom = 280
        width_at_top = 120
        ratio = y_in_roi / roi_height
        return int(width_at_bottom * (1.0 - ratio) + width_at_top * ratio)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width, _ = frame.shape

        roi_start_y = int(height * (1.0 - self.crop_ratio))
        roi_start_y = roi_start_y - 20
        roi = frame[roi_start_y:height, :].copy()

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        # --- RED TAPE MASK: combine both hue ranges ---
        mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
        mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # --- MORPHOLOGY: connect first, clean last ---
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)

        lane_msg = Vector3()
        lane_detected = 0
        normalized_error = 0.0
        confidence = 0.0
        lane_center_x = int(width / 2)
        lane_center_y = int(roi.shape[0] / 2)

        lane_line_centers = []
        lane_line_points_y = []

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        valid_contours = [c for c in contours if cv2.contourArea(c) > self.min_pixels]

        if not valid_contours:
            # Nothing found — publish zeros and return
            lane_msg.x = 0.0
            lane_msg.y = 0.0
            lane_msg.z = 0.0
            self.lane_pub.publish(lane_msg)
            debug_msg = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
            return

        # --- SPLIT INTO LEFT / RIGHT GROUPS by centroid X ---
        roi_center_x = width // 2
        left_contours = []
        right_contours = []

        for c in valid_contours:
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            if cx < roi_center_x:
                left_contours.append(c)
            else:
                right_contours.append(c)

        # --- PICK THE SINGLE LARGEST CONTOUR ON EACH SIDE ---
        left_lane = max(left_contours, key=cv2.contourArea) if left_contours else None
        right_lane = max(right_contours, key=cv2.contourArea) if right_contours else None
        selected_contours = [c for c in [left_lane, right_lane] if c is not None]

        # --- SAMPLE EACH LANE AT ITS BOTTOM (max_y - 20) ---
        for contour in selected_contours:
            cv2.drawContours(roi, [contour], -1, (0, 255, 0), 3)

            contour_mask = np.zeros_like(mask)
            cv2.drawContours(contour_mask, [contour], -1, 255, thickness=cv2.FILLED)

            points = contour.reshape(-1, 2)
            max_y = int(np.max(points[:, 1]))

            # Try to sample 20 px up from the bottom of this lane line.
            # If that row is somehow empty, scan upward until we hit pixels.
            found = False
            for offset in [20, 10, 0, 30, 50]:
                sample_y = max_y - offset
                if sample_y < 0 or sample_y >= contour_mask.shape[0]:
                    continue
                row = contour_mask[sample_y, :]
                xs = np.where(row > 0)[0]
                if len(xs) > 0:
                    found = True
                    break

            if not found:
                continue

            left_x = int(xs[0])
            right_x = int(xs[-1])
            line_center_x = int((left_x + right_x) / 2)

            lane_line_centers.append(line_center_x)
            lane_line_points_y.append(sample_y)

            cv2.circle(roi, (left_x, sample_y), 6, (255, 0, 255), -1)
            cv2.circle(roi, (right_x, sample_y), 6, (255, 0, 255), -1)
            cv2.circle(roi, (line_center_x, sample_y), 10, (0, 0, 255), -1)

        image_center_x = width / 2.0

        # --- CENTER & ERROR CALCULATION ---
        if len(lane_line_centers) == 2:
            lane_center_x = int((lane_line_centers[0] + lane_line_centers[1]) / 2)
            lane_center_y = int((lane_line_points_y[0] + lane_line_points_y[1]) / 2)
            confidence = 1.0
            lane_detected = 1

        elif len(lane_line_centers) == 1:
            y = lane_line_points_y[0]
            half_width = self.estimate_lane_width_px(y, roi.shape[0]) // 2

            if lane_line_centers[0] < image_center_x:
                # Left lane found → road center is to the right
                lane_center_x = lane_line_centers[0] + half_width  #  error will be right
            else:
                # Right lane found → road center is to the left
                lane_center_x = lane_line_centers[0] - half_width #  error will be left

            lane_center_y = y
            confidence = 0.5
            lane_detected = 1

        normalized_error = (lane_center_x - image_center_x) / image_center_x

        if lane_detected == 1:
            cv2.circle(roi, (lane_center_x, lane_center_y), 14, (255, 0, 255), -1)

        cv2.line(roi, (width // 2, 0), (width // 2, roi.shape[0]), (255, 0, 0), 2)

        lane_msg.x = float(normalized_error)
        lane_msg.y = float(confidence)
        lane_msg.z = float(lane_detected)

        # Save debug frames
        if self.save_counter < 50:
            cv2.imwrite(f"{self.save_folder}/frame_{self.save_counter}.jpg", frame)
            cv2.imwrite(f"{self.save_folder}/roi_{self.save_counter}.jpg", roi)
            cv2.imwrite(f"{self.save_folder}/mask_{self.save_counter}.jpg", mask)
            self.get_logger().info(f"Saved debug frame {self.save_counter}")
            self.save_counter += 1

        self.lane_pub.publish(lane_msg)

        debug_msg = self.bridge.cv2_to_imgmsg(roi, encoding='bgr8')
        debug_msg.header = msg.header
        self.debug_image_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaneDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()