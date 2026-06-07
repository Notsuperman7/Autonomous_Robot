import cv2
import numpy as np
import glob
import os


def main(args=None):
    image_folder = "/home/pi5/ws2/src/camera_pkg/lane_pic"
    output_folder = "/home/pi5/ws2/src/camera_pkg/lane_results"

    os.makedirs(output_folder, exist_ok=True)

    crop_ratio = 0.80
    lower_white = np.array([0, 0, 210])
    upper_white = np.array([180, 60, 255])

    min_pixels = 500

    image_paths = glob.glob(os.path.join(image_folder, "*"))

    for path in image_paths:
        frame = cv2.imread(path)

        if frame is None:
            print(f"Could not read: {path}")
            continue

        height, width, _ = frame.shape

        roi_start_y = int(height * (1.0 - crop_ratio))
        roi = frame[roi_start_y:height, :].copy()

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_white, upper_white)

        kernel = np.ones((7, 7), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        lane_detected = 0
        normalized_error = 0.0
        confidence = 0.0
        lane_center_x = int(width / 2)
        lane_center_y = int(roi.shape[0] / 2)

        lane_line_centers = []
        lane_line_points_y = []

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE
        )

        valid_contours = [
            c for c in contours
            if cv2.contourArea(c) > min_pixels
        ]

        valid_contours = sorted(
            valid_contours,
            key=cv2.contourArea,
            reverse=True
        )

        selected_contours = valid_contours[:2]

        for contour in selected_contours:
            cv2.drawContours(
                roi,
                [contour],
                -1,
                (0, 255, 0),
                3
            )
            contour_mask = np.zeros_like(mask)

            cv2.drawContours(
                contour_mask,
                [contour],
                -1,
                255,
                thickness=cv2.FILLED
            )

            points = contour.reshape(-1, 2)
            max_y = np.max(points[:, 1])

            red_point_y = max_y - 20

            if red_point_y < 0:
                red_point_y = max_y

            row = contour_mask[red_point_y, :]

            xs = np.where(row > 0)[0]

            if len(xs) > 0:
                left_x = int(xs[0])
                right_x = int(xs[-1])

                line_center_x = int((left_x + right_x) / 2)

                lane_line_centers.append(line_center_x)
                lane_line_points_y.append(red_point_y)

                cv2.circle(roi, (left_x, red_point_y), 6, (255, 0, 255), -1)
                cv2.circle(roi, (right_x, red_point_y), 6, (255, 0, 255), -1)
                cv2.circle(roi, (line_center_x, red_point_y), 10, (0, 0, 255), -1)

        if len(lane_line_centers) == 2:
            lane_center_x = int((lane_line_centers[0] + lane_line_centers[1]) / 2)
            lane_center_y = int((lane_line_points_y[0] + lane_line_points_y[1]) / 2)
            confidence = 1.0
            lane_detected = 1

        elif len(lane_line_centers) == 1:
            lane_center_x = lane_line_centers[0]
            lane_center_y = lane_line_points_y[0]
            confidence = 0.7
            lane_detected = 1

        image_center_x = width / 2.0
        normalized_error = (lane_center_x - image_center_x) / image_center_x

        if lane_detected == 1:
            cv2.circle(
                roi,
                (lane_center_x, lane_center_y),
                14,
                (255, 0, 255),
                -1
            )

        cv2.line(
            roi,
            (width // 2, 0),
            (width // 2, roi.shape[0]),
            (255, 0, 0),
            2
        )

        name = os.path.splitext(os.path.basename(path))[0]

        cv2.imwrite(os.path.join(output_folder, name + "_original.jpg"), frame)
        cv2.imwrite(os.path.join(output_folder, name + "_roi_detection.jpg"), roi)
        cv2.imwrite(os.path.join(output_folder, name + "_mask.jpg"), mask)

        print("Image:", os.path.basename(path))
        print("lane_error:", normalized_error)
        print("confidence:", confidence)
        print("lane_detected:", lane_detected)
        print("detected_lines:", len(lane_line_centers))
        print("saved to:", output_folder)
        print("--------------------")


if __name__ == '__main__':
    main()