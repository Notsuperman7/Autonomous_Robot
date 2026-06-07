import threading
import time

from flask import Flask, app, render_template, request, jsonify, Response

import os
from ament_index_python.packages import get_package_share_directory
from camera_interfaces.msg import DetectionArray

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge
import cv2


class RobotGuiNode(Node):
    def __init__(self):
        super().__init__("robot_gui_node")

        self.command_pub = self.create_publisher(String, "/gui_command", 10)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_lane_frame = None  # For lane detection visualization

        self.camera_sub = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.camera_callback,
            10
        )
        self.lane_debug_sub = self.create_subscription(  # For receiving lane detection debug images
            Image,
            "/lane_detection/debug_image",
            self.lane_debug_callback,
            10
        )


        self.latest_detections = []

        self.detection_sub = self.create_subscription(
            DetectionArray,
            "/detections",
            self.detections_callback,
            10
        )

        template_dir = os.path.join(
            get_package_share_directory("robot_gui"),
            "templates"
        )

        self.app = Flask(
             __name__,
            template_folder=template_dir
        )

        self.setup_routes()


    def detections_callback(self, msg):
        self.latest_detections = []

        for det in msg.detections:
            self.latest_detections.append({
                "class_name": det.class_name,
                "confidence": float(det.confidence),
                "x_min": int(det.x_min),
                "y_min": int(det.y_min),
                "x_max": int(det.x_max),
                "y_max": int(det.y_max),
            })

    def camera_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Camera frame conversion failed: {e}")
    
    def lane_debug_callback(self, msg):  # For receiving lane detection debug images
        try:
            self.latest_lane_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Lane debug frame conversion failed: {e}")

    
    def generate_frames(self):
        while True:
            if self.latest_lane_frame is not None:  # dont forget to change this back to "latest_frame" 
                #success, buffer = cv2.imencode(".jpg", self.latest_frame)
                success, buffer = cv2.imencode(".jpg", self.latest_lane_frame)  # Send lane detection debug image instead of raw camera feed

                if success:
                    frame = buffer.tobytes()

                    yield (
                        b"--frame\r\n"
                        b"Content-Type: image/jpeg\r\n\r\n" + frame + b"\r\n"
                    )

            time.sleep(0.03)

    def setup_routes(self):
        @self.app.route("/")
        def home():
            return render_template("index.html")

        @self.app.route("/command", methods=["POST"])
        def command():
            data = request.get_json()
            cmd = data.get("command", "")

            msg = String()
            msg.data = cmd
            self.command_pub.publish(msg)

            self.get_logger().info(f"Published command: {cmd}")

            return jsonify({
                "status": "ok",
                "command": cmd
            })
        
        @self.app.route("/detections")
        def get_detections():
            return jsonify(self.latest_detections)

        @self.app.route("/video_feed")  #-> FOR RAW CAMERA FEED
        def video_feed():
            return Response(
                self.generate_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )
        
        @self.app.route("/lane_video_feed") # FOR LANE DETECTION DEBUG FEED
        def lane_video_feed():
            return Response(
                self.generate_frames(),
                mimetype="multipart/x-mixed-replace; boundary=frame"
            )

    def run_flask(self):
        self.app.run(
            host="0.0.0.0",
            port=5000,
            debug=False,
            use_reloader=False,
            threaded=True
        )


def main(args=None):
    rclpy.init(args=args)

    node = RobotGuiNode()

    flask_thread = threading.Thread(target=node.run_flask)
    flask_thread.daemon = True
    flask_thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()