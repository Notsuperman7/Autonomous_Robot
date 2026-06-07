#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16MultiArray


class MecanumControllerNode(Node):
    def __init__(self):
        super().__init__('mecanum_controller_node')

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.wheel_radius = 0.04
        self.wheel_base = 0.175
        self.track_width = 0.20

        self.cmd_gain = 40.0
        self.rpm_gain = 4.0
        self.max_cmd = 100  # Changed to int for Int32

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        self.last_cmd_time = self.get_clock().now()

        self.cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        #Apply the QoS profile ONLY to the wheel_cmds publisher
        self.wheel_cmds_pub = self.create_publisher(
            Int16MultiArray,
            '/wheel_cmds',
            qos
        )

        self.timer = self.create_timer(0.05, self.publish_wheel_cmds) 

        self.get_logger().info('Mecanum controller running at 50 Hz')

    def clamp(self, value):
        return max(min(int(value), self.max_cmd), -self.max_cmd)

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        self.last_cmd_time = self.get_clock().now()
        
    def rad_s_to_rpm(self, w):
        return w * 60.0 / (2.0 * math.pi)

    def publish_wheel_cmds(self):
        now = self.get_clock().now()
        dt = (now - self.last_cmd_time).nanoseconds / 1e9

        if dt > 5.0:
            vx = 0.0
            vy = 0.0
            wz = 0.0
        else:
            vx = self.vx
            vy = self.vy
            wz = self.wz

        k = self.wheel_base + self.track_width
        r = self.wheel_radius

        w_fl = (vx - vy - k * wz) / r
        w_fr = (vx + vy + k * wz) / r
        w_rl = (vx + vy - k * wz) / r
        w_rr = (vx - vy + k * wz) / r

        cmd_fl = self.clamp(self.rad_s_to_rpm(w_fl) * self.rpm_gain)
        cmd_fr = self.clamp(self.rad_s_to_rpm(w_fr) * self.rpm_gain)
        cmd_rl = self.clamp(self.rad_s_to_rpm(w_rl) * self.rpm_gain)
        cmd_rr = self.clamp(self.rad_s_to_rpm(w_rr) * self.rpm_gain)

        out = Int16MultiArray()
        out.data = [
            int(cmd_fl),
            int(cmd_fr),
            int(cmd_rl),
            int(cmd_rr)
        ]

        self.wheel_cmds_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()