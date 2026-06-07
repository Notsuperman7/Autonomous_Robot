#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


class PositionPIDNode(Node):
    def __init__(self):
        super().__init__('position_pid_node')

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_yaw = 0.0
        self.has_target = False
        
        # ADDED: State tracker to prevent bouncing between driving and turning
        self.phase = 'DRIVE' 

        self.kp_dist = 0.8
        self.kp_steer = 2.5  
        self.kp_final_yaw = 0.8

        self.max_v = 0.12
        self.max_w = 0.3     

        self.pos_tolerance = 0.12
        self.yaw_tolerance = math.radians(8.0)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(Bool, '/goal_reached', 10)

        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.target_sub = self.create_subscription(Pose2D, '/target_pose', self.target_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Position PID Node Started')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, max_value):
        return max(min(value, max_value), -max_value)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.yaw = math.atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)

    def target_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.target_yaw = msg.theta
        self.has_target = True
        
        # Reset phase to DRIVE for the new target
        self.phase = 'DRIVE'
        self.status_pub.publish(Bool(data=False))

        self.get_logger().info(
            f'New target: x={self.target_x:.2f}, '
            f'y={self.target_y:.2f}, yaw={math.degrees(self.target_yaw):.1f} deg'
        )

    def stop_robot(self):
        self.cmd_pub.publish(Twist())

    def control_loop(self):
        if not self.has_target:
            return

        dx_world = self.target_x - self.x
        dy_world = self.target_y - self.y

        distance = math.sqrt(dx_world * dx_world + dy_world * dy_world)
        final_yaw_error = self.normalize_angle(self.target_yaw - self.yaw)

        # --- STATE TRANSITION LOGIC ---
        # If we are driving and get close enough to the target, lock into TURN mode.
        if self.phase == 'DRIVE' and distance <= self.pos_tolerance:
            self.phase = 'TURN'
            self.get_logger().info('Arrived at XY, locking into final rotation...')

        cmd = Twist()

        # PHASE 1: DRIVING
        if self.phase == 'DRIVE':
            cos_yaw = math.cos(self.yaw)
            sin_yaw = math.sin(self.yaw)

            ex_body = cos_yaw * dx_world + sin_yaw * dy_world
            ey_body = -sin_yaw * dx_world + cos_yaw * dy_world
            
            alignment_factor = max(0.0, 1.0 - abs(ey_body / distance))
            vx_raw = (self.kp_dist * ex_body) * alignment_factor
            
            vx = self.clamp(vx_raw, self.max_v)
            wz = self.clamp(self.kp_steer * ey_body, self.max_w)

            cmd.linear.x = vx
            cmd.linear.y = 0.0  
            cmd.angular.z = wz

        # PHASE 2: TURNING (Latched)
        elif self.phase == 'TURN':
            if abs(final_yaw_error) > self.yaw_tolerance:
                # Force X and Y to strictly 0 so we ONLY rotate, even if we drift
                cmd.linear.x = 0.0 
                cmd.linear.y = 0.0
                cmd.angular.z = self.clamp(self.kp_final_yaw * final_yaw_error, self.max_w)
            else:
                self.stop_robot()
                self.has_target = False
                self.get_logger().info('Target reached completely')
                self.status_pub.publish(Bool(data=True)) 
                return

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PositionPIDNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()