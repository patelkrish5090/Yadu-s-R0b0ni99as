#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import math


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.planned_path_subscription = self.create_subscription(Point, '/planned_path', self.planned_path_callback, 10)
        self.next_waypoint = self.create_publisher(Bool, '/next_waypoint', 10)
        self.target_point = None

        self.offset_x = 35.7567 + 0.01475
        self.offset_y = 22.6746 + 0.01475
        self.scale_factor = 13.002
        self.current_position = None
        self.timer = self.create_timer(0.1, self.timer_callback)

        # State variables
        self.turning = False

        # Publish the first /next_waypoint message when the node starts
        self.publish_initial_waypoint()

    def odom_callback(self, msg):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self.get_yaw_from_quaternion(msg.pose.pose.orientation)
        )

    def planned_path_callback(self, msg):
        self.set_target_point(msg.x, msg.y)

    def set_target_point(self, x, y):
        self.target_point = (x, y)

    def get_yaw_from_quaternion(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def timer_callback(self):
        if self.target_point is None:
            self.stop_robot()
            return

        target_x, target_y = self.target_point
        target_x, target_y = self.point_transform((target_x, target_y))

        if self.current_position is None:
            return

        current_x, current_y, current_theta = self.current_position

        if self.is_at_waypoint(current_x, current_y, target_x, target_y):
            self.stop_robot()
            self.update_waypoint()
        else:
            self.navigate_to_waypoint(current_x, current_y, current_theta, target_x, target_y)

    def is_at_waypoint(self, current_x, current_y, target_x, target_y):
        distance = math.sqrt((current_x - target_x) ** 2 + (current_y - target_y) ** 2)
        return distance < 0.2  # Threshold for being "at" a waypoint

    def navigate_to_waypoint(self, current_x, current_y, current_theta, target_x, target_y):
        msg = Twist()
        angle_to_target = math.atan2(target_y - current_y, target_x - current_x)
        angle_diff = angle_to_target - current_theta

        # Normalize angle_diff to be between -pi and pi
        angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

        if abs(angle_diff) > 0.1:  # Turning threshold
            self.turning = True
            msg.angular.z = 2.0 * angle_diff
            msg.linear.x = 0.0  # Stop linear motion while turning
        else:
            self.turning = False
            distance_to_target = math.sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
            msg.linear.x = min(0.5, distance_to_target)
            msg.angular.z = 0.0

        self.publisher_.publish(msg)

    def stop_robot(self):
        msg = Twist()
        self.publisher_.publish(msg)

    def point_transform(self, pnt):
        gazebo_x = (-1 * pnt[1] / self.scale_factor) + self.offset_x
        gazebo_y = (-1 * pnt[0] / self.scale_factor) + self.offset_y
        return gazebo_x, gazebo_y
    
    def update_waypoint(self):
        msg = Bool()
        msg.data = True
        self.next_waypoint.publish(msg)
        self.get_logger().info('Published next waypoint request')

    def publish_initial_waypoint(self):
        msg = Bool()
        msg.data = True
        self.next_waypoint.publish(msg)
        self.get_logger().info('Published initial waypoint request')

def main(args=None):
    rclpy.init(args=args)
    navigator = WaypointNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
