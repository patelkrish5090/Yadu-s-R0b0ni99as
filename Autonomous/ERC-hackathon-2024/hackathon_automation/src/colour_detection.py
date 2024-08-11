#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2 as cv
import numpy as np

class ColorDetection(Node):
    def __init__(self):
        super().__init__('color_detection')
        self.publisher = self.create_publisher(String, 'task_status', 10)
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.color_detected_pub = self.create_publisher(String, '/color_detected', 10)
        self.start_sub = self.create_subscription(Bool, '/start_color_detection', self.start_detection_callback, 10)
        self.stop_sub = self.create_subscription(Bool, '/stop_color_detection', self.stop_detection_callback, 10)
        self.cone_detected = self.create_publisher(Bool, '/cone_detected', 10)
        self.bridge = CvBridge()
        self.cone_count = 0
        self.total_cones = 5
        self.get_logger().info("Color detection node initialized")
        self.detecting = False

    def start_detection_callback(self, msg):
        if msg.data:
            self.detecting = True
            self.get_logger().info("Started color detection.")

    def stop_detection_callback(self, msg):
        if msg.data:
            self.detecting = False
            self.get_logger().info("Stopped color detection.")

    def image_callback(self, msg):
        if not self.detecting:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            hsv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2HSV)

            red_lower1 = np.array([0, 100, 100])
            red_upper1 = np.array([10, 255, 255])
            red_lower2 = np.array([160, 100, 100])
            red_upper2 = np.array([180, 255, 255])

            blue_lower = np.array([100, 100, 100])
            blue_upper = np.array([140, 255, 255])

            mask_red1 = cv.inRange(hsv_image, red_lower1, red_upper1)
            mask_red2 = cv.inRange(hsv_image, red_lower2, red_upper2)
            mask_red = mask_red1 | mask_red2

            mask_blue = cv.inRange(hsv_image, blue_lower, blue_upper)

            red_detected = np.any(mask_red)
            blue_detected = np.any(mask_blue)

            if red_detected:
                self.get_logger().info("Error Detected at Site!!!")
                self.color_detected_pub.publish(String(data="Red"))

            if blue_detected:
                self.get_logger().info("No Error Detected at Site")
                self.color_detected_pub.publish(String(data="Blue"))

            self.cone_detected.publish(Bool(data=True))

        except Exception as e:
            self.get_logger().error(f"Error in image processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    color_detection_node = ColorDetection()
    rclpy.spin(color_detection_node)
    color_detection_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
