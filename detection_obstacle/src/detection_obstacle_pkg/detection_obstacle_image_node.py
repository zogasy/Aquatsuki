#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
import numpy as np
from .obstacle import Obstacle

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge 

class DetectionObstacle(Node):

    def __init__(self):
        super().__init__('Obstacle_detection')
        self.get_logger().info('Hello world from detection obstacle !')

        # Create a subscriber on the topic "random_image"
        self.subscriber_optical = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/optical/image_raw', self.image_callback, 10)

        self.obstacle_detector = Obstacle()
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Received image !')
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        masque, contour = self.obstacle_detector.detect(current_frame)

        # Display image
        cv2.imshow("Image ",current_frame)
        self.obstacle_detector.draw_obstacles(current_frame,contour)
        cv2.imshow("Masque niveau de gris",masque)
        #cv2.imshow("Image filtre",contour)
        cv2.waitKey(1)


   
def main(args=None):
    rclpy.init(args=args)

    detection_obstacle = DetectionObstacle()

    rclpy.spin(detection_obstacle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    detection_obstacle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
