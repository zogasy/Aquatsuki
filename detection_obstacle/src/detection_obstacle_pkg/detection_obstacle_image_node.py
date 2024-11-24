#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from sensor_msgs.msg import Imu
from ros_gz_interfaces.msg import ParamVec
from rcl_interfaces.msg import Parameter

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
        #self.subscriber_optical = self.create_subscription(Imu, '/aquabot/sensors/imu/imu/data', self.speed_callback, 10)
        self.subscription = self.create_subscription(ParamVec,'/aquabot/sensors/acoustics/receiver/range_bearing',self.listener_callback,10)


        self.obstacle_detector = Obstacle()
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def image_callback(self, msg):
        #self.get_logger().info('Received image !')
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        masque, contours = self.obstacle_detector.detect(current_frame)

        # Dessiner les contours des obstacles sur l'image
        img_contour = self.obstacle_detector.draw_obstacles(current_frame, contours)

        # Mesurer la largeur des obstacles et dessiner les rectangles
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 100:  # Ignore les petits objets
                width, img_with_rect = self.obstacle_detector.measure_obstacle_width(contour, current_frame)
                #self.get_logger().info(f'Taille de l\'obstacle : {width:.2f} pixels')

        # Affichage des images
        cv2.imshow("Image avec rectangles", img_with_rect)
        #cv2.imshow("Masque binaire du filtre", masque)
        #cv2.imshow("Image avec contours", img_contour)
        cv2.waitKey(1)

    def listener_callback(self, msg):
        param = msg.params 
        for p in param:
            if p.name == 'range':
                distance = p.value.double_value
                self.get_logger().info(f'Distance : {distance} m')
            elif p.name == 'bearing':
                angle = p.value.double_value
                self.get_logger().info(f"Obstacle : angle = {angle} m")
            
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
