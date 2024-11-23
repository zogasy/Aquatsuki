#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2
import numpy as np
from .obstacle import Obstacle

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge 

class MninimalImageSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_image_subscriber')
        self.get_logger().info('Hello world from minimal_image_subscriber !')

        # Create a subscriber on the topic "random_image"
        self.subscriber_optical = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/optical/image_raw', self.image_callback, 10)

        self.obstacle_detector = Obstacle()
        
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def image_callback(self, msg):
        self.get_logger().info('Received image !')
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #masque , resultat = self.filtrer_gris(current_frame)
        masque, contour = self.obstacle_detector.detect(current_frame)

        """dimensions = np.array([576, 720])
            ind=dimensions//2
            offset = 30
            ind[0] = ind[0] - offset
            middle = current_frame[ind[0],ind[1],:]
            self.get_logger().info(f"valeur hsv millieu : {middle}")
            current_frame[ind[0] ,:] = [0,0,0]
            radius = 10
            current_frame[ind[0] :ind[0] + radius ,ind[1] :ind[1] + radius ,:] = [0, 0 , 0]"""

        # Display image
        cv2.imshow("Image ",current_frame)
        self.obstacle_detector.draw_obstacles(current_frame,contour)
        cv2.imshow("Masque niveau de gris",masque)
        #cv2.imshow("Image filtre",contour)
        cv2.waitKey(1)


    def filtrer_gris(self,image):
        # Convertir l'image en espace de couleur HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Définir les seuils pour la couleur grise en HSV
        bas_gris = np.array([0, 0, 50])     # H = 0, S = 0 (faible saturation), V minimum pour éviter le noir
        haut_gris = np.array([50, 50, 200])  # H = 180 (max), S = faible, V max pour éviter le blanc

        # Créer un masque avec les plages définies
        masque = cv2.inRange(hsv, bas_gris, haut_gris)

        # Appliquer le masque à l'image originale
        resultat = cv2.bitwise_and(image, image, mask=masque)

        return masque, resultat
   
def main(args=None):
    rclpy.init(args=args)

    minimal_image_subscriber = MninimalImageSubscriber()

    rclpy.spin(minimal_image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
