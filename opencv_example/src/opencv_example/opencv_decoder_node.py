#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import cv2

# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge 

class OpenCvDecoder(Node):

    def __init__(self):
        super().__init__('opencv_decoder_node')
        self.get_logger().info('Hello world from opencv_decoder_node !')

        # Create a subscriber on the topic "random_image"
        self.subscriber = self.create_subscription(Image, '/aquabot/sensors/cameras/main_camera_sensor/image_raw', self.image_callback, 10)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        # Used to decode QR codes
        self.qr_decoder = cv2.QRCodeDetector()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.br.imgmsg_to_cv2(msg)

        # Decode image
        data,bbox,rectifiedImage = self.qr_decoder.detectAndDecode(current_frame)

        if len(data) > 0:
            self.get_logger().info('Decoded data: ' + data)
        else:
            self.get_logger().info('No QR code detected')

def main(args=None):
    rclpy.init(args=args)

    opencv_decoder_node = OpenCvDecoder()

    rclpy.spin(opencv_decoder_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    opencv_decoder_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
