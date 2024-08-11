#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVThresholdingNode(Node):
    def __init__(self):
        super().__init__('hsv_thresholding_node')

        # Parameters
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')
        self.declare_parameter('output_topic', '/zed/zed_node/rgb/camera_info')
        self.declare_parameter('hue_min', 0)
        self.declare_parameter('hue_max', 30)
        self.declare_parameter('saturation_min', 0)
        self.declare_parameter('saturation_max', 100)
        self.declare_parameter('value_min', 200)
        self.declare_parameter('value_max', 255)

        # Get parameters
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        hue_min = self.get_parameter('hue_min').get_parameter_value().integer_value
        hue_max = self.get_parameter('hue_max').get_parameter_value().integer_value
        saturation_min = self.get_parameter('saturation_min').get_parameter_value().integer_value
        saturation_max = self.get_parameter('saturation_max').get_parameter_value().integer_value
        value_min = self.get_parameter('value_min').get_parameter_value().integer_value
        value_max = self.get_parameter('value_max').get_parameter_value().integer_value

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Create subscription and publisher
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(CompressedImage, output_topic, 10)

        # HSV Thresholding parameters
        self.hue_min = hue_min
        self.hue_max = hue_max
        self.saturation_min = saturation_min
        self.saturation_max = saturation_max
        self.value_min = value_min
        self.value_max = value_max

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define HSV thresholds
        lower_bound = np.array([0, 0, 200])
        upper_bound = np.array([35, 100, 255])

        # Apply HSV thresholding
        mask = cv2.inRange(hsv_image, lower_bound, upper_bound)
        result_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Convert result image to ROS message
        # compressed_image = self.bridge.cv2_to_compressed_imgmsg(result_image)

        cv2.imshow('Image',result_image)
        cv2.waitKey(1)

        # Publish the thresholded image
        # self.publisher.publish(compressed_image)

def main(args=None):
    rclpy.init(args=args)
    node = HSVThresholdingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
