#!/usr/bin/python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2


class LaneDetector(Node):
    def __init__(self):

        super().__init__('lanes_dilator')
        self.img_rgb_sub = self.create_subscription(
            Image,
            'igvc/lanes_binary',
            self.get_binary,
            10)
        self.img_rgb_sub
        
        self.lane_dilation = self.create_publisher(Image, 'igvc/lanes_dilated', 10)
        

        
    def get_binary(self, data):
        binary_image = CvBridge().imgmsg_to_cv2(data,desired_encoding="passthrough")
        kernel1 = np.ones((6, 6), np.uint8) 
        kernel2 = np.ones((6, 6), np.uint8) 
         
        img_dilation = cv2.dilate(binary_image, kernel2, iterations=1) 
        img_erosion = cv2.erode(img_dilation, kernel1, iterations=1) 

        self.lane_dilation.publish(CvBridge().cv2_to_imgmsg(img_erosion, encoding="mono8"))

    
        




def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    lane_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
