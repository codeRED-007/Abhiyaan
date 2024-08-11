#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
from skimage.filters import threshold_multiotsu
import numpy as np

class OtsuThresholdNode(Node):
    def __init__(self):
        super().__init__('otsu_threshold_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/zed/masked_image',
            self.image_callback,
            10
        )

    def image_callback(self, msg):
        try:
            # Convert the ROS Image message to a format OpenCV understands
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.apply_threshold(cv_image)
        except CvBridgeError as e:
            self.get_logger().error(f'CvBridge Error: {e}')

    def apply_threshold(self, cv_image):
        # Convert the image to grayscale
        gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # Apply multilevel Otsu thresholding
        thresholds = threshold_multiotsu(gray_image, classes=5)
        # Digitize the grayscale image based on thresholds
        segmented_image = np.digitize(gray_image, bins=thresholds)
        # Scale the segmented image for display
        segmented_image = segmented_image * (255 // segmented_image.max())
        segmented_image = segmented_image.astype(np.uint8)
        cv2.inRange(segmented_image,255,255,segmented_image)
        # Display the result
        cv2.imshow('Otsu Thresholding', segmented_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = OtsuThresholdNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
