#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('pixel_coord')
        self.subscription = self.create_subscription(Image, '/zed/zed_node/rgb/image_rect_color', self.image_callback, 10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the ROS Image message to a CV2 image
        image = self.bridge.imgmsg_to_cv2(msg)
        # Convert the image from RGB to HSV
        image = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        # Display the image
        cv2.imshow('Image', image)
        # Set mouse callback function
        cv2.setMouseCallback('Image', self.on_mouse, image)
        cv2.waitKey(1)

    def on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Extract HSV values from the clicked pixel
            hsv_value = self.get_intensity(param, x, y)
            # Convert HSV values from BGR format to float for printing
            h, s, v = hsv_value
            print(f"Clicked at pixel coordinates (x={x}, y={y}), HSV=(H={h}, S={s}, V={v})")

    def get_intensity(self, image, x, y):
        # Get the HSV values at the (x, y) position
        return image[y, x]

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
