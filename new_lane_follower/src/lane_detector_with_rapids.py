#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
from cuml.cluster import HDBSCAN
import numpy as np

class ImageConverter(Node):
    def __init__(self):
        super().__init__('detector')
        self.flag = Bool()
        # Subscribe to the RGB image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera_forward/image_raw',
            self.process_image,
            10)
   
    def process_image(self, msg):
        
            # Convert ROS image message to OpenCV image
            bridge = CvBridge()
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            # Convert RGB image to grayscale
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
            binary_image = cv2.inRange(gray_image, 120, 140)
            # binary_image2 = binary_image.copy()
            # binary_image[:binary_image.shape[0], :] = 0
            # binary_image2[:binary_image2.shape[0], :] = 0
            binary_image = cv2.medianBlur(binary_image, 5)
            binary_image2 = cv2.medianBlur(binary_image2, 5)
            # Find the indices of white pixels
            white_pixel_indices = np.argwhere(binary_image == 255)

            # print('Estimated no. of clusters: %d' % no_clusters)
            # print('Estimated no. of noise points: %d' % no_noise)
            db = HDBSCAN(min_samples=5)
            db.fit(white_pixel_indices);

            labels = db.labels_
            # probabilities = db.probabilities_
            
            
            
            print (labels)

#             # Determine the size of each cluster
            cluster_sizes = [np.sum(labels == label) for label in np.unique(labels) if label != -1]
#             cluster_sizes2 = [np.sum(labels2 == label) for label in np.unique(labels2) if label != -1]
            
            if cluster_sizes:
                self.flag.data = True
                largest_cluster_size = max(cluster_sizes)
                print('Size of the largest cluster: %d' % largest_cluster_size)
            else:
                self.flag.data = False
                print('No clusters found in the first binary image.')

            largest_clusters_indices = np.argsort(cluster_sizes)[-2:]

            clustered_image = np.zeros_like(cv_image)
     
                    
# 
            for label in np.unique(labels):
                
                if label == -1 or label not in largest_clusters_indices:
                    continue
                
                cluster_indices = white_pixel_indices[labels == label]
                for point in cluster_indices:
                    clustered_image[point[0], point[1]] = 255
                    
            cv2.imshow("window",clustered_image)
            cv2.waitKey(10)
       
        

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
