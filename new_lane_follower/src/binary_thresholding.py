#!/root/anaconda3/envs/rapids-24.06/bin/python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import cudf
from cuml.cluster import HDBSCAN
import numpy as np

class ImageConverter(Node):
    def __init__(self):
        super().__init__('detector')
        self.flag = Bool()
        # Subscribe to the RGB image topic
        self.subscription = self.create_subscription(
            Image,
            '/zed/masked_image',
            self.process_image,
            10)
        self.publisher_ = self.create_publisher(Image, '/thresholded_image', 10)

    def process_image(self, msg):
            try:
        
                # Convert ROS image message to OpenCV image
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

                # Convert RGB image to grayscale
                gray_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2GRAY)
                binary_image = cv2.inRange(gray_image, 120, 140)
        
                # Adding Median Blur
                binary_image = cv2.medianBlur(binary_image, 5)
                
                
                # Find the indices of white pixels
                white_pixel_indices = np.argwhere(binary_image == 255)
                
                gdf_data = cudf.DataFrame.from_records(white_pixel_indices)
                # DBSCAN from cuML
                db = HDBSCAN(min_samples=5)
                db.fit(gdf_data)

                # Getting labels for white pixels
                labels = db.labels_.to_numpy()
    #           # Determine the size of each cluster
                cluster_sizes = [np.sum(labels == label) for label in np.unique(labels) if label != -1]
                
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
                
                image_msg = bridge.cv2_to_imgmsg(clustered_image,"bgr8")
                self.publisher_.publish(image_msg)

            except:
                print("nigga")
       
        

def main(args=None):
    rclpy.init(args=args)
    image_converter = ImageConverter()
    rclpy.spin(image_converter)
    image_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()