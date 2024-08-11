#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from custom_msgs.msg import ArrayXY
import numpy as np

from sensor_msgs.msg import PointCloud2
from pcl_pub import PointCloudPublisher

class LanesCloud(Node):
    def __init__(self):
        super().__init__('lanes_to_cloud')
        self.xy_sub = self.create_subscription(
            ArrayXY,
            'igvc/lanes_xy_array',
            self.callback,
            10)
        self.xy_sub  # prevent unused variable warning
        self.lanes_pub = PointCloudPublisher("igvc/lanes_cloud")

    def callback(self,msg):
        self.lanes_pub.publish_cloud(msg.x,msg.y,np.zeros(len(msg.x)))


        

def main(args=None):
    rclpy.init(args=args)
    lanes_to_cloud = LanesCloud()

    rclpy.spin(lanes_to_cloud)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lanes_to_cloud.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



