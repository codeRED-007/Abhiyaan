#!/usr/bin/python3
# import sensor_msgs.point_cloud2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
# from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult
# import tf_transformations
import numpy as np
import tf_transformations
# import time

# Global variables
global pos, posflag, odom, odomflag, start_time, end_time, flag
pos = Point()
odom = Odometry()
odomflag = False
posflag= False


# class Subscriber(Node):
#     def __init__(self):
#         super().__init__('lf')
#         global modoflag
#         modoflag == False
#         self.create_subscription(Float32MultiArray, '/far_ipm', self.goal, 10)
#         self.create_subscription(Odometry, '/zed/zed_node/transformed_odom', self.odom, 10)
        

#     def odom(self, msg):
       
#         global modo, modoflag
#         if modoflag == False:
            
#             modo = msg
#             modoflag = True
            

#     def goal(self, msg):
#         global pos, start_time
#         # if msg is None:
#             # Aligner().run()
#         pos = msg
        
#         # LaneFollower().run()


class Odom_Subscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.create_subscription(Odometry, '/odom', self.odom, 10)
            
    def odom(self, msg):
       
        global odom, odomflag
        if odomflag == False:
            print ("odom receieved")
            odom = msg
            odomflag = True

class Goal_Pose_Subscriber(Node):
        def __init__(self):
            super().__init__('goal_pose_subscriber')
            self.create_subscription(Float32MultiArray, '/goal_vec', self.goal, 10)
            self.publisher = self.create_publisher(PointCloud2,'goal',10)
        def goal(self, msg):
            global new_pos, flag, posflag         
            new_pos = msg
            print ("goal received")
            posflag = True
        def goal_publisher(self,pointcloud):
            self.publisher.publish(pointcloud)
            

        


class LaneFollower:
    def __init__(self):
        self.navigator = BasicNavigator()

    def run(self):
        global pos, odom, odomflag, start_time, end_time, flag
        flag = False
        # self.navigator.waitUntilNav2Active()
        
            
        self.goal_pose = PoseStamped()
        self.goal_pose.header.frame_id = 'map' # changed from odom to map frame
        self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

        # Extract the position and orientation from odometry
        position = np.array([odom.pose.pose.position.x, 
                            odom.pose.pose.position.y, 
                            odom.pose.pose.position.z])
        
        orientation = odom.pose.pose.orientation
        quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Convert quaternion to rotation matrix
        rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
        
        # Convert goal point from base_link frame to map frame
        goal_in_base_link = np.array([pos.data[0], pos.data[1], pos.data[2]])  
        goal_in_map_frame = np.dot(rotation_matrix, goal_in_base_link) + position

        # Prepare the PointCloud2 message
        points = np.array([goal_in_map_frame])
        ros_dtype = PointField.FLOAT32
        dtype = np.float32
        
        itemsize = np.dtype(dtype).itemsize
        data = points.astype(dtype).tobytes()
        fields = [PointField(
            name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
            for i, n in enumerate('xyz')]
        header = Header(frame_id='map')

        self.pointcloud_msg = PointCloud2(
                            header=header,
                            height=1,
                            width=points.shape[0],
                            is_dense=False,
                            is_bigendian=False,
                            fields=fields,
                            point_step=(itemsize * 3),  # Every point consists of three float32s.
                            row_step=(itemsize * 3 * points.shape[0]),
                            data=data
                        )
        
        
        
        
        self.goal_pose.pose.position.x = goal_in_map_frame[0]
        self.goal_pose.pose.position.y = goal_in_map_frame[1]
        self.goal_pose.pose.position.z = goal_in_map_frame[2]
        self.goal_pose.pose.orientation.x = odom.pose.pose.orientation.x
        self.goal_pose.pose.orientation.y = odom.pose.pose.orientation.y
        self.goal_pose.pose.orientation.z = odom.pose.pose.orientation.z
        self.goal_pose.pose.orientation.w = odom.pose.pose.orientation.w
        print("navigating to goal")
        self.navigator.goToPose(self.goal_pose)
        if (self.navigator.isNavComplete()) :
            flag = True

        


# class Goal_Publisher(Node):
#         def __init__(self):
#             super().__init__('goal_pose_publisher')
#             self.publisher = self.create_publisher(PointCloud2, '/goal_points', 10)
#         def publish(self, goal_pose):
#             global odom

#             # Extract the position and orientation from odometry
#             position = np.array([odom.pose.pose.position.x, 
#                                 odom.pose.pose.position.y, 
#                                 odom.pose.pose.position.z])
            
#             orientation = odom.pose.pose.orientation
#             quaternion = [orientation.x, orientation.y, orientation.z, orientation.w]
            
#             # Convert quaternion to rotation matrix
#             rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]
            
#             # Convert goal point from base_link frame to map frame
#             goal_in_base_link = np.array([pos.data[0], pos.data[1], pos.data[2]])  
#             goal_in_map_frame = np.dot(rotation_matrix, goal_in_base_link) + position
            
#             # Prepare the PointCloud2 message
#             points = np.array([goal_in_map_frame])
#             ros_dtype = PointField.FLOAT32
#             dtype = np.float32
            
#             itemsize = np.dtype(dtype).itemsize
#             data = points.astype(dtype).tobytes()
#             fields = [PointField(
#                 name=n, offset=i * itemsize, datatype=ros_dtype, count=1)
#                 for i, n in enumerate('xyz')]
#             header = Header(frame_id='map')

#             pointcloud_msg = PointCloud2(
#                                 header=header,
#                                 height=1,
#                                 width=points.shape[0],
#                                 is_dense=False,
#                                 is_bigendian=False,
#                                 fields=fields,
#                                 point_step=(itemsize * 3),  # Every point consists of three float32s.
#                                 row_step=(itemsize * 3 * points.shape[0]),
#                                 data=data
#                             )
#             self.publisher.publish(pointcloud_msg)





def main(args=None):
    global flag, odom, pos, odomflag, posflag, new_pos
    rclpy.init(args=args)
    flag = True
   
    odom_subscriber = Odom_Subscriber()
    goal_pos_subscriber = Goal_Pose_Subscriber()
    # goal_publisher = Goal_Publisher()
    lane_follower = LaneFollower()
    goal_threshold = 0.5
    posflag = False
    print("getting goal pose")
    rclpy.spin_once(goal_pos_subscriber)
    print ("Waiting for NAV2")
    lane_follower.navigator.waitUntilNav2Active()
    while True:
        rclpy.spin_once(odom_subscriber)
        rclpy.spin_once(goal_pos_subscriber)
        if odomflag:
            current_pos = np.array([odom.pose.pose.position.x,odom.pose.pose.position.y,odom.pose.pose.position.z])
            odomflag = False
        else: 
            print("Odom not Received")
            while (not odomflag):
                rclpy.spin_once(odom_subscriber)
            continue
        
        if flag :
            pos = new_pos
            if (posflag and (pos.data[0]> 0 ) ):
                posflag = False
                lane_follower.run()
                # goal_pos_subscriber.goal_publisher(lane_follower.pointcloud_msg)
                # goal_publisher.publish(lane_follower.goal_pose)

            else:
                print("goal pose not received")
                continue

        pos_list = np.array([pos.data[0],pos.data[1],pos.data[2]])
        if (np.linalg.norm(pos_list-current_pos) < goal_threshold) or (lane_follower.navigator.isNavComplete()) :
            print ("threshold reached")
            flag = True


        

if __name__ == '__main__':
    main()
