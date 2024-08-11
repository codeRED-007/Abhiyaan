#!/usr/bin/python3
# import sensor_msgs.point_cloud2
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
# from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from robot_navigator import BasicNavigator, NavigationResult
# import tf_transformations
import numpy as np
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
        self.create_subscription(Odometry, '/zed/zed_node/transformed_odom', self.odom, 10)
            
    def odom(self, msg):
       
        global odom, odomflag
        if odomflag == False:
            print ("odom subscriber")
            odom = msg
            odomflag = True

class Goal_Pose_Subscriber(Node):
        def __init__(self):
            super().__init__('goal_pose_subscriber')
            self.create_subscription(Float32MultiArray, '/far_ipm', self.goal, 10)
        def goal(self, msg):
            global new_pos, flag, posflag         
            new_pos = msg
            print ("goal subscriber")
            posflag = True
            

        


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
        
        
        self.goal_pose.pose.position.x = pos.data[0] + odom.pose.pose.position.x
        self.goal_pose.pose.position.y = pos.data[1] + odom.pose.pose.position.y
        self.goal_pose.pose.position.z = pos.data[2] + odom.pose.pose.position.z
        self.goal_pose.pose.orientation.x = odom.pose.pose.orientation.x
        self.goal_pose.pose.orientation.y = odom.pose.pose.orientation.y
        self.goal_pose.pose.orientation.z = odom.pose.pose.orientation.z
        self.goal_pose.pose.orientation.w = odom.pose.pose.orientation.w
        print("navigating to goal")
        self.navigator.goToPose(self.goal_pose)
        if (self.navigator.isNavComplete()) :
            flag = True

        


       




def main(args=None):
    global flag, odom, pos, odomflag, posflag, new_pos
    rclpy.init(args=args)
    flag = True
    odom_subscriber = Odom_Subscriber()
    goal_pos_subscriber = Goal_Pose_Subscriber()
    lane_follower = LaneFollower()
    goal_threshold = 0.5
    posflag = False
    rclpy.spin_once(goal_pos_subscriber)
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
            else:
                print("goal pose not received")
                continue

        pos_list = np.array([pos.data[0],pos.data[1],pos.data[2]])
        if (np.linalg.norm(pos_list-current_pos) < goal_threshold) or (lane_follower.navigator.isNavComplete()) :
            print ("threshold reached")
            flag = True


        

if __name__ == '__main__':
    main()
