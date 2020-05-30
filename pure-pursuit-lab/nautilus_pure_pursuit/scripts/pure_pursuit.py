#!/usr/bin/env python

import rospy
from race.msg import drive_param
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import copy

import math
import numpy as np
from numpy import linalg as LA
import tf 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 
import time

class PurePursuit:
    def __init__(self, filepath="DEFAULT"):
        self.LOOKAHEAD_DISTANCE = 1.5 #meters
        self.TIME_H = 0.25       # sec

        self.THETA_TOLERANCE = math.radians(15)
        self.L_TOLERANCE = 0.05 # 5cm
    
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/waypoints_saver.csv')
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        
        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]

        self.start_time = time.time()
        self.tf_lis = tf.TransformListener()

        # Publisher for 'drive_parameters' (speed and steering angle)
        self.des_pub = rospy.Publisher('desired_point', PoseStamped, queue_size=1)
        self.cur_pub = rospy.Publisher('current_point', PoseStamped, queue_size=1)
        self.path_pub = rospy.Publisher('/waypoints_path', Path, queue_size=1)
        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

        # Subscriber for odom
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        # self.pf_sub = rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.pf_callback)

    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def update_lookahead(self, angle):
        ang_deg =  math.degrees(angle)
        if abs(ang_deg) >= 0.0 and abs(ang_deg) <= 10.0:
            self.LOOKAHEAD_DISTANCE = 1.5
            vel = 1.5

        elif abs(ang_deg) > 10.0 and abs(ang_deg) <= 20.0:
            self.LOOKAHEAD_DISTANCE = 1.0
            vel = 1.0

        else:
            self.LOOKAHEAD_DISTANCE = 0.5
            vel = 0.5
        return vel

    def searchClosestFront(self, cur_pos, path_points):
        min_dist = 1e10
        desired_point = None
        for i in range(len(path_points)):
            path_point = path_points[i]
            L = self.dist(cur_pos, path_point)
            if (L >= 1.0*self.LOOKAHEAD_DISTANCE): #if (path_point[0] - cur_pos[0] > 0 and L >= 0.75*self.LOOKAHEAD_DISTANCE):
                dist_diff = abs(L-self.LOOKAHEAD_DISTANCE)
                if  dist_diff < min_dist:
                    min_dist = dist_diff
                    desired_point = path_point

        des_pose = PoseStamped()
        des_pose.header.stamp = rospy.Time.now()
        des_pose.header.frame_id = "map"
        des_pose.pose.position.x = desired_point[0]
        des_pose.pose.position.y = desired_point[1]
        des_pose.pose.position.z = 0.0
        quat = quaternion_from_euler(0.0, 0.0, desired_point[2])
        des_pose.pose.orientation.x = quat[0]
        des_pose.pose.orientation.y = quat[1]
        des_pose.pose.orientation.z = quat[2]
        des_pose.pose.orientation.w = quat[3]

        self.tf_lis.waitForTransform("/map", "/base_link", des_pose.header.stamp, rospy.Duration(0.5))
        target_pose = self.tf_lis.transformPose("/base_link", des_pose)

        return desired_point, des_pose, target_pose

    def PP_planner(self, cur_pose):
        # Plan for a horizon
        # if (time.time() - self.start_time > self.TIME_H):
        # 1. Determine the current location of the vehicle (we are subscribed to /odom)
        # Hint: Read up on Odometry message type in ROS to determine how to extract x, y, and yaw. Make sure to convert quaternion to euler angle.
        
        # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
        x = cur_pose.pose.position.x
        y = cur_pose.pose.position.y
        quaternion = cur_pose.pose.orientation
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        yaw = euler_from_quaternion(quat)[2]
        cur_pos = (x,y,yaw)

        found = False
        path_points = copy.deepcopy(self.path_points)
        while (found == False):
            desired_point, des_pose, target_pose = self.searchClosestFront(cur_pos, path_points)
            path_points.remove(desired_point)

            if target_pose.pose.position.x > 0:
                found = True

        # Rviz the desired point
        self.des_pub.publish(des_pose)

        # Rviz the path
        waypoints = Path()
        waypoints.header.stamp = rospy.Time.now()
        waypoints.header.frame_id = "map"
        waypoints.poses.append(cur_pose)
        waypoints.poses.append(des_pose)
        self.path_pub.publish(waypoints)

        # 4. Calculate the curvature = 1/r = 2x/l^2
        # The curvature is transformed into steering wheel angle and published to the 'drive_param' topic.
        x = target_pose.pose.position.x
        y = target_pose.pose.position.y
#############################
#        L2 = self.LOOKAHEAD_DISTANCE*self.LOOKAHEAD_DISTANCE
#	angle = (2*abs(y))/L2
############################

	distance = self.dist(cur_pos, desired_point)
	L2 = distance*distance
	kappa = (2*abs(y))/L2	#kappa = 1/radis
	wheelbase = 0.3302
        angle = np.arctan(wheelbase*kappa)
        
        if (y < 0):
            angle = -angle # Right turn
        else:
            angle = angle # Left turn

        angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
        velocity = self.update_lookahead(angle)
        
        msg = drive_param()
        msg.velocity = velocity
        msg.angle = angle
        self.pub.publish(msg)
        self.start_time = time.time()

    def odom_callback(self, odom_msg):
        cur_pose = PoseStamped()
        cur_pose.header.stamp = rospy.Time.now()
        cur_pose.header.frame_id = "map"
        cur_pose.pose = odom_msg.pose.pose
        self.cur_pub.publish(cur_pose)
        self.PP_planner(cur_pose)

    def pf_callback(self, pose_msg):
        x = pose_msg.pose.position.x
        y = pose_msg.pose.position.y
        quaternion = pose_msg.pose.orientation
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        yaw = euler_from_quaternion(quat)[2]
        cur_pos = (x,y,yaw)
        self.PP_planner(cur_pos)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    pp_obj = PurePursuit()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down PurePursuit...")


