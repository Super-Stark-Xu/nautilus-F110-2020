#!/usr/bin/env python

import rospy
from race.msg import drive_param
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from std_msgs.msg import String
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import copy

import math
import numpy as np
from numpy import linalg as LA
import tf 
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 
import time

MAX_VEL = rospy.get_param('max_velocity')
MIN_VEL = rospy.get_param('min_velocity')
L_CONST = rospy.get_param('l_const')
CSV = rospy.get_param('csv_file')

class FinalRace:
    def __init__(self, filepath="DEFAULT"):
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/' + CSV)
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]
        
        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
        self.tf_lis = tf.TransformListener()

        # Publisher for 'drive_parameters' (speed and steering angle)
        self.path_pub = rospy.Publisher('waypoints_path', Path, queue_size=1)
        self.drive_pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)
        self.key_pub = rospy.Publisher('key', String, queue_size=1)
        self.initialpose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.text_marker_pub = rospy.Publisher('display_param', Marker, queue_size=1)

        self.idx = 0
        self.chk_idx = [0, 700, 1750, 2300, 3000, 3700]
        self.m = (MIN_VEL - MAX_VEL)/24.0
        self.c = MAX_VEL
        self.LOOKAHEAD_DISTANCE = 2.0 #meters
        self.initialpose_publish()
        self.key_publish()

        # Subscriber for odom
        self.reset_sub = rospy.Subscriber('/reset_car', Bool, self.reset_callback, queue_size=1 )
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=1)
        # self.pf_sub = rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.pf_callback)
        # self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback, queue_size=1)
        
    def initialpose_publish(self):
        rospy.sleep(0.75)
        pose_cov_msg = PoseWithCovarianceStamped()
        pose_cov_msg.header.stamp = rospy.Time.now()
        pose_cov_msg.header.frame_id = "map"
        pose_cov_msg.pose.pose.position.x = 0.0
        pose_cov_msg.pose.pose.position.y = 0.0
        pose_cov_msg.pose.pose.position.z = 0.0
        pose_cov_msg.pose.pose.orientation.x = 0.0
        pose_cov_msg.pose.pose.orientation.y = 0.0
        pose_cov_msg.pose.pose.orientation.z = 0.0
        pose_cov_msg.pose.pose.orientation.w = 1.0
        pose_cov_msg.pose.covariance = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.06]
        self.initialpose_pub.publish(pose_cov_msg)

    def key_publish(self):
        key_msg = String()
        key_msg.data = 'n'
        self.key_pub.publish(key_msg)

    def display_param(self, drive_msg, cur_pos):
        text_marker = Marker()
        text_marker.type = text_marker.TEXT_VIEW_FACING
        text_marker.header.frame_id = 'map'
        text_marker.scale.x = 1.0
        text_marker.scale.y = 1.0
        text_marker.scale.z = 1.0
        text_marker.color.a = 1.0
        text_marker.color.r = 0.0    
        text_marker.pose.position.x = -6.388 # cur_pos[0]
        text_marker.pose.position.y = -9.032 # cur_pos[1]
        text_marker.pose.position.z = 0.0
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        ang_deg = str(round(math.degrees(drive_msg.angle), 1))
        velocity = str(round(drive_msg.velocity, 1))
        text_msg = "Velocity: " + velocity + "\nSteering: " + ang_deg
        text_marker.text =  text_msg
        self.text_marker_pub.publish(text_marker)

    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    def update_lookahead(self, angle):
        ang_deg =  math.degrees(angle)
        if abs(ang_deg) >= 0.0 and abs(ang_deg) <= 10.0:
            self.LOOKAHEAD_DISTANCE = 2.0
            
        elif abs(ang_deg) > 10.0 and abs(ang_deg) <= 20.0:
            self.LOOKAHEAD_DISTANCE = 1.5
            
        else:
            self.LOOKAHEAD_DISTANCE = 1.0
            
        vel = self.m*abs(ang_deg) + self.c
        return vel

    def TransformPoint(self, desired_point):
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

    def searchNew(self, cur_pos, path_points):
        min_dist = 1e10
        desired_point = None
        N = len(path_points)
        if (N - self.idx) < 25: # reset idx
            self.idx = 0
        start_idx = self.idx
        for i in range(start_idx, N):
            path_point = path_points[i]
            L = self.dist(cur_pos, path_point)

            if (L >= self.LOOKAHEAD_DISTANCE) and (L <= L_CONST*self.LOOKAHEAD_DISTANCE):
                dist_diff = abs(L-self.LOOKAHEAD_DISTANCE)
                if  dist_diff < min_dist:
                    min_dist = dist_diff
                    desired_point = path_point
                    self.idx = i

            if (L > L_CONST*self.LOOKAHEAD_DISTANCE): # Prune Search Tree
                break
        return desired_point
    
    def reset_idx(self, cur_idx):
        close = 1e5
        des_idx = 0
        for idx in self.chk_idx:
            if idx > cur_idx:
                continue

            dist = cur_idx - idx
            if dist < close:
                close = dist
                des_idx = idx
        return des_idx

    def PP_planner(self, cur_pose):
        # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.
        x = cur_pose.pose.position.x
        y = cur_pose.pose.position.y
        quaternion = cur_pose.pose.orientation
        quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        yaw = euler_from_quaternion(quat)[2]
        cur_pos = (x,y,yaw)

        # new search method
        desired_point = None
        while(desired_point is None):
            desired_point = self.searchNew(cur_pos, self.path_points)
            if desired_point == None:
                rospy.sleep(0.1)
                self.idx = self.reset_idx(self.idx)
        desired_point, des_pose, target_pose = self.TransformPoint(desired_point)

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

        distance = self.dist(cur_pos, desired_point)
        L2 = distance*distance
        kappa = (2*y)/L2    #kappa = 1/radius
        wheelbase = 0.5 #0.35
        angle = np.arctan(wheelbase*kappa)

        angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max
        velocity = self.update_lookahead(angle)
    
        # velocity_tmp = velocity
        # velocity_idx = self.idx
        # if velocity_idx > 1664 and velocity_idx < 2200: # max speed on straight line
        #     velocity = 10
        #     if velocity_idx > 2200 and velocity_idx < 2300: # break before curve
        #         velocity = (MAX_VEL - 10)*(velocity_idx - 2200)/100 + MAX_VEL

        # if self.idx > 4099 or self.idx < 150:
        #     velocity = 10
        
        msg = drive_param()
        msg.velocity = velocity
        msg.angle = angle
        self.drive_pub.publish(msg)
        self.display_param(msg, cur_pos)

    def odom_callback(self, odom_msg):
        cur_pose = PoseStamped()
        cur_pose.header.stamp = rospy.Time.now()
        cur_pose.header.frame_id = "map"
        cur_pose.pose = odom_msg.pose.pose
        self.PP_planner(cur_pose)

    def reset_callback(self, reset_msg):
        if reset_msg.data == True:
            rospy.sleep(0.5)
            self.key_publish()
            self.idx = self.reset_idx(self.idx)

    def pf_callback(self, pose_msg):
        self.PP_planner(pose_msg)

if __name__ == '__main__':
    rospy.init_node('nautilus_final_race')
    fr_obj = FinalRace()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down FinalRace...")


