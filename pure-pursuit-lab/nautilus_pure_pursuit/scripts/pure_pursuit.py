#!/usr/bin/env python

import rospy
from race.msg import drive_param
from nav_msgs.msg import Odometry
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os 

class PurePursuit:
    def __init__(self, filepath="DEFAULT"):
        self.LOOKAHEAD_DISTANCE = 2.0 #meters
        self.VELOCITY = 1.0 #m/s

        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../waypoints/levine-waypoints.csv')
        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
                
        # Publisher for 'drive_parameters' (speed and steering angle)
        self.pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

        # Subscriber for odom
        self.sub = rospy.Subscriber('/odom', Odometry, self.callback, queue_size=1)

    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Input data is PoseStamped message from topic /odom.
    # Runs pure pursuit and publishes velocity and steering angle.
    def callback(self, msg):

        # Note: These following numbered steps below are taken from R. Craig Coulter's paper on pure pursuit.

        # 1. Determine the current location of the vehicle (we are subscribed to /odom)
        # Hint: Read up on Odometry message type in ROS to determine how to extract x, y, and yaw. Make sure to convert quaternion to euler angle.
        

        # 2. Find the path point closest to the vehicle that is >= 1 lookahead distance from vehicle's current location.


        # 3. Transform the goal point to vehicle coordinates. 
        
        
        # 4. Calculate the curvature = 1/r = 2x/l^2
        # The curvature is transformed into steering wheel angle and published to the 'drive_param' topic.

        angle = np.clip(angle, -0.4189, 0.4189) # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

        msg = drive_param()
        msg.velocity = self.VELOCITY
        msg.angle = angle
        self.pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    pp_obj = PurePursuit()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down PurePursuit...")


