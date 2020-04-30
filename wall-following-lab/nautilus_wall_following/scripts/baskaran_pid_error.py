#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import pdb
import csv

pub = rospy.Publisher('pid_error', Float64, queue_size=10)

# You can define constants in Python as uppercase global names like these.
MIN_DISTANCE = 0.1
MAX_DISTANCE = 30.0
MIN_ANGLE = -45.0
MAX_ANGLE = 225.0

THETA = rospy.get_param('theta')
L_DIST = rospy.get_param('lookahead')
TARGET_DIST = rospy.get_param('target_distance')
MODE = rospy.get_param('mode')

SCAN_LIMIT = 1
scan_vector = []

def smoothScan(data):
    global scan_vector
    scan_vector.append(data.ranges)
    if len(scan_vector) > SCAN_LIMIT:
        scan_vector.pop(0)

    scan_vector = np.array(scan_vector)
    smooth_ranges = np.mean(scan_vector, axis=0)
    scan_vector = scan_vector.tolist()
    smooth_ranges = smooth_ranges.tolist()
    return smooth_ranges

# data: single message from topic /scan
# angle: between -45 to 225 degrees, where 0 degrees is directly to the right
# Outputs length in meters to object with angle in lidar scan field of view
def getRange(data, angle):
    laser_deg = angle - 90
    laser_rad = np.deg2rad(laser_deg)
    laser_idx = int((laser_rad - data.angle_min)/data.angle_increment)
    ranges = smoothScan(data)
    laser_range = ranges[laser_idx]
    return laser_range
# data: single message from topic /scan
# desired_distance: desired distance to the right wall [meters]
# Outputs the PID error required to make the car follow the right wall.
def followRight(data):
 	a = getRange(data, THETA)
 	b = getRange(data, 0)
 	theta = np.deg2rad(THETA)

 	tan_alpha = (a*math.cos(theta) - b)/(a*math.sin(theta))
 	alpha = math.atan(tan_alpha)
 	d_t = b*math.cos(alpha) + L_DIST*math.sin(alpha)
 	return d_t

# data: single message from topic /scan
# desired_distance: desired distance to the left wall [meters]
# Outputs the PID error required to make the car follow the left wall.
def followLeft(data):
 	a = getRange(data, 180-THETA)
 	b = getRange(data, 180-0)
 	theta = np.deg2rad(THETA)

 	tan_alpha = (a*math.cos(theta) - b)/(a*math.sin(theta))
 	alpha = math.atan(tan_alpha)
 	d_t = b*math.cos(alpha) + L_DIST*math.sin(alpha)
 	return d_t

# data: single message from topic /scan
# Outputs the PID error required to make the car drive in the middle
# of the hallway.
def followCenter(data):
 	a = getRange(data, THETA)
 	b = getRange(data, 0)
 	theta = np.deg2rad(THETA)
 	tan_alpha = (a*math.cos(theta) - b)/(a*math.sin(theta))
 	alpha = math.atan(tan_alpha)

 	left_dist = followLeft(data)
 	right_dist = followRight(data)
 	e_t = left_dist - right_dist - L_DIST*math.sin(alpha)
 	return e_t

# Callback for receiving LIDAR data on the /scan topic.
# data: the LIDAR data, published as a list of distances to the wall.
def scan_callback(data):
    error_left = followLeft(data)  - TARGET_DIST
    error_right = TARGET_DIST - followRight(data)
    error_center = followCenter(data)

    MODE = rospy.get_param('mode')
    print("***Mode: ",MODE)
    if MODE == 'L':
 	    error = error_left
    elif MODE =='R':
 	   error = error_right
    elif MODE == 'C':
 	   error = error_center
    else:
        print("Invalid choice!")
        rospy.on_shutdown()

    msg = Float64()
    msg.data = error
    pub.publish(msg)

# Boilerplate code to start this ROS node.
# DO NOT MODIFY!
if __name__ == '__main__':
    rospy.init_node('pid_error_node', anonymous = True)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.spin()
