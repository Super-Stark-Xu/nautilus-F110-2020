#!/usr/bin/env python

import rospy
#from race.msg import drive_param
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import math
import numpy as np
from numpy import linalg as LA
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import csv
import os
from nav_msgs.msg import Path
from csv import writer

#############
# CONSTANTS #
#############

WINDOW_SIZE = 100
CSV = rospy.get_param('csv_file')

###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/' + CSV)
with open(filename) as f:
    path_points = [tuple(line) for line in csv.reader(f)]

# Turn path_points into a list of floats to eliminate the need for casts in the code below.
path_points = [(float(point[0]), float(point[1]), float(point[2])) for point in path_points]
path_idx = 0
num_points = 0
total_abs_max = 0
error = np.array([])
max_errors = np.array([])

# Publisher for 'drive_parameters' (speed and steering angle)
#pub = rospy.Publisher('pursuit_quality', float, queue_size=1)
text_marker_pub = rospy.Publisher('error_param', Marker, queue_size=1)

#############
# FUNCTIONS #
#############

def error_param(abs_max, total_abs_max):
    text_marker = Marker()
    text_marker.type = text_marker.TEXT_VIEW_FACING
    text_marker.header.frame_id = 'map'
    text_marker.scale.x = 1.0
    text_marker.scale.y = 1.0
    text_marker.scale.z = 1.0
    text_marker.color.a = 1.0
    text_marker.color.r = 0.5
    text_marker.pose.position.x = -9.30 # cur_pos[0]
    text_marker.pose.position.y = -9.578 # cur_pos[1]
    text_marker.pose.position.z = 0.0
    text_marker.pose.orientation.x = 0.0
    text_marker.pose.orientation.y = 0.0
    text_marker.pose.orientation.z = 0.0
    text_marker.pose.orientation.w = 1.0
    abs_max = str(round(abs_max, 1))
    total_abs_max = str(round(total_abs_max, 1))
    text_msg = "Max. Absolute Error: " + abs_max + "\nTotal Absolute Error: " + total_abs_max
    text_marker.text = text_msg
    text_marker_pub.publish(text_marker)

# Computes the Euclidean distance between two 2D points p1 and p2.
def dist(p1, p2):
    return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

def searchClosest(cur_point, path_points):
    min_dist = 1e10
    desired_point = None
    for i in range(len(path_points)):
        path_point = path_points[i]
	dist_diff = dist(cur_point, path_point)
	if  dist_diff < min_dist:
            min_dist = dist_diff
            p1 = path_point
	    if i == 0:
	 	p2 = path_points[i+1]
	    elif i == len(path_points)-1:
		p2 = path_points[i-1]
            else:
	    	if dist(cur_point, path_points[i-1]) < dist(cur_point, path_points[i+1]):
		    p2 = path_points[i-1]
	        else:
		    p2 = path_points[i+1]
    return p1, p2

def height(cur_point, p1, p2):
    a = dist(cur_point, p1)
    b = dist(p1, p2)
    c = dist(cur_point, p2)
    gamma = np.arccos((a*a + b*b - c*c)/2*a*b)
    h = np.sin(a)
    return h

def append_to_csv(filename, error_data):
    # Open file in append mode
    with open(filename, "a") as write_obj:
        # Create a writer object from csv module
        csv_writer = writer(write_obj)
        # Add error values as last row in the csv file
        csv_writer.writerow(error_data)

# Input data is PoseStamped message from topic /odom.
# Runs pure pursuit and publishes velocity and steering angle.
def callback(msg):
    global path_points
    global path_idx
    global num_points
    global error
    global max_errors
    global total_abs_max

    cur_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    p1, p2 = searchClosest(cur_point, path_points)
    h = height(cur_point, p1, p2)
    error = np.append(error,h)

    abs_error = np.absolute(error)
    abs_max = np.max(abs_error)
    max_errors = np.append(max_errors,abs_max)
    error = np.array([])

    max_val = np.max(max_errors)
    total_abs_max = total_abs_max + max_val
    csvname = os.path.join(dirname,"error_values.csv")
    append_to_csv(csvname, [abs_max,total_abs_max])

    # print("total_abs_max: ",total_abs_max)
    # print("abs_max: ",abs_max)
    error_param(abs_max, total_abs_max)

if __name__ == '__main__':
    rospy.init_node('final_race_quality')
    rospy.sleep(1.0)
    rospy.Subscriber('/pf/pose/odom', Odometry, callback, queue_size=1) # Changed the topic to pf/odom
    rospy.spin()
