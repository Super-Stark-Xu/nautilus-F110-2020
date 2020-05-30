#!/usr/bin/env python

import rospy
#from race.msg import drive_param
from nav_msgs.msg import Odometry
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

###########
# GLOBALS #
###########

# Import waypoints.csv into a list (path_points)
dirname = os.path.dirname(__file__)
filename = os.path.join(dirname, '../waypoints/waypoints_saver.csv')
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

#############
# FUNCTIONS #
#############

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
            if path_points[i-1] < path_points[i+1]:
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

##########
#    actual_point = [msg.pose.pose.position.x,msg.pose.pose.position.y] 
#    desired_point = path_points[path_idx]
#    num_points += 1
    #print("actual_point: ",actual_point)
    #print("desired_point: ",desired_point)

#    if path_idx >= len(path_points):
#        path_idx = 0
#    else:
#        path_idx += 1

#    error = np.append(error,dist(actual_point, desired_point))
############


    cur_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    p1, p2 = searchClosest(cur_point, path_points)
    h = height(cur_point, p1, p2)
    error = np.append(error,h)

    abs_error = np.absolute(error)
    abs_max = np.max(abs_error)
    print("abs_max: ",abs_max)
    max_errors = np.append(max_errors,abs_max)
    error = np.array([])

    max_val = np.max(max_errors)
    total_abs_max = total_abs_max + max_val
    print("total_abs_max: ",total_abs_max)
    csvname = os.path.join(dirname,"error_values.csv")
    append_to_csv(csvname, [abs_max,total_abs_max])





#    if (num_points % WINDOW_SIZE) == 0:
#        print("**** After window at: ",num_points," *****")
#        abs_error = np.absolute(error)
#        abs_max = np.max(abs_error)
#        print("abs_max: ",abs_max)
#        max_errors = np.append(max_errors,abs_max)
#        error = np.array([])

#        max_val = np.max(max_errors)
#        total_abs_max = total_abs_max + max_val
#        print("total_abs_max: ",total_abs_max)
#        csvname = os.path.join(dirname,"error_values.csv")
#        append_to_csv(csvname, [abs_max,total_abs_max])
	#df = pd.DataFrame(self.points_list, columns=['x', 'y', 'theta'])
	#df.to_csv('waypoints_saver.csv')

    #orientation = msg.pose.pose.orientation
    #msg = drive_param()
    #msg.velocity = VELOCITY
    #msg.angle = angle
    #pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('pure_pursuit_quality')
    rospy.Subscriber('/odom', Odometry, callback, queue_size=1)
    rospy.spin()
