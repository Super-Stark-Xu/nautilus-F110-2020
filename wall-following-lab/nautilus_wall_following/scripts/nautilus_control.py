#!/usr/bin/env python

import rospy
from race.msg import drive_param
from std_msgs.msg import Float64
import numpy as np
import math
import os

# TODO: modify these constants to make the car follow walls smoothly.

pid_gains = rospy.get_param('pid_gains')
KP, KI, KD = pid_gains['P'], pid_gains['I'], pid_gains['D']

# KP = 0.5
# KI = 0.00 #0.0001
# KD = 0.005 #0.001 - 0.005

# ALPHA = 0.7

prev_error = 0.00
sum_error = 0.00

pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

# Get the direction and velocity from the instructions.csv file
# def get_direction():
    # direction_possible = True # this should be from a function that checks the gap finding output
    # with open('instructions.csv', 'rb') as csvfile:
    #    direction_reader = csv.reader(csvfile, delimiter=' ')
    #    for row in direction_reader:
    #        print(', '.join(row))
    #        # Check if direction is applicable to turn
    #        print("direction_possible: ",direction_possible)
    #        if direction_possible == True:
    #            direction = row[0]
    #            velocity = row[1]
    #            print("get_direction will return: ",row[0]," ",row[1])
    #            return direction, velocity
    # return 'left', 3


# Callback for receiving PID error data on the /pid_error topic
# data: the PID error from pid_error_node, published as a Float64
def control_callback(msg):

    global sum_error
    global prev_error
    global prev_angle

    # TODO: Based on the error (msg.data), determine the car's required velocity and steering angle.
    error = msg.data
    sum_error = sum_error + error
    pid_output = KP*error + KI*sum_error + KD*(error - prev_error)/0.025  #TODO: compute pid response for steering angle based on the error
    prev_error = error

    angle = pid_output
    angle = np.clip(angle, -0.4189, 0.4189)  # 0.4189 radians = 24 degrees because car can only turn 24 degrees max

    ang_deg =  math.degrees(angle)

    T_junction_check = True
    vel = 0.5 # default

    # Car starts driving down center
    direction = rospy.get_param('direction')
    vel = rospy.get_param('velocity')
    
    print("Check Direction: ", direction)

    if abs(ang_deg) >= 0.0 and abs(ang_deg) <= 10.0:
        vel = vel

    elif abs(ang_deg) >=10 and abs(ang_deg) <= 20.0:
        vel = 1.0

    else:
        vel = 0.5

    if direction == 'stop':
        vel = 0.0
        angle = 0.0
    '''
    # Check for T-junction - this should be output from gap finding
    if T_junction_check == True:
        print("After t-junction check")
        # Take next direction from instructions.csv
        direction, vel = get_direction()
        print("after get_direction")
        if direction == 'center':
            rospy.set_param('mode', 'C')
        elif direction == 'right':
            rospy.set_param('mode', 'R')
        elif direction == 'left':
            rospy.set_param('mode', 'L')
        else:
            rospy.set_param('mode', 'C')
    else:
        if abs(ang_deg) >= 0.0 and abs(ang_deg) <= 10.0:
            vel = 1.5
        elif abs(ang_deg) >=10 and abs(ang_deg) <= 20.0:
            vel = 1.0
        else:
            vel = 0.5
    '''

    # Commit to do next turn --> send action/velocity
    msg = drive_param()
    msg.velocity = vel  # TODO: implement PID for velocity
    msg.angle = angle    # TODO: implement PID for steering angle
    pub.publish(msg)
    print("Error: {:0.2f}, PID o/p: {:0.2f}, Angle: {:0.2f}, Velocity: {:0.2f}".format(error, pid_output, ang_deg, vel))

# Boilerplate code to start this ROS node.
if __name__ == '__main__':
    rospy.init_node('pid_controller_node', anonymous=True)
    rospy.Subscriber("pid_error", Float64, control_callback)
    rospy.spin()
