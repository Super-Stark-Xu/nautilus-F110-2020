#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from std_msgs.msg import Float64
from baskaran_wall_following.msg import error_analysis

cnt = 0
prev_avg = 0.0
max_error = 0.0

pub = rospy.Publisher('wall_following_analysis', error_analysis, queue_size=10)

def error_callback(msg):
	global cnt
	global prev_avg
	global max_error

	error = msg.data
	error = abs(error)

	if error > max_error:
		max_error = error

	avg_error = (cnt*prev_avg + error)/(cnt + 1)

	prev_avg = avg_error
	cnt += 1

	error_analysis_msg = error_analysis()
	error_analysis_msg.avg_error = avg_error
	error_analysis_msg.max_error = max_error
	pub.publish(error_analysis_msg)

if __name__ == '__main__':
 	rospy.init_node('baskaran_anaysis_node', anonymous = True)
	rospy.Subscriber("pid_error", Float64, error_callback)
	rospy.spin()
