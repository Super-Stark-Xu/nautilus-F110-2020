#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64

import tf
import pandas as pd 
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class waypointsSaver:
	def __init__(self):
		self.points_list = []
		self.waypoints = Path()
		rospy.on_shutdown(self.save_csv)

		self.path_pub = rospy.Publisher('/waypoints_path', Path, queue_size=1)
		self.pf_sub = rospy.Subscriber('/pf/viz/inferred_pose', PoseStamped, self.pf_callback)

	def pf_callback(self, pose_msg):
		x = pose_msg.pose.position.x
		y = pose_msg.pose.position.y
		quaternion = pose_msg.pose.orientation
		quat = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
		yaw = tf.transformations.euler_from_quaternion(quat)[2]
		waypoint_tup = (x,y,yaw)
		self.points_list.append(waypoint_tup)

		self.waypoints.header.stamp = rospy.Time.now()
		self.waypoints.header.frame_id = "map"
		self.waypoints.poses.append(pose_msg)
		self.path_pub.publish(self.waypoints)

	def save_csv(self):
		print("Saving waypoints...")
		df = pd.DataFrame(self.points_list, columns=['x', 'y', 'theta'])
		df.to_csv('waypoints_saver.csv')

if __name__ == '__main__':
	rospy.init_node('waypoints_saver', anonymous=True)
	ws_obj = waypointsSaver()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")







