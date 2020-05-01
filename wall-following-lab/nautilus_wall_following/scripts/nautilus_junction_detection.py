#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
import csv

from nautilus_wall_following.msg import gap
from nautilus_wall_following.msg import gaps
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import rospkg
import tf
import math

RIGHT_ANGLE = -20
LEFT_ANGLE = 20

TURN_RADIUS = 80
DIST_THRESH = 6.00

MIN_WIDTH = 2.0
MAX_WIDTH = 8.0

class junctionDetection:
	def __init__(self):
		self.csv_instructions = self.read_csv()
		self.tf_listener = tf.TransformListener()

		self.direction_default = rospy.get_param('direction')
		self.velocity_default = rospy.get_param('velocity')
		self.yaw_init = None
		self.trans_init = None
		self.current_behavior = None

		self.ackermann_pub = rospy.Publisher('/nav', AckermannDriveStamped, queue_size=5)
		self.marker_pub = rospy.Publisher('/junction_visualization', Marker, queue_size=1)

		# All subscriber calls after this point
		self.gaps_sub = rospy.Subscriber("lidar_gaps", gaps, self.gaps_callback)

	def read_csv(self):
		csv_instructions = []
		rp = rospkg.RosPack()
		pkg_path = rp.get_path('nautilus_wall_following')
		file_path = pkg_path + '/explicit_instructions/instructions.csv'
		with open(file_path) as csv_file:
			csv_reader = csv.reader(csv_file, delimiter=' ')
			for row in csv_reader:
				row[0] = row[0].lower()
				row[1] = float(row[1])
				csv_instructions.append(tuple(row))
		print(csv_instructions)
		return csv_instructions

	def publish_marker(self, turn_flag):
		marker = Marker()
		marker.header.frame_id = "/laser"
		marker.pose.position.x = 1.5
		marker.pose.position.y = 0.0
		marker.pose.position.z = 0.0

		marker.type = marker.CUBE

		if turn_flag == 'ACTIVE':
			marker.color.r = 1.0
			marker.color.g = 0.0
			marker.color.b = 0.0

		elif turn_flag == 'INACTIVE':
			marker.color.r = 0.0
			marker.color.g = 1.0
			marker.color.b = 0.0

		marker.scale.x = 1.0 # If marker is too small in Rviz can make it bigger here
		marker.scale.y = 1.0
		marker.scale.z = 1.0
		marker.color.a = 1.0
		self.marker_pub.publish(marker)

	def find_jn_type(self, flag_L, flag_C, flag_R):
		if flag_L == True and flag_R == True and flag_C == True:
			jn_type = 'cross'

		elif flag_L == True and flag_R == True:
			jn_type = 'T'

		elif flag_L == True and flag_C == True:
			jn_type = 'T'
		
		elif flag_C == True and flag_R == True:
			jn_type = 'T'

		else:
			jn_type = 'Nil'
		return jn_type

	def get_yaw_angle(self):
		(trans, quat) = self.tf_listener.lookupTransform('/base_link', '/map', rospy.Time(0))
		euler = tf.transformations.euler_from_quaternion(quat)
		yaw = math.degrees(euler[2])
		return yaw, trans

	def get_distance_mag(self, trans_i, trans_j):
		(x1, y1, z1) = trans_i
		(x2, y2, z2) = trans_j
		dist = math.hypot(x2-x1, y2-y1)
		return dist

	def monitor_turn(self, jn_type):
		direction = rospy.get_param('direction')
		yaw_cur, trans_cur = self.get_yaw_angle()
		yaw_diff = abs(yaw_cur - self.yaw_init)
		dist_mag = self.get_distance_mag(trans_cur, self.trans_init)
	
		if direction == 'left' or direction == 'right':
			if  yaw_diff >= TURN_RADIUS and dist_mag >= DIST_THRESH: #jn_type == 'Nil':

				print("\n\n***********************************")
				print("****** Completed ********")
				print(self.current_behavior)
				print("***********************************")
				rospy.set_param('turn_flag', 'INACTIVE')
				self.publish_marker('INACTIVE')
					
		if direction == 'center':
			if jn_type == 'Nil' and dist_mag >= DIST_THRESH:

				print("\n\n***********************************")
				print("****** Completed ********")
				print(self.current_behavior)
				print("***********************************")
				rospy.set_param('turn_flag', 'INACTIVE')
				self.publish_marker('INACTIVE')

		if direction == 'stop':
			self.direction_default = 'stop'
			self.velocity_default = 0.0
			rospy.set_param('direction', 'stop')
			rospy.set_param('velocity', 0.00)
			rospy.set_param('turn_flag', 'ACTIVE')

			# infinite while loop may be 
			msg = AckermannDriveStamped()
			msg.header.stamp = rospy.Time.now()
			msg.header.frame_id = "base_link"
			msg.drive.speed = 0.0
			msg.drive.acceleration = 0.0
			msg.drive.jerk = 0.0
			msg.drive.steering_angle = 0.0
			msg.drive.steering_angle_velocity = 0.0
			self.ackermann_pub.publish(msg)

	def gaps_callback(self, gap_array):
		flag_L = False
		flag_R = False
		flag_C = False
		
		for i in range(len(gap_array.gapArray)):
			cur_gap = gap_array.gapArray[i]

			if cur_gap.width >= MIN_WIDTH and cur_gap.width <= MAX_WIDTH:

				if cur_gap.angle <= RIGHT_ANGLE:
					flag_R = True

				elif cur_gap.angle > RIGHT_ANGLE and cur_gap.angle < LEFT_ANGLE:
					flag_C = True

				elif cur_gap.angle >= LEFT_ANGLE:
					flag_L = True

				else:
					print("Invalid Angle!")

		jn_type = self.find_jn_type(flag_L, flag_C, flag_R)
		
		if rospy.get_param('turn_flag') == 'INACTIVE':
			if jn_type == 'cross' or jn_type == 'T':
				self.yaw_init,  self.trans_init = self.get_yaw_angle()
				(direction, vel) = self.csv_instructions.pop(0)
				self.velocity_default = vel

				rospy.set_param('turn_flag', 'ACTIVE')
				rospy.set_param('direction', direction)
				rospy.set_param('velocity', vel)
			
				self.publish_marker('ACTIVE')
				self.current_behavior = (direction, vel)

				print("\n\n***********************************")
				print("******BEHAVIOR INTIATED********")
				print(self.current_behavior)
				print("***********************************\n\n")

			elif jn_type == 'Nil':
				rospy.set_param('direction', self.direction_default)
				rospy.set_param('velocity', self.velocity_default)

		if rospy.get_param('turn_flag') == 'ACTIVE':
			self.monitor_turn(jn_type)

if __name__ == '__main__':
	rospy.init_node('junction_node', anonymous=True)
	jd_obj = junctionDetection()
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

	