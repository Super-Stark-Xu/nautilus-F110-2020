#!/usr/bin/env python

import rospy
import math
import numpy as np
import yaml
import sys
import csv

from nautilus_wall_following.msg import gap
from nautilus_wall_following.msg import gaps
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import rospkg

RIGHT_ANGLE = -30
LEFT_ANGLE = 30

class junctionDetection:
	def __init__(self):
		self.gaps_sub = rospy.Subscriber("lidar_gaps", gaps, self.gaps_callback)
		self.csv_instructions = self.read_csv()

		print(self.csv_instructions)
		self.junction_type = None

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
		return csv_instructions

	def find_jn_type(self, flag_L, flag_C, flag_R):
		if flag_L == True and flag_R == True and flag_C == True:
			jn_type = 'cross'
		elif flag_L == True and flag_R == True:
			jn_type = 'T'
		else:
			jn_type = 'Nil'
		return jn_type

	def monitor_turn(self, direction, vel):
		## TODO
		pass

	def gaps_callback(self, gap_array):
		flag_L = False
		flag_R = False
		flag_C = False
		
		for i in range(len(gap_array)):
			cur_gap = gap_array.gapArray[i]

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
				(direction, vel) = self.instructions.pop(0)
				rospy.set_param('turn_flag', 'ACTIVE')
				rospy.set_param('direction', direction)
				rospy.set_param('velocity', vel)

		self.monitor_turn(direction, vel)

if __name__ == '__main__':
	jd_obj = junctionDetection()
	rospy.init_node('junction_node', anonymous=True)
	try:
		rospy.spin()
	except KeyboardInterrupt:
		print("Shutting down")

	