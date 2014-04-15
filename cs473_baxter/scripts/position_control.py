#!/usr/bin/env python

import sys

import os.path

import rospy

import baxter_interface

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

ARM_POSITIONS = "./../config/arm_positions"

class PositionControl():
	def __init__(self, limb):
		self._limb = baxter_interface.limb.Limb(limb) 
	
	def set_neutral(self):
		self._limb.move_to_neutral()

	def move_to_jp(self, position, timeout=7, speed=0.3):
		if speed != 0.3:
			self._limb.set_joint_position_speed(speed)

		try:
			self._limb.move_to_joint_positions(position, timeout)
		except Exception:
			print "Warning: did not reach commanded joint position"

		self._limb.set_joint_position_speed(0.3)

	def get_jp_from_file(self, selector):
		file_path = os.path.dirname(__file__)
		if file_path != "":
			os.chdir(file_path)
		f = open(ARM_POSITIONS,"r")
		
		found_selector = False
		jp = {}
		for line in f:
			if line.strip('\n') == selector:
				found_selector = True
			elif found_selector and line == '\n':
				break
			elif found_selector: 
				l = line.split(', ')
				jp[l[0]] = float(l[1].strip('\n'))

		f.close()

		if found_selector == False:
			print "Error: selector in '" + ARM_POSITIONS + "' was not found."
			return {}
		else:
			return jp


def main():
	rospy.init_node("cs473_basic_poke")

	pc = PositionControl('right')

	print "Moving to neutral pose..."
	pc.set_neutral()

	joint_position = pc.get_jp_from_file("RIGHT_ARM_INIT_POSITION")
	print "Moving to pose specified by joint positions..."
	pc.move_to_jp(joint_position)

if __name__ == '__main__':
	main() 
