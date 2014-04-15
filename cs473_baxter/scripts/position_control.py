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

ARM_POSITIONS = "./src/cs473-baxter-project/cs473_baxter/config/arm_positions"

class PositionControl():
	"""Provides an interface to Baxter's limbs and 
	joint control functions. 
	"""
	def __init__(self, limb):
		self._limb = baxter_interface.limb.Limb(limb) 
	
	def set_neutral(self):
		"""Move limb to neutral position."""
		self._limb.move_to_neutral()

	def move_to_jp(self, position, timeout=7, speed=0.3):
		"""Move limb to specified joint positions.

		params:
			position 	dict of joint position destinations
			timeout		seconds to wait for movement completion	
			speed 		speed at which to move to position. range: (0,1)
		"""
		if speed != 0.3:
			self._limb.set_joint_position_speed(speed)

		try:
			self._limb.move_to_joint_positions(position, timeout)
		except Exception:
			print "Warning: did not reach commanded joint position"

		self._limb.set_joint_position_speed(0.3)

	def get_jp_from_file(self, selector, filename=ARM_POSITIONS):
		"""Return a dict of joint positions from a file.

		params:
			selector 	section of the file to read in
			filename 	name of the file to read from 

		Return an empty dict if there was a read error.
		"""
		f = open(filename, 'r')
		
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
			print "Error: selector in '" + filename + "' was not found."
			return {}
		else:
			return jp


def main():
	"""
	"""
	rospy.init_node("cs473_basic_poke")

	pc = PositionControl('right')

	print "Moving to neutral pose..."
	pc.set_neutral()

	joint_position = pc.get_jp_from_file("RIGHT_ARM_INIT_POSITION")
	print "Moving to pose specified by joint positions..."
	pc.move_to_jp(joint_position)

if __name__ == '__main__':
	main() 
