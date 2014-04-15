#!/usr/bin/env python

import sys
import ConfigParser

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

CONFIG = "./src/cs473-baxter-project/cs473_baxter/config/config"

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

	def get_jp_from_file(self, section, filename=CONFIG):
		"""Return a dict of joint positions from a file.

		params:
			section 	header to read in from file
			filename 	name and path of the file to read from 

		Return an empty dict if there was a read error.
		"""
		Config = ConfigParser.ConfigParser()
		Config.read(filename)
		positions = Config.items(section)

		jp ={}
		for pair in positions:
			jp[pair[0]] = float(pair[1])

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
