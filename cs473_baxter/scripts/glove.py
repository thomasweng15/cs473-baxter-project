#!/usr/bin/env python

import argparse

import rospy

import baxter_interface



class Glove():
	"""Provides an interface for attaching and releasing 
	Baxter's pusher glove.
	"""
	def __init__(self, gripper):
		rospy.init_node("cs473_gripper")

		self.gripper = baxter_interface.aGripper(gripper)

		# Verify robot is enabled
		print "Getting robot state..."
		self._rs = baxter_interface.RobotEnable()
		self._init_state = self._rs.state().enabled
		print "Enabling robot..."
		self._rs.enable()
		print "Running. Ctrl-c to quit"

	def grip_glove(self):
		"""Command Baxter to grip an item.

		Press ENTER to execute the command. 
		"""
		print "Calibrating gripper..."
		self.gripper.calibrate()
		self.gripper.open()
		print "Calibration complete."
		prompt = raw_input("Press ENTER when glove is in position.")
		self.gripper.close()

	def release_glove(self):
		"""Command Baxter to release an item."""
		self.gripper.open()
		print "If Baxter does not release, you may need to run this script again."
		print "If you see an error, you will have to manually remove the glove."


def main():
	"""
	"""
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									description=main.__doc__)
	parser.add_argument(
		'-g', '--grip', choices=['grip', 'release'], required=True,
		help="grip or release glove"
	)
	args = parser.parse_args(rospy.myargv()[1:])

	g = Glove('right')

	g.grip_glove() if args.grip == 'grip' else g.release_glove()

if __name__ == '__main__':
	main()
