#!/usr/bin/env python

import argparse

import rospy

import baxter_interface

class Glove():
	def __init__(self, gripper):
		rospy.init_node("cs473_gripper")

		self.gripper = baxter_interface.Gripper(gripper)

		# Verify robot is enabled
		print "Getting robot state..."
		self._rs = baxter_interface.RobotEnable()
		self._init_state = self._rs.state().enabled
		print "Enabling robot..."
		self._rs.enable()
		print "Running. Ctrl-c to quit"

	def grip_glove(self):
		print "Calibrating gripper..."
		self.gripper.calibrate()
		self.gripper.open()
		# set moving force
		# set holding force
		print "Calibration complete."
		prompt = raw_input("Press any key when glove is in position.")
		self.gripper.close()

	def release_glove(self):
		self.gripper.open()

	def clean_shutdown(self):
		print "\nExiting glove routine..."
		if not self._init_state and self._rs.state().enabled:
			print "Disabling robot..."
			self._rs.disable()


def main():
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									description=main.__doc__)
	parser.add_argument(
		'-g', '--grip', choices=['grip', 'release'], required=True,
		help="grip or release glove"
	)
	args = parser.parse_args(rospy.myargv()[1:])

	g = Glove('right')

	# register shutdown callback
	rospy.on_shutdown(g.clean_shutdown)

	if args.grip == 'grip':
		g.grip_glove()
	else:
		g.release_glove()

if __name__ == '__main__':
	main()
