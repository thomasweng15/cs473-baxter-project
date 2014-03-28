#!/usr/bin/env python

import rospy

import baxter_interface

from basic_poke import BasicMove

class BoxFit():
	def __init__(self):
		# Verify robot is enabled
		print "Getting robot state..."
		self._rs = baxter_interface.RobotEnable()
		self._init_state = self._rs.state().enabled
		print "Enabling robot..."
		self._rs.enable()
		print "Running. Ctrl-c to quit"
		print "\nIf Baxter is not gripping the gripper shield,"
		print "please terminate this instance and attach the shield by running glove.py."

		self.bm = BasicMove('right')

	def compress_object(self):
		print "compress object"

	def clean_shutdown(self):
		print "\nExiting box fit routine..."
		if not self._init_state and self._rs.state().enabled:
			print "Disabling robot..."
			self._rs.disable()

def main():
	rospy.init_node("cs473_box_fit")

	# Initializations
	bf = BoxFit()
	bf.bm.move_to_jp(bf.bm.get_jp_from_file())
	
	## register shutdown callback
	rospy.on_shutdown(bf.clean_shutdown)

	# Extract object from background

	# Calculate pixel dimensions of object

	# Compare pixel dimensions with that of box

	# Compress object

	# Measure compression to evaluate new pixel dimensions

	# Return final answer

if __name__ == '__main__':
	main()