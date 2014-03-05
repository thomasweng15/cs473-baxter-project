#!/usr/bin/env python

import rospy

import baxter_interface

from basic_poke import BasicMove

class BoxFit():
	def __init__(self):
		# NOTE the init of BM may cause issues...
		self._bm = BasicMove('right')

		# Verify robot is enabled
		print "Getting robot state..."
		self._rs = baxter_interface.RobotEnable()
		self._init_state = self._rs.state().enabled
		print "Enabling robot..."
		self._rs.enable()
		print "Running. Ctrl-c to quit"

	def compress_object(self):
		print "compress object"

	def clean_shutdown(self):
		print "\nExiting box fit routine..."
		if not self._init_state and self._rs.state().enabled:
			print "Disabling robot..."
			self._rs.disable()

def main():
	rospy.init_node("cs473_box_fit")
	
	bf = BoxFit()

	# register shutdown callback
	rospy.on_shutdown(bf.clean_shutdown)

	# Extract object from background

	# Calculate pixel dimensions of object

	# Compare pixel dimensions with that of box

	# Compress object

	# Measure compression to evaluate new pixel dimensions

	# Return final answer

if __name__ == '__main__':
	main()