#!/usr/bin/env python
import time

import rospy

import baxter_interface

from basic_poke import BasicMove
from webcam import Webcam

IMG_DIR = "./src/cs473-baxter-project/cs473_baxter/images/"

class BoxFit():
	def __init__(self, img_dir):
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

		self._camera = Webcam(img_dir)

	def extract_object_from_bg(self):
		print "Taking snapshot of view without object."
		self._camera.take_snapshot("background")
		wait = raw_input("Place object in center. Press ENTER when finished.")
		print "Taking snapshot of view with object."
		self._camera.take_snapshot("foreground")

		# Do diff using cs473vision

		self._camera.release()

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
	bf = BoxFit(IMG_DIR)
	rospy.on_shutdown(bf.clean_shutdown)

	# Extract object from background
	bf.extract_object_from_bg()

	# Calculate pixel dimensions of object

	# Compare pixel dimensions with that of box

	# Compress object
	bf.bm.move_to_jp(bf.bm.get_jp_from_file())

	# Measure compression to evaluate new pixel dimensions

	# Return final answer


if __name__ == '__main__':
	main()