#!/usr/bin/env python
import os

import rospy

import baxter_interface

from basic_poke import BasicMove
from webcam import Webcam
from cs473vision.cs473vision.obj_detect import SegmentedObject
from cs473vision.cs473vision.obj_detect import check_fit

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
		self._camera.take_snapshots()

		bg_path = os.path.join(IMG_DIR, "background.jpg")
		fg_path = os.path.join(IMG_DIR, "foreground.jpg")
		try:
			obj = SegmentedObject(bg_path, fg_path)
		except IOError:
			print "Error loading images!"
		return obj

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
	obj = bf.extract_object_from_bg()
	box = bf.extract_object_from_bg()

	# Calculate pixel dimensions of object
	__, __, obj_width, obj_height = obj.get_object_rectangle()
	__, __, box_width, box_height = box.get_object_rectangle()
	
	# Compare pixel dimensions with that of box
	obj_fits = check_fit(obj_width, obj_height, box_width, box_height)

	# Compress object
	bf.bm.move_to_jp(bf.bm.get_jp_from_file())

	# Measure compression to evaluate new pixel dimensions

	# Return final answer


if __name__ == '__main__':
	main()
