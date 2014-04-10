#!/usr/bin/env python
import os
import time

import sys

import rospy

import baxter_interface

from basic_poke import BasicMove
from webcam import Webcam
from joint_velocity import JointVelocity
from cs473vision.cs473vision.obj_baxter import BaxterObject

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

		self.bm = BasicMove('right')
		self.jv = JointVelocity()

		self.img_dir = self._create_img_dir(img_dir)
		self._camera = Webcam(self.img_dir)

	def _create_img_dir(self, img_dir):
		dirname = ''.join([img_dir, time.strftime("%d%m%Y_%H-%M-%S")])
		os.mkdir(dirname)
		return dirname

	def is_glove_attached(self):
		# Verify glove is attached
		glove_on = raw_input("Is Baxter's glove attached? (y/n): ")
		if glove_on is not "y":
			print "\nERROR: Run glove.py to attach the glove before running BoxFit."
			sys.exit(1)

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
	bf.is_glove_attached()
	
	bf.bm.set_neutral()
	bf.bm.move_to_jp(bf.bm.get_jp_from_file())
	#bf.jv.move()

	"""
	bg_path = os.path.join(bf.img_dir, "background.png")
	box_path = None
	#box_path = os.path.join(IMG_DIR, "box.png")
	arm_path = os.path.join(bf.img_dir, "arm.png")
	obj_path = os.path.join(bf.img_dir, "uncompressed_object.png")
	compress_path = os.path.join(bf.img_dir, "compressed_object.png")

	# Take background images
	bf._camera.take_reference_snapshot()

	# Extract object from background
	bf._camera.take_uncompressed_snapshot()
	baxter_obj = BaxterObject(bg_path, box_path, obj_path)

	# Calculate pixel dimensions of object
	u_w, u_h = baxter_obj.get_uncompressed_size()
	
	# Compare pixel dimensions with that of box
	fits = baxter_obj.check_uncompressed_fit()

	# Compress object
	bf.bm.move_to_jp(bf.bm.get_jp_from_file())
	bf._camera.take_compressed_snapshot()
	baxter_obj.set_arm_image(arm_path)
	baxter_obj.set_compressed_image(compress_path)

	# Measure compression to evaluate new pixel dimensions
	c_w, c_h = baxter_obj.get_compressed_size()
	fits = baxter_obj.check_compressed_fit()

	# Return final answer
	print fits
	"""

if __name__ == '__main__':
	main()
