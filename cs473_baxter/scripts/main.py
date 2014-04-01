#!/usr/bin/env python
import os

import rospy

import baxter_interface

from basic_poke import BasicMove
from webcam import Webcam
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
		print "\nIf Baxter is not gripping the gripper shield,"
		print "please terminate this instance and attach the shield by running glove.py."

		self.bm = BasicMove('right')

		self._camera = Webcam(img_dir)

	def extract_object_from_bg(self):
		self._camera.take_snapshots()

		bg_path = os.path.join(IMG_DIR, "background.png")
		fg_path = os.path.join(IMG_DIR, "foreground.png")
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
	
	bg_path = os.path.join(IMG_DIR, "background.png")
	box_path = None
	#box_path = os.path.join(IMG_DIR, "box.png")
	arm_path = os.path.join(IMG_DIR, "arm.png")
	obj_path = os.path.join(IMG_DIR, "uncompressed_object.png")
	compress_path = os.path.join(IMG_DIR, "compressed_object.png")

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

if __name__ == '__main__':
	main()
