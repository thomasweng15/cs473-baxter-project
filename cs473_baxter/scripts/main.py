#!/usr/bin/env python
import os
import time
import subprocess
import sys
import ConfigParser

import rospy

import baxter_interface

from position_control import PositionControl
from webcam import Webcam
from cs473vision.cs473vision.obj_baxter import BaxterObject

CONFIG = './src/cs473-baxter-project/cs473_baxter/config/config'

class BoxFit():
	"""The primary module for running compression trials.
	Links the webcam, vision segmentation, and actuation
	modules together.
	"""
	def __init__(self):
		rospy.init_node("cs473_box_fit")

		self.is_glove_attached()

		# Verify robot is enabled
		print "Getting robot state..."
		self._rs = baxter_interface.RobotEnable()
		self._init_state = self._rs.state().enabled
		print "Enabling robot..."
		self._rs.enable()
		print "Running. Ctrl-c to quit"

		self.ps = PositionControl('right')

		self.img_dir = self._create_img_dir()
		self._camera = Webcam(self.img_dir)

	def _create_img_dir(self):
		"""Creates a timestamped folder in the img_dir directory
		that stores the images of one compression run.
		"""
		Config = ConfigParser.ConfigParser()
		Config.read(CONFIG)
		base_img_dir = Config.get("IMAGE_DIRECTORY", "base_img_dir")
		img_dir = ''.join([base_img_dir, time.strftime("%d%m%Y_%H-%M-%S")])
		os.mkdir(img_dir)
		return img_dir

	def is_glove_attached(self):
		"""Prompt the user to check if Baxter's pusher glove
		is attached or not. Exit the process if it is not. 
		"""
		glove_on = raw_input("Is Baxter's glove attached? (y/n): ")
		if glove_on is not "y":
			print "\nERROR: Run glove.py to attach the glove before running BoxFit."
			sys.exit(1)

	def set_init_joint_positions(self):
		"""Move arm(s) to initial joint positions."""
		self.ps.set_neutral()
		self.ps.move_to_jp(self.ps.get_jp_from_file('RIGHT_ARM_INIT_POSITION'))
		
	def compress_object(self, filename="compression"):
		"""Compress an object while opening the webcam to take 
		snapshots during the compression. 
		"""
		self._camera.capture.release()

		time_data = open(os.path.join(self.img_dir, 'timestamps.txt'), 'a+')
		r_data = open(os.path.join(self.img_dir, 'rostopic_data.txt'), 'a+')
		
		time_data.write("webcam: " + str(rospy.Time.now().nsecs) + '\n')
		w_proc = subprocess.Popen(['rosrun', 'cs473_baxter', 'webcam.py', 
			"-d", self.img_dir])
		
		time.sleep(2) # Buffer time for webcam subprocess to get ready

		time_data.write("rostopic: " + str(rospy.Time.now().nsecs) + '\n')
		r_proc = subprocess.Popen(['rostopic', 'echo', 
			'/robot/limb/right/endpoint_state'], 
			stdout=r_data) 

		time_data.write("compress: " + str(rospy.Time.now().nsecs) + '\n')
   		self.ps.move_to_jp(
   			self.ps.get_jp_from_file('RIGHT_ARM_COMPRESS_POSITION'),
   			timeout=5, speed=0.05)
   		
		self.ps.move_to_jp(self.ps.get_jp_from_file('RIGHT_ARM_INIT_POSITION'))

   		r_proc.terminate()
   		w_proc.terminate()
   		time_data.close()

	def clean_shutdown(self):
		print "\nExiting box fit routine..."
		if not self._init_state and self._rs.state().enabled:
			print "Disabling robot..."
			self._rs.disable()

def main():
	"""
	"""
	bf = BoxFit()
	rospy.on_shutdown(bf.clean_shutdown)
	
	bf.set_init_joint_positions()
	bf.compress_object()

	
	"""bg_path = os.path.join(bf.img_dir, "background.png")
	box_path = None
	#box_path = os.path.join(IMG_DIR, "box.png")
	arm_path = os.path.join(bf.img_dir, "arm.png")
	obj_path = os.path.join(bf.img_dir, "uncompressed_object.png")
	compress_path = os.path.join(bf.img_dir, "compressed_object.png")

	# Take background images
	print "Taking snapshot of background."
	bf._camera.take_reference_snapshot()

	# Extract object from background
	print "Place object alone in center. Press SPACE when finished."
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
	print fits"""
	

if __name__ == '__main__':
	main()
