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
from cs473vision.cs473vision.view_baxter import BaxterObjectView

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

	def is_glove_attached(self):
		"""Prompt the user to check if Baxter's pusher glove
		is attached or not. Exit the process if it is not. 
		"""
		glove_on = raw_input("Is Baxter's glove attached? (y/n): ")
		if glove_on is not "y":
			print "\nERROR: Run glove.py to attach the glove before running BoxFit."
			sys.exit(1)

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
	
	def set_neutral(self):
		"""Move arm(s) to initial joint positions."""
		self.ps.set_neutral()
		
	def compress_object(self, filename="compression"):
		"""Compress an object while opening the webcam to take 
		snapshots during the compression. 
		"""
		self.ps.move_to_jp(self.ps.get_jp_from_file('RIGHT_ARM_INIT_POSITION'))

		# Suppress collision detection and contact safety 
		contact_safety_proc = subprocess.Popen(['rostopic', 'pub', 
			'-r', '10', 
			'/robot/limb/right/suppress_contact_safety', 
			'std_msgs/Empty'])

		time_data = open(os.path.join(self.img_dir, 'timestamps.txt'), 'a+')
		r_data = open(os.path.join(self.img_dir, 'rostopic_data.txt'), 'a+')
		
		time_data.write('webcam: ' + str(rospy.Time.now().nsecs) + '\n')
		w_proc = subprocess.Popen(['rosrun', 'cs473_baxter', 'webcam.py', 
			"-d", self.img_dir, 
			"-t", "12"])
		
		time.sleep(2) # Buffer time for webcam subprocess to get ready

		time_data.write('rostopic: ' + str(rospy.Time.now().nsecs) + '\n')
		r_proc = subprocess.Popen(['rostopic', 'echo', 
			'/robot/limb/right/endpoint_state'], 
			stdout=r_data) 

		time_data.write('compress: ' + str(rospy.Time.now().nsecs) + '\n')
   		self.ps.move_to_jp(
   			self.ps.get_jp_from_file('RIGHT_ARM_COMPRESS_POSITION'),
   			timeout=10, speed=0.05)

   		time.sleep(1.5)
   		
		self.ps.move_to_jp(self.ps.get_jp_from_file('RIGHT_ARM_INIT_POSITION'))

		contact_safety_proc.terminate()
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
	BF = BoxFit()
	WC = Webcam(BF.img_dir)
	rospy.on_shutdown(BF.clean_shutdown)
	BF.set_neutral()

	print 'Taking snapshot of the background.'
	WC.open()
	time.sleep(2)
	WC.take_snapshot('background.png')
	WC.close()

	user_input = raw_input("Place reference object alone in center. Press ENTER when finished.")
	WC.open()
	time.sleep(2)
	WC.take_snapshot('reference.png')
	WC.close()
	user_input = raw_input("Remove the reference object. Press ENTER when finished.")

	print 'Taking snapshot of just the arm'
	BF.ps.move_to_jp(BF.ps.get_jp_from_file('RIGHT_ARM_INIT_POSITION'))
	WC.open()
	WC.take_snapshot('arm.png')
	WC.close()
	BF.set_neutral()

	user_input = raw_input("Place object alone in center. Press ENTER when finished.")
	WC.open()
	WC.take_snapshot('object.png')
	WC.close()

	BF.compress_object(WC)

	# Do image stuff

	base = BF.img_dir + "/"
	bg_path = base + '/background.png'
	arm_path = base + '/arm.png'
	uncompressed_path =  base + '/object.png'

	baxter_obj = BaxterObjectView(bg_path)
	baxter_obj.set_arm_image(arm_path)
	#baxter_obj.set_arm_color((h, s, v), (h, s, v))
	baxter_obj.set_uncompressed_image(uncompressed_path)

	print "Uncompressed size: " + str(baxter_obj.get_uncompressed_size())

	for i in range(999):
		path = base + "/compression" + ('%03d' % i) + ".png"
		if os.path.isfile(path):
			baxter_obj.set_compressed_image(path, force=-1)
		else:
			break

	print "Compressed size: " + str(baxter_obj.get_compressed_size())

	baxter_obj.export_sizes("./sizes.txt")
	
	#baxter_obj.display_results()

if __name__ == '__main__':
	main()
