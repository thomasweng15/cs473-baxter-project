#!/usr/bin/python

import time
import sys
import os
import argparse

import cv2

import rospy
import genpy

class Webcam():
	"""Provides an interface to open an image capture device
	to take both manual and automatic snapshots. 
	"""
	def __init__(self, img_dir, device=0):
		rospy.init_node("webcam")

		self.device = device 
		print "Opening capture device..."
		self.capture = cv2.VideoCapture(self.device)
		self.capture.set(3,960) # CV_CAP_PROP_FRAME_WIDTH  
		self.capture.set(4,720) # CV_CAP_PROP_FRAME_HEIGHT 
		self.img_dir = img_dir

		if not self.capture:
			print "Error opening capture device"
			sys.exit(1)

	def show_video_stream(self):
		"""Show a video feed from the webcam. 
		
		Quit when the 'q' key is pressed. 
		"""
		while True:
			val, frame = self.capture.read()
			cv2.imshow("input", frame)

			key = cv2.waitKey(50) % 256
			if key == ord('q'):
				break
			
	def take_manual_snapshot(self, filename, num=1, delay=0.5):
		"""Show a video feed from the webcam,
		taking a snapshot when SPACE is pressed. 
		
		params:
			filename 	base name with which to save snapshots. 
			num			max number of snapshots to take.
			delay		minimum # of seconds to wait between snapshots.

		Quit when the 'q' key is pressed. 
		"""
		while True:
			val, frame = self.capture.read()
			cv2.imshow("input", frame)

			key = cv2.waitKey(50) % 256
			if key == ord('q'):
				print "Taking snapshot cancelled."
				break
			elif key == ord(' '):
				print "Taking snapshot..."
				for __ in range(num):
					cur_name = filename
					if num > 1:
						file_split = os.path.splitext(filename)
						cur_name = file_split[0] + str(num) + file_split[1]	
					cv2.imwrite(os.path.join(self.img_dir, cur_name), frame)
					time.sleep(delay)
					print "Image saved."
				break

 	def take_automatic_snapshot(self, filename, time=3, delay=200):
 		"""Automatically take snapshots from the webcam. 
		
		params:
			filename 	base name with which to save snapshots. 
			time		time in seconds in which to take snapshots. 
			delay		interval in ms between snapshots. 
		"""
 		rate = rospy.Rate(delay)
 		start = rospy.Time.now()
 		elapsed = rospy.Time.now() - start
 		num = 0
 		while elapsed < genpy.rostime.Duration(time):
 			val, frame = self.capture.read()

 			cur_name = filename + str(num) + ".png"
 			num += 1
 			cv2.imwrite(os.path.join(self.img_dir, cur_name), frame)
 			print cur_name + " saved."	
 			rate.sleep()
 			elapsed = rospy.Time.now() - start

	def take_snapshot(self, filename):
		"""Take one snapshot from the webcam.

		params:
			filename 	base name with which to save snapshots. 
		"""
		val, frame = self.capture.read()
		cv2.imwrite(os.path.join(self.img_dir, "background.png"), frame)
		print "Image saved."
		
def main():
	"""
	"""
	arg_fmt = argparse.RawDescriptionHelpFormatter
	parser = argparse.ArgumentParser(formatter_class=arg_fmt,
									description=main.__doc__)
	parser.add_argument(
		'-d', '--dir', dest='directory', required=False,
		help="the directory to save to"
	)
	parser.add_argument(
		'-f', '--file', dest='filename', required=False,
		help="the base filename to save to"
	)
	args = parser.parse_args(rospy.myargv()[1:])

	args.directory = "." if args.directory is None
	args.filename = "TEST" if args.filename is None
	
	w = Webcam(args.directory)
	w.take_automatic_snapshot(args.filename)


if __name__ == '__main__':
	main()

