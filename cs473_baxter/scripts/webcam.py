#!/usr/bin/python

import time
import sys
import os
import argparse

import cv2

import rospy
import genpy

class Webcam():
	def __init__(self, img_dir):
		self.device = 0 # assume we want first device
		print "Opening capture device..."
		self.capture = cv2.VideoCapture(self.device)
		self.capture.set(3,960) # CV_CAP_PROP_FRAME_WIDTH  
		self.capture.set(4,720) # CV_CAP_PROP_FRAME_HEIGHT 
		self.img_dir = img_dir

		if not self.capture:
			print "Error opening capture device"
			sys.exit(1)

	def show_video_stream(self):
		while True:
			val, frame = self.capture.read()
			cv2.imshow("input", frame)

			key = cv2.waitKey(50) % 256
			if key == ord('q'):
				break
			
	def take_manual_snapshot(self, file_name, num=1, delay=0.5):
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
					cur_name = file_name
					if num > 1:
						file_split = os.path.splitext(file_name)
						cur_name = file_split[0] + str(num) + file_split[1]	
					cv2.imwrite(os.path.join(self.img_dir, cur_name), frame)
					time.sleep(delay)
					print "Image saved."
				break

 	def take_automatic_snapshot(self, filename, time=3, delay=200):
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

	def take_reference_snapshot(self):
		print "Taking snapshot of background."
		val, frame = self.capture.read()
		cv2.imwrite(os.path.join(self.img_dir, "background.png"), frame)
		print "Image saved."
		
		# Use hard-coded box dimensions for now
		#print "Place box in center. Press SPACE when finished."
		#self.take_snapshot("box.png")
		
		# TODO automate the moving of the arm
		print "Remove box and move arm into view. Press SPACE when finished."
		self.take_manual_snapshot("arm.png")
		
	def take_uncompressed_snapshot(self):
		print "Place object alone in center. Press SPACE when finished."
		self.take_manual_snapshot("uncompressed_object.png")
		
def main():
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

	if args.directory is None:
		args.directory = "."
	if args.filename is None:
		args.filename = "TEST"

	rospy.init_node("webcam")
	
	w = Webcam(args.directory)
	w.take_automatic_snapshot(args.filename)


if __name__ == '__main__':
	main()

