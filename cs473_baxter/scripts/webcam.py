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
	def __init__(self, img_dir):
		self.img_dir = img_dir
		self.is_open = False
		self.capture = None

	def open(self, device=0):
		"""Intialize the capture device.

		params:
			device		the id of the device to open
		"""
		print "Opening capture device."
		self.capture = cv2.VideoCapture(device)
		self.capture.set(3,960) # CV_CAP_PROP_FRAME_WIDTH  
		self.capture.set(4,720) # CV_CAP_PROP_FRAME_HEIGHT 

		if not self.capture:
			print "Error opening capture device"
			sys.exit(1)
		else:
			self.is_open = True

	def close(self):
		"""Close the capture device."""
		print "Closing capture device."
		if self.is_open:
			self.capture.release()
			self.capture = None
			self.is_open = False
		else:
			print "Error: no capture device open, cannot close device."

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

 	def take_automatic_snapshot(self, filename, time=8, delay=200):
 		"""Automatically take snapshots from the webcam. 
		
		params:
			filename 	base name with which to save snapshots. 
			time		time in seconds in which to take snapshots. 
			delay		interval in ms between snapshots. 
		"""
 		rate = rospy.Rate(delay)
 		num = 0
 		start = rospy.Time.now()
 		elapsed = rospy.Time.now() - start

 		time_data = open(os.path.join(self.img_dir, "webcam_data.txt"), 'a+')
		time_data.write("webcam start: " + str(start.nsecs) + '\n')
			
 		while elapsed < genpy.rostime.Duration(time):
 			val, frame = self.capture.read()

 			cur_name = filename + ("%03d" % num) + ".png"
 			num += 1
 			cv2.imwrite(os.path.join(self.img_dir, cur_name), frame)
 			time_data.write(str(num) + ": " + str(rospy.Time.now().nsecs) + '\n')	
 			rate.sleep()
 			elapsed = rospy.Time.now() - start

 		time_data.close()

	def take_snapshot(self, filename):
		"""Take one snapshot from the webcam.

		params:
			filename 	base name with which to save snapshots. 
		"""
		val, frame = self.capture.read()
		cv2.imwrite(os.path.join(self.img_dir, filename), frame)
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
		'-t', '--time', dest='time', required=False,
		help="how long to take snapshots"
	)
	args = parser.parse_args(rospy.myargv()[1:])

	if args.directory is None:
		args.directory = '.'
	if args.time is None:
		args.time = 8
	else:
		args.time = int(args.time)

	rospy.init_node("webcam")

	w = Webcam(args.directory)
	w.open()
	w.take_automatic_snapshot("compression", time=args.time)
	w.close()

if __name__ == '__main__':
	main()

