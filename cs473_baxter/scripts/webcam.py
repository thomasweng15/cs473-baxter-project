#!/usr/bin/python

import time
import sys
import os

import cv2

import rospy

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
			
	def take_snapshot(self, file_name, num=1, delay=0.5):
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
					print "Image saved. ",
				print
				break

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
		self.take_snapshot("arm.png")
		
	def take_uncompressed_snapshot(self):
		print "Place object alone in center. Press SPACE when finished."
		self.take_snapshot("uncompressed_object.png")
		
	def take_compressed_snapshot(self):
		# TODO take multiple images over time as compression occurs
		print "Prepare to press object with arm. Press SPACE when to start taking pictures."
		self.take_snapshot("compressed_object.png", num=10)

def main():
	img_dir = "./src/cs473-baxter-project/cs473_baxter/images/"
	w = Webcam(img_dir)
	w.take_snapshot("no_arm.png")
	w.take_snapshot("arm.png")


if __name__ == '__main__':
	main()

