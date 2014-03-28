#!/usr/bin/python

import sys
import os

import cv2

import rospy

IMAGES_DIR = "./src/cs473-baxter-project/cs473_baxter/images/"

class Webcam():
	def __init__(self):
		self.device = 0 # assume we want first device
		self.capture = cv2.VideoCapture(self.device)
		self.capture.set(3,1200)
		self.capture.set(4,1600)

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

	def get_background(self):
		print "Getting background for object detection through bg subtraction."
		val, frame = self.capture.read()
		cv2.imwrite(os.path.join(IMAGES_DIR, "background.jpg"), frame)
		print "Background saved."
		self.capture.release

def main():
	w = Webcam()
	w.get_background()

	#w.show_video_stream()

if __name__ == '__main__':
	main()