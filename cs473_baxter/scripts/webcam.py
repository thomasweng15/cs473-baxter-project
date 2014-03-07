#!/usr/bin/python

import sys

import cv2

import rospy

class Webcam():
	def __init__(self):
		self.device = 0 # assume we want first device
		self.capture = cv2.VideoCapture(self.device)
		self.capture.set(3,1280)
		self.capture.set(4,1024)

		# check if capture device is OK
		if not self.capture:
			print "Error opening capture device"
			sys.exit(1)

	def capture_frames(self):
		while 1:
			ret, img = self.capture.read()
			cv2.imshow("input", img)

			key = cv2.waitKey(10)

def main():
	w = Webcam()

	w.capture_frames()

if __name__ == '__main__':
	main()