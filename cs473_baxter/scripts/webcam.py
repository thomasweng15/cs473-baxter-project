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
		self.capture.set(3,960)
		self.capture.set(4,720)
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

	def take_snapshots(self):
		print "Taking snapshot of view without object."
		val, frame = self.capture.read()
		cv2.imwrite(os.path.join(self.img_dir, "background.jpg"), frame)
		print "Image saved."
		print "Place object in center. Press SPACE when finished."

		while True:
			val, frame = self.capture.read()
			cv2.imshow("input", frame)

			key = cv2.waitKey(50) % 256
			if key == ord(' '):
				print "Taking snapshot of view with object."
				cv2.imwrite(os.path.join(self.img_dir, "foreground.jpg"), frame)
				print "Image saved."
				break

def main():
	img_dir = "./src/cs473-baxter-project/cs473_baxter/images/"
	w = Webcam(img_dir)
	w.show_video_stream()
	time.sleep(10)
	w.take_snapshots()


if __name__ == '__main__':
	main()