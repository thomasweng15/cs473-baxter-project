#!/usr/bin/python

import sys
import os

import cv2

import rospy

class Webcam():
	def __init__(self, img_dir):
		self.device = 0 # assume we want first device
		print "Opening capture device..."
		self.capture = cv2.VideoCapture(self.device)
		self.capture.set(3,1200)
		self.capture.set(4,1600)
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

	def take_snapshot(self, filename):
		val, frame = self.capture.read()
		full = [filename, '.jpg']
		cv2.imwrite(os.path.join(self.img_dir, ''.join(full)), frame)
		print "Image saved."

	def release(self):
		self.capture.release

def main():
	img_dir = "./src/cs473-baxter-project/cs473_baxter/images/"
	w = Webcam(img_dir)
	w.take_snapshot("back")
	w.take_snapshot("fore")
	w.release()


if __name__ == '__main__':
	main()