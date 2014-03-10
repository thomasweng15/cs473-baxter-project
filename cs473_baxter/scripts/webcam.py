#!/usr/bin/python

import sys

import cv2

import rospy

class Webcam():
	def __init__(self):
		self.device = 0 # assume we want first device
		self.capture = cv2.VideoCapture(self.device)
		self.capture.set(3,320)
		self.capture.set(4,240)

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

	def take_snapshots(self, delay=2):
		print "press SPACE to take snapshots, press 'q' to quit."
		take_picture = False;
		t0, filenum = 0, 1

		while True:
			val, frame = self.capture.read()
			cv2.imshow("video", frame)

			key = cv2.waitKey(50) % 256
			if key == ord(' '):
				t0 = cv2.getTickCount()
				take_picture = True
			elif key == ord('q'):
				break

			if take_picture and ((cv2.getTickCount()-t0) / cv2.getTickFrequency()) > delay:
				cv2.imwrite(str(filenum) + ".jpg", frame)
				print "Image saved."
				filenum += 1
				take_picture = False

			self.capture.release

def main():
	w = Webcam()

	#w.show_video_stream()
	w.take_snapshots()

if __name__ == '__main__':
	main()