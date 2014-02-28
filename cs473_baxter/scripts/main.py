#!/usr/bin/env python

import rospy

from basic_poke import BasicMove

class BoxFit():
	def compress_object(self):
		print "compress object"

def main():
	rospy.init_node("cs473_box_fit")
	bm = BasicMove('right')
	print "Hello world!"
	# Extract object from background

	# Calculate pixel dimensions of object

	# Compare pixel dimensions with that of box

	# Compress object

	# Measure compression to evaluate new pixel dimensions

	# Return final answer

if __name__ == '__main__':
	main()