import numpy as np
import matplotlib.pyplot as plt
#from pylab import *
import sys


def parseFile(file):
	xarr = []
	yarr = []
	f = open(file, "r")
	str_to_find = ""
	lines = []
	for line in f:
		lines.append(line)

	for i in range(len(lines)):
		if "position" in lines[i]:
			xline = lines[i + 2]
			x = float(xline[6:])
			xarr.append(x)

		if "wrench" in lines[i]:
			yline = lines[i + 2]
			y = float(yline[6:])
			yarr.append(y)
	
	plt.scatter(xarr, yarr)
	plt.show()


arr = parseFile(sys.argv[1])