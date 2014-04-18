import numpy as np
import matplotlib.pyplot as plt
#from pylab import *
import sys
import csv

ROSTOPIC_START = 0
WEBCAM_START = 0




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

def parseCSV(csv_filename, webcam_filename):
	csvfile = open(csv_filename, 'r')
	webcamfile = open(webcam_filename, 'r')
	csvfile.next()
	csvreader = csv.reader(csvfile, delimiter=',', quotechar='|')
	print csvreader
	timestamps = []
	objects = []
	px_widths = []
	px_heights = []
	mm_widths = []
	mm_heights = []
	#forces = []
	#spring_constants = []
	for line in webcamfile.readlines():
		temp = line.split(':')
		t = temp[1]
		t = t[1:]
		t = int(t)
		timestamps.append(t)

		#timestamp.append()
	print timestamps
	for row in csvreader:
		objects.append(row[0])
		px_widths.append(float(row[1]))
		px_heights.append(float(row[2]))
		mm_widths.append(float(row[3]))
		mm_heights.append(float(row[4]))
		#forces.append(float(row[5]))
		#spring_constants.append(float(row[6]))
		#print row

	#plt.scatter(mm_heights, forces)
	#plt.show()

def parseTimingFile(filename):
	f = open(filename, "r")
	for line in f:
		if 'webcam' in line:
			WEBCAM_START = int(line[8:])
		elif 'rostopic' in line:
			ROSTOPIC_START = int(line[10:])

	assert WEBCAM_START != 0, "webcam timing data not found."
	assert ROSTOPIC_START != 0, "rostopic timing data not found."
	print WEBCAM_START
	print ROSTOPIC_START




def main():
	rostopic_filename = sys.argv[1]
	csv_filename = sys.argv[2]
	webcam_data_filename = sys.argv[3]
	timing_filename = sys.argv[4]

	parseTimingFile(timing_filename)
	parseCSV(csv_filename, webcam_data_filename)

if __name__ == "__main__":
	main()