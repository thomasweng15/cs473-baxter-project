import numpy as np
import matplotlib.pyplot as plt
#from pylab import *
import sys
import csv

class Plotting(object):
	def __init__(self):


	def parseRostopic(rostopic_filename):
		timestamps = []
		positions = []
		wrenches = []
		f = open(rostopic_filename, "r")
		lines = []
		time_of_capture = ROSTOPIC_START + 30000000
		for line in f:
			lines.append(line)
			
		for i in range(len(lines)):
			if "position" in lines[i]:
				xline = lines[i + 2]
				x = float(xline[6:])
				positions.append(x)
				#add timestamp
				timestamps.append(time_of_capture)
				#increase by 10ms = 10000000ns
				time_of_capture += 10000000

			if "wrench" in lines[i]:
				yline = lines[i + 2]
				y = float(yline[6:])
				wrenches.append(y)

		dict = {'timestamps': timestamps,
				'positions': positions,
				'wrenches': wrenches}
		return dict
		

	def parseCSV(csv_filename, webcam_filename):
		csvfile = open(csv_filename, 'r')
		webcamfile = open(webcam_filename, 'r')
		csvfile.next()
		csvreader = csv.reader(csvfile, delimiter=',', quotechar='|')
		timestamps = []
		objects = []
		px_widths = []
		px_heights = []
		px_width_changes = []
		px_height_changes = []
		mm_widths = []
		mm_heights = []
		mm_width_changes = []
		mm_height_changes = []

		webcamlines =  []
		for line in webcamfile.readlines():
			webcamlines.append(line)

		for line in webcamlines[1:]:
			temp = line.split(':')
			t = temp[1]
			t = int(t)
			timestamps.append(t)
		
		csvrows = []
		for row in csvreader:
			csvrows.append(row)

		for row in csvrows[3:]:
			objects.append(row[0])
			px_widths.append(float(row[1]))
			px_heights.append(float(row[2]))
			px_width_changes.append(float(row[3]))
			px_height_changes.append(float(row[4]))
			mm_widths.append(float(row[5]))
			mm_heights.append(float(row[6]))
			mm_width_changes.append(float(row[7]))
			mm_height_changes.append(float(row[8]))
			
		dict = {'timestamps': timestamps,
		 'objects': objects,
		 'px_widths': px_widths,
		 'px_heights': px_heights,
		 'px_width_changes': px_width_changes,
		 'px_height_changes': px_height_changes,
		 'mm_widths': mm_widths,
		 'mm_heights': mm_heights,
		 'mm_width_changes': mm_width_changes,
		 'mm_height_changes': mm_height_changes}
		return dict
		#plt.scatter(mm_heights, forces)
		#plt.show()

	def parseTimingFile(filename):
		f = open(filename, "r")
		global WEBCAM_START
		global ROSTOPIC_START
		for line in f:
			if 'webcam' in line:
				WEBCAM_START = int(line[8:])
			elif 'rostopic' in line:
				ROSTOPIC_START = int(line[10:])

		assert WEBCAM_START != 0, "webcam timing data not found."
		assert ROSTOPIC_START != 0, "rostopic timing data not found."

	def mergeTiming(rostopicdict, csvdict):
		rostopic_wrenches = []
		rostopic_positions = []
		for i in range(len(csvdict['timestamps'])):
			rostopic_index = 0
			while rostopic_index < len(rostopicdict['timestamps']) and rostopicdict['timestamps'][rostopic_index] < csvdict['timestamps'][i]:
				rostopic_index += 1
			rostopic_wrenches.append(rostopicdict['wrenches'][rostopic_index])
			rostopic_positions.append(rostopicdict['positions'][rostopic_index])

		csvdict['rostopic_wrenches'] = rostopic_wrenches
		csvdict['rostopic_positions'] = rostopic_positions
		return csvdict

	def saveAsCSV(filename, mergedict):
		with open(filename, 'wb') as csvfile:
			csvwriter = csv.writer(csvfile, delimiter=',', quotechar='|')
			csvwriter.writerow(['timestamp', 'object', 'px_widths',
			 'px_heights', 'px_width_changes', 'px_height_changes', 'mm_widths', 
			 'mm_heights', 'mm_width_changes', 'mm_height_changes', 
			 'rostopic_wrenches', 'rostopic_positions'])
			for i in range(len(mergedict['timestamps'])):
				csvwriter.writerow([mergedict['timestamps'][i],
					mergedict['objects'][i],
					mergedict['px_widths'][i],
					mergedict['px_heights'][i],
					mergedict['px_width_changes'][i],
					mergedict['px_height_changes'][i],
					mergedict['mm_widths'][i],
					mergedict['mm_heights'][i],
					mergedict['mm_width_changes'][i],
					mergedict['mm_height_changes'][i],
					mergedict['rostopic_wrenches'][i],
					mergedict['rostopic_positions'][i]])

	def main():
		rostopic_filename = sys.argv[1]
		csv_filename = sys.argv[2]
		webcam_data_filename = sys.argv[3]
		timing_filename = sys.argv[4]

		parseTimingFile(timing_filename)
		print "ROSTOPIC_START = ", ROSTOPIC_START
		print "WEBCAM_START = ", WEBCAM_START
		csvdict = parseCSV(csv_filename, webcam_data_filename)
		#print csvdict

		rostopicdict = parseRostopic(rostopic_filename)
		#print rostopicdict

		mergedict = mergeTiming(rostopicdict, csvdict)
		saveAsCSV('merge.csv', mergedict)


		#plt.scatter(xarr, yarr)
		#plt.show()

if __name__ == "__main__":
	main()