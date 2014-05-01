#!/usr/bin/python

"""Module for controlling the webcam."""

import time
import sys
import os
import argparse

import cv2

import rospy
import genpy

class Webcam(object):
    """Provides an interface to open an image capture device
    to take both manual and automatic snapshots.
    """
    def __init__(self, img_dir):
        self.img_dir = img_dir
        self.is_open = False
        self.capture = None

    def open(self, device=0):
        """Intialize the capture device.

        params:
            device      the id of the device to open
        """
        print "Opening capture device."
        self.capture = cv2.VideoCapture(device)
        self.capture.set(3, 960) # CV_CAP_PROP_FRAME_WIDTH
        self.capture.set(4, 720) # CV_CAP_PROP_FRAME_HEIGHT
        
        if not self.capture:
            print "Error opening capture device"
            sys.exit(1)
        else:
            self.is_open = True

    def close(self):
        """Close the capture device."""
        print "Closing capture device."
        if self.is_open:
            self.capture.release()
            self.capture = None
            self.is_open = False
        else:
            print "Error: no capture device open, cannot close device."

    def show_video_stream(self):
        """Show a video feed from the webcam.

        Quit when the 'q' key is pressed.
        """
        self.open()
        while True:
            val, frame = self.capture.read()
            cv2.imshow("input", frame)

            key = cv2.waitKey(50) % 256
            if key == ord('q'):
                break
        self.close()

    def take_manual_snapshot(self, filename, num=1, delay=0.5):
        """Show a video feed from the webcam,
        taking a snapshot when SPACE is pressed.

        params:
            filename    base name with which to save snapshots.
            num         max number of snapshots to take.
            delay       minimum # of seconds to wait between snapshots.

        Quit when the 'q' key is pressed.
        """
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
                    cur_name = filename
                    if num > 1:
                        file_split = os.path.splitext(filename)
                        cur_name = file_split[0] + str(num) + file_split[1]
                    cv2.imwrite(os.path.join(self.img_dir, cur_name), frame)
                    time.sleep(delay)
                    print "Image saved."
                break

    def take_automatic_snapshot(self, filename, sleep=2, duration=8, delay=200):
        """Automatically take snapshots from the webcam.

        params:
            filename    base name with which to save snapshots.
            duration    time in seconds in which to take snapshots.
            delay       interval in ms between snapshots.
        """
        self.open()

        rate = rospy.Rate(delay)    # sleep rate between snapshots
        num = 0                     # index of snapshot
        rospy_time = rospy.Time()   # time object
        start = rospy_time.now()    # start of snapshot taking
        elapsed = rospy_time.now() - start  # time now vs. start
        sec = 1000000000            # one second in nanoseconds
        num_secs = 0                # number of seconds that have elapsed. 

        time_data = open(os.path.join(self.img_dir, "webcam_data.txt"), 'a+')
        time_data.write("webcam start: " + str(start.nsecs) + '\n')

        over_nine = False
        while elapsed < genpy.rostime.Duration(duration):
            val, frame = self.capture.read()

            if elapsed > genpy.rostime.Duration(sleep):
                cur_name = filename + ("%03d" % num) + ".png"
                num += 1
                cv2.imwrite(os.path.join(self.img_dir, cur_name), frame)

                t_stamp = rospy_time.now() - start

                if t_stamp.nsecs > 900000000:
                	over_nine = True
                else:
                	if over_nine:
                		num_secs += 1
                	over_nine = False
                		
                formatted_stamp = "%010d" % (num_secs * sec + t_stamp.nsecs)
                time_data.write(("%03d" % num) + ":" + formatted_stamp + "\n")
                

            rate.sleep()
            prev = elapsed
            elapsed = rospy_time.now() - start


        time_data.close()
        self.close()

    def take_snapshot(self, filename, sleep=2):
        """Take one snapshot from the webcam.

        params:
            filename    base name with which to save snapshots.
        """
        self.open()
        start = rospy.Time.now()
        elapsed = rospy.Time.now() - start

        while elapsed < genpy.rostime.Duration(sleep):
            val, frame = self.capture.read()
            elapsed = rospy.Time.now() - start

        val, frame = self.capture.read()
        cv2.imwrite(os.path.join(self.img_dir, filename), frame)
        self.close()
        print "Image saved."


def main():
    """Module for interfacing with the webcam.

    Run this module as a standalone program by running 
    `rosrun cs473_baxter webcam.py [args]`
    
    Running the module by itself is useful for 
    adjusing the camera angle (see comment below) or 
    for debugging."""
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    parser.add_argument(
        '-d', '--dir', dest='directory', required=False,
        help="the directory to save to"
    )
    parser.add_argument(
        '-t', '--time', dest='time', required=False,
        help="how long to take snapshots"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    if args.directory is None:
        args.directory = '.'
    if args.time is None:
        args.time = 8
    else:
        args.time = int(args.time)

    rospy.init_node("webcam")

    cam = Webcam(args.directory)
    cam.take_automatic_snapshot("compression", duration=args.time)
    
    # Comment out the line above and un-comment the line below
    # in order to access the camera for positioning. 

    #cam.show_video_stream()

if __name__ == '__main__':
    main()

