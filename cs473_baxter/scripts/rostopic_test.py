#!/usr/bin/env python

"""Rostopic time opening test module.

In your ros_ws workspace, run:
    rosrun cs473_baxter rostopic_test.py -f [FILE] -s [SECONDS]

The seconds that worked on our workstation was 0.3.
"""

import time
import argparse
import subprocess

import rospy

def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    parser.add_argument(
        '-f', '--file', dest='filename', required=True,
        help="the file to save to and the path to that file"
    )
    parser.add_argument(
        '-s', '--seconds', dest='seconds', required=True,
        help="how long to sleep"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("rostopic_time_test")
    r_time = rospy.Time()

    r_data = open(args.filename, 'a+')
    print 'start: ' + str(r_time.now().secs) + "." + str(r_time.now().nsecs)
    r_proc = subprocess.Popen(['rostopic', 'echo',
        '/robot/limb/right/endpoint_state'],
        stdout=r_data)
    time.sleep(float(args.seconds))
    r_proc.terminate()
    print 'end: ' + str(r_time.now().secs) + "." + str(r_time.now().nsecs)
    r_data.close()

if __name__ == '__main__':
    main()