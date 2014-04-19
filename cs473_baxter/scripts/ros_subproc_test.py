#!/usr/bin/env python

"""Rostopic time opening test module."""

rospy.init_node("rostopic_time_test")

time_data = open('TEST.txt', 'a+')
r_data = open('ROSTEST.txt', 'a+')
time_data.write('rostopic: ' + str(rospy.Time.now().secs) + "." + str(rospy.Time.now().nsecs) + '\n')
r_proc = subprocess.Popen(['rostopic', 'echo',
    '/robot/limb/right/endpoint_state'],
    stdout=r_data)
time.sleep(.3) # make into a command line argument
r_proc.terminate()
time_data.write('rostopic: ' + str(rospy.Time.now().secs) + "." + str(rospy.Time.now().nsecs) + '\n')
time_data.close()
r_data.close()