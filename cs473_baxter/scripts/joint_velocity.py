#!/usr/bin/env python

# Copyright (c) 2013, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import math
import random

import genpy
import rospy

from std_msgs.msg import (
    UInt16,
)

import baxter_interface

from basic_poke import BasicMove


class JointVelocity(object):

    def __init__(self):
        self._pub_rate = rospy.Publisher('robot/joint_state_publish_rate',
                                         UInt16)
        self._left_arm = baxter_interface.limb.Limb("left")
        self._right_arm = baxter_interface.limb.Limb("right")
        self._left_joint_names = self._left_arm.joint_names()
        self._right_joint_names = self._right_arm.joint_names()

        # set joint state publishing to 100Hz
        self._pub_rate.publish(100)

    def _reset_control_modes(self):
        rate = rospy.Rate(100)
        for _ in xrange(100):
            if rospy.is_shutdown():
                return False
            self._left_arm.exit_control_mode()
            self._right_arm.exit_control_mode()
            self._pub_rate.publish(100)
            rate.sleep()
        return True

    def clean_shutdown(self):
        print("\nExiting example...")
        #return to normal
        self._reset_control_modes()
        #self._right_arm.move_to_neutral()
        return True

    def move(self):
        rate = rospy.Rate(100) # in Hz
        start = rospy.Time.now()

        elapsed = rospy.Time.now() - start
        while elapsed < genpy.rostime.Duration(2): # in secs
            self._pub_rate.publish(100) # in Hz
            cmd = {'right_s1': -5, 'right_e1':-20, 'right_w1':-40}
            self._right_arm.set_joint_velocities(cmd)
            #print self._right_arm.endpoint_effort()
            elapsed = rospy.Time.now() - start
            rate.sleep()


def main():
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_velocity")

    bm = BasicMove('right')
    bm.move_to_jp(bm.get_jp_from_file())

    joint_velocity = JointVelocity()
    rospy.on_shutdown(joint_velocity.clean_shutdown)
    joint_velocity.move()
    bm.move_to_jp(bm.get_jp_from_file())

    print("Done.")

if __name__ == '__main__':
    main()
