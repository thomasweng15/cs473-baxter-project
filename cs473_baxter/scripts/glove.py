#!/usr/bin/env python

"""Compressor piece grip/release module"""

import argparse

import rospy

import baxter_interface

class Glove(object):
    """Provides an interface for attaching and releasing
    Baxter's pusher glove.
    """
    def __init__(self, gripper):
        rospy.init_node("cs473_gripper")

        self.gripper = baxter_interface.Gripper(gripper)

        # Verify robot is enabled
        print "Getting robot state..."
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print "Enabling robot..."
        self._rs.enable()
        print "Running. Ctrl-c to quit"

    def grip_glove(self):
        """Command Baxter to grip an item.

        Press ENTER to execute the command.
        """
        print "Calibrating gripper..."
        self.gripper.calibrate()
        self.gripper.open()
        print "Calibration complete."
        raw_input("Press ENTER when glove is in position.")
        self.gripper.close()

    def release_glove(self):
        """Command Baxter to release an item."""
        self.gripper.open()
        print "If Baxter does not release, try running this script again."
        print "If you see an error, you must manually remove the glove."


def main():
    """Glove grip/release module"""
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                    description=main.__doc__)
    parser.add_argument(
        '-g', '--grip', choices=['grip', 'release'], required=True,
        help="grip or release glove"
    )
    args = parser.parse_args(rospy.myargv()[1:])

    glove = Glove('right')

    if args.grip == 'grip':
        glove.grip_glove()
    else:
        glove.release_glove()

if __name__ == '__main__':
    main()
