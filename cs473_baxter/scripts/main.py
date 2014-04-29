#!/usr/bin/env python

"""The experiment module"""

import os
import time
import subprocess
import sys
import yaml

import rospy

import baxter_interface

from position_control import PositionControl
from webcam import Webcam
from cs473vision.cs473vision.view_baxter import BaxterExperiment

CONFIG = './src/cs473-baxter-project/cs473_baxter/config/config.yaml'

class BoxFit(object):
    """The primary module for running compression trials.
    Links the webcam, vision segmentation, and actuation
    modules together.
    """
    def __init__(self):
        rospy.init_node("cs473_box_fit")

        self.is_glove_attached()

        # Verify robot is enabled
        print "Getting robot state..."
        self._rs = baxter_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print "Enabling robot..."
        self._rs.enable()
        print "Running. Ctrl-c to quit"

        self.p_ctrl = PositionControl('right')

        self.img_dir = self._create_img_dir()

    def is_glove_attached(self):
        """Prompt the user to check if Baxter's pusher glove
        is attached or not. Exit the process if it is not.
        """
        glove_on = raw_input("Is Baxter's glove attached? (y/n): ")
        if glove_on is not "y":
            print "\nERROR: Attach glove with glove.py before running BoxFit."
            sys.exit(1)

    def _create_img_dir(self):
        """Creates a timestamped folder in the img_dir directory
        that stores the images of one compression run.
        """
        config_file = open(CONFIG)
        dataMap = yaml.safe_load(config_file)
        base_img_dir = dataMap["image_directory"]
        img_dir = ''.join([base_img_dir, time.strftime("%d%m%Y_%H-%M-%S")])
        os.mkdir(img_dir)
        return img_dir

    def set_neutral(self):
        """Move arm(s) to initial joint positions."""
        self.p_ctrl.set_neutral()

    def take_reference_images(self, camera):
        """Takes reference images like the background image, 
        reference object image, arm image, and object image."""
        print 'Taking snapshot of the background.'
        camera.take_snapshot('background.png')

        raw_input("Place reference object in center. Press ENTER when finished.")
        camera.take_snapshot('reference.png')
        raw_input("Remove the reference object. Press ENTER when finished.")

        print 'Taking snapshot of just the arm'
        joint_pos = self.p_ctrl.get_jp_from_file('r_arm_check_positions')
        self.p_ctrl.move_to_jp(joint_pos)
        camera.take_snapshot('arm.png')
        self.set_neutral()

        raw_input("Place object alone in center. Press ENTER when finished.")
        camera.take_snapshot('object.png')

    def compress_object(self):
        """Compress an object while opening the webcam to take
        snapshots during the compression.
        """
        joint_pos = self.p_ctrl.get_jp_from_file('r_arm_init_positions')
        self.p_ctrl.move_to_jp(joint_pos)

        # Suppress collision detection and contact safety
        contact_safety_proc = subprocess.Popen(['rostopic', 'pub',
            '-r', '10',
            '/robot/limb/right/suppress_contact_safety',
            'std_msgs/Empty'])

        time_data = open(os.path.join(self.img_dir, 'timestamps.txt'), 'a+')
        r_data = open(os.path.join(self.img_dir, 'rostopic_data.txt'), 'a+')

        time_data.write('webcam: ' + str(rospy.Time.now().nsecs) + '\n')
        w_proc = subprocess.Popen(['rosrun', 'cs473_baxter', 'webcam.py',
            "-d", self.img_dir,
            "-t", "12"])

        time.sleep(4) # Buffer time for webcam subprocess to get ready

        time_data.write('rostopic: ' + str(rospy.Time.now().nsecs) + '\n')
        r_proc = subprocess.Popen(['rostopic', 'echo',
            '/robot/limb/right/endpoint_state'],
            stdout=r_data)

        time_data.write('compress: ' + str(rospy.Time.now().nsecs) + '\n')
        self.p_ctrl.move_to_jp(
            self.p_ctrl.get_jp_from_file('r_arm_compress_positions'),
            timeout=10, speed=0.05)

        time.sleep(1.5)

        joint_pos = self.p_ctrl.get_jp_from_file('r_arm_init_positions')
        self.p_ctrl.move_to_jp(joint_pos)

        contact_safety_proc.terminate()
        r_proc.terminate()
        w_proc.terminate()
        time_data.close()

    def process_images(self):
        """Use the cs473vision module to process the images."""
        base = self.img_dir + "/"
        bg_path = base + 'background.png'
        arm_path = base + 'arm.png'
        uncompressed_path =  base + 'object.png'

        baxter_obj = BaxterExperiment(bg_path)
        baxter_obj.set_arm_image(arm_path)
        #baxter_obj.set_arm_color((h, s, v), (h, s, v))
        baxter_obj.set_uncompressed_image(uncompressed_path)

        print "Uncompressed size: " + str(baxter_obj.get_uncompressed_size())
        for i in range(999):
            path = base + "compression" + ('%03d' % i) + ".png"
            if os.path.isfile(path):
                baxter_obj.set_compressed_image(path)
            else:
                break
        print "Compressed size: " + str(baxter_obj.get_compressed_size())

        baxter_obj.export_sizes(base + "sizes.csv")
        baxter_obj.display_results()

    def clean_shutdown(self):
        """Clean up after shutdown callback is registered."""
        print "\nExiting box fit routine..."
        if not self._init_state and self._rs.state().enabled:
            print "Disabling robot..."
            self._rs.disable()

def main():
    """Experiment module"""
    box_fit = BoxFit()
    camera = Webcam(box_fit.img_dir)
    rospy.on_shutdown(box_fit.clean_shutdown)
    box_fit.set_neutral()

    box_fit.take_reference_images(camera)

    box_fit.compress_object()

    box_fit.process_images()

if __name__ == '__main__':
    main()
