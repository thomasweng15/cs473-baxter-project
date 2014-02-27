#!/usr/bin/env python

import sys

import rospy

import baxter_interface

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

class BasicMove():
	def __init__(self, limb):
		self._limb = baxter_interface.limb.Limb(limb) 

		self._ik_srv = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		self._iksvc = rospy.ServiceProxy(self._ik_srv, SolvePositionIK)
		self._ikreq = SolvePositionIKRequest()

		rs = baxter_interface.RobotEnable()
		print "Enabling robot..."
		rs.enable()

	def _find_jp(self, position):
		self._ikreq.pose_stamp.append(position)
		
		try:
			rospy.wait_for_service(self._ik_srv, 5.0)
			resp = self._iksvc(self._ikreq)
			if resp.isValid[0]:
				print "SUCCESS - Valid Joint Solution Found"
				# Format solution into Limb API-compatible dictionary
				limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
				return limb_joints
			else:
				print "INVALID POSE - No Valid Joint Solution Found."
				return False
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))
			return False
	
	def set_neutral(self):
		self._limb.move_to_neutral()

	def move_to_position(self, position):
		limb_joints = self._find_jp(position) 
		if limb_joints != False:
			self._limb.move_to_joint_positions(limb_joints)
			return True
		else:
			return False


def main():
	rospy.init_node("cs473_basic_poke")

	bm = BasicMove('right')

	print "Moving to neutral pose..."
	bm.set_neutral()

	# Hard-coded pose
	hdr = Header(stamp=rospy.Time.now(), frame_id='base')
	poses = {
		'right': PoseStamped(
			header=hdr,
	   		pose=Pose(
	       		position=Point(
	           		x=0.656982770038,
	           		y=-0.852598021641,
	           		z=0.0388609422173,
	       		),
	       		orientation=Quaternion(
	           		x=0.367048116303,
	           		y=0.885911751787,
	           		z=-0.108908281936,
	           		w=0.261868353356,
	       		),
	   		),
		),
	}

	bm.move_to_position(poses['right'])

if __name__ == '__main__':
	sys.exit(main()) 