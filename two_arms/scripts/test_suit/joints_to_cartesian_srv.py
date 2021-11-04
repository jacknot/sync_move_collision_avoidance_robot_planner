#!/usr/bin/env python
import rospy

import sys
import argparse

from moveit_msgs.srv import GetPositionFK
from moveit_msgs.srv import GetPositionFKRequest
from moveit_msgs.srv import GetPositionFKResponse
from sensor_msgs.msg import JointState


class JtcSrv(object):
    def __init__(self, frame_id='/world'):     
        self._frame_id = frame_id
        self._fk_srv = rospy.ServiceProxy('/two_arms/compute_fk', GetPositionFK)
        self._fk_srv.wait_for_service()

    def jointsToCartesian(self, joints, arm):
        if arm not in ["rleft", "rright"]:
            raise ValueError("Not a valid arm")
    
        req = GetPositionFKRequest()
        req.header.frame_id = self._frame_id
        req.fk_link_names = ['%s_tool0' % arm]
        req.robot_state.joint_state.name = ['%s_shoulder_pan_joint' % arm, '%s_shoulder_lift_joint' % arm, '%s_elbow_joint' % arm, '%s_wrist_1_joint' % arm, '%s_wrist_2_joint' %arm, '%s_wrist_3_joint' %arm]
        req.robot_state.joint_state.position = joints

        res = self._fk_srv.call(req)
        isinstance(res, GetPositionFKResponse)
        if res.error_code.val != 1:
            rospy.logerr("IK error, code is %d" % res.error_code.val)
            return None

        return res.pose_stamped[0]

def main():  
    
    if len(sys.argv) > 1:
        rospy.init_node("joints_to_cartesian_srv")

        pose = JtcSrv()

        cli = argparse.ArgumentParser()
        cli.add_argument("-left", nargs="*", type=float)
        cli.add_argument("-right", nargs="*", type=float)

        args = cli.parse_args()		

        if len(args.left) < 6 or len(args.right) < 6:
			rospy.logerr("Six joints configurations are required for each arm")
			return 

        left = pose.jointsToCartesian(args.left, "rleft")
        right = pose.jointsToCartesian(args.right, "rright")
        print("======Generated poses=======\n")
        print("left = GoalPose()" + "\n" +
        "left.pose.header.frame_id = '/world'" + "\n" +
        "left.pose.pose.position.x = " + str(left.pose.position.x) + "\n" +
        "left.pose.pose.position.y = " + str(left.pose.position.y) + "\n" +
        "left.pose.pose.position.z = " + str(left.pose.position.z) + "\n" +
        "left.pose.pose.orientation.x = " + str(left.pose.orientation.x) + "\n" +
        "left.pose.pose.orientation.y = " + str(left.pose.orientation.y) + "\n" +
        "left.pose.pose.orientation.z = " + str(left.pose.orientation.z) + "\n" +
        "left.pose.pose.orientation.w = " + str(left.pose.orientation.w) + "\n" +
        "left.assign_to = \"left_arm\""  + "\n\n" +

        "right = GoalPose()" + "\n" +
        "right.pose.header.frame_id = '/world'" + "\n" +
        "right.pose.pose.position.x = " + str(right.pose.position.x) + "\n" +
        "right.pose.pose.position.y = " + str(right.pose.position.y) + "\n" +
        "right.pose.pose.position.z = " + str(right.pose.position.z) + "\n" +
        "right.pose.pose.orientation.x = " + str(right.pose.orientation.x) + "\n" +
        "right.pose.pose.orientation.y = " + str(right.pose.orientation.y) + "\n" +
        "right.pose.pose.orientation.z = " + str(right.pose.orientation.z) + "\n" +
        "right.pose.pose.orientation.w = " + str(right.pose.orientation.w) + "\n" +
        "right.assign_to = \"right_arm\"")          
    else:
		rospy.logerr("Two joints configurations missing. Pass them with -left and -right parameters")
    

if __name__ == '__main__':
    main()
