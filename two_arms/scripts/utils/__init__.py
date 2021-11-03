from copy import deepcopy

import rospy
from control_msgs.msg import FollowJointTrajectoryGoal

# Conversion ---------------------------------------------------
def joint_trj2follow_joint_trj(jointTrajectory):
    """JointTrajectory -> FollowJointTrajectoryGoal"""
    goal =  FollowJointTrajectoryGoal()
    goal.trajectory = deepcopy(jointTrajectory)
    return goal

# MoveIt Version Control -----------------------------------------
def checkPlanningSceneVersion(planningScene):
    if not hasattr(planningScene, 'check'):
        rospy.logerr("Please upgrade the moveit_commander library using the bash script in moveit_upgrade")
        exit()
