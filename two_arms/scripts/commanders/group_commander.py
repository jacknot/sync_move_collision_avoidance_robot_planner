#!/usr/bin/env python
from threading import Lock
import os
import sys

import rospy
from actionlib_msgs.msg import GoalStatusArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionFeedback
import moveit_commander
import actionlib

parentDir = os.path.dirname(os.path.dirname(__file__))
os.chdir(parentDir)
sys.path.append(parentDir)

from status.action_status import ActionStatus
from utils import joint_trj2follow_joint_trj
from utils.clock_sub import Clock
from utils.logging import Level, Logger

class GroupCommander(object):
    def __init__(self, 
                    groupName, 
                    descriptionParam = "robot_description", 
                    ns = "two_arms",
                    trajectoryNameServer = "follow_joint_trajectory"
                ):
        self._logger = Logger(id=self.__class__.__name__, level=Level.info)
        self.id = groupName
        self._loginfo("Initializing ")
        
        # Init ros stuff
        self.ns = ns
        self._absNS = "/%s" % self.ns
        self.descParam = descriptionParam
        self._absDesc = "%s/%s" % (self._absNS, self.descParam)

        # Sub to clock
        self._loginfo("Init: Tracking ros clock ")
        self._clock = Clock()

        # Mechanism to interact with Action:follow_joint_trajectory:Status
        self.trajectoryNS = trajectoryNameServer
        self.controllerName = "%s_controller" % self.id
        self._actionServer = "%s/%s/%s" % (
            self._absNS,
            self.controllerName,
            self.trajectoryNS
        )        
            
        # Initializing action client
        self._actionClient = actionlib.SimpleActionClient(self._actionServer, FollowJointTrajectoryAction)     
        self._actionClient.wait_for_server()            

        # Subscribing to Status topic
        ## Status list
        self._stateTopic = "%s/status" % self._actionServer
        self._statusList = list()
        self._statusListLock = Lock()
        self._loginfo("Init: Subscribing to 'Status' topic ")
        self._stateSub = rospy.Subscriber(self._stateTopic, GoalStatusArray, self._callbackStatus)
        
        # Subscribing to Feedback topic
        ## Goal ID
        self._currentGoalID = ""
        self._currentGoalIDLock = Lock()
        self._feedbackTopic = "%s/feedback" % self._actionServer
        self._loginfo("Init: Subscribing to 'Feedback' topic ")
        self._feedbackSub = rospy.Subscriber(self._feedbackTopic, FollowJointTrajectoryActionFeedback, self._callbackFeedback)

        #moveit_commander.roscpp_initialize(sys.argv)
        self._moveitGroup = moveit_commander.MoveGroupCommander(name=groupName, robot_description=self._absDesc, ns=self.ns)
        
        self.currentTrajectory = JointTrajectory()
        self.currentTrajectory.points.append(JointTrajectoryPoint())
        self.currentTrajectory.points[-1].positions = [0.0]*6
        
    def _callbackFeedback(self, data):
        self._currentGoalIDLock.acquire()
        try:
            #Getting goal ID
            self._currentGoalID = data.status.goal_id.id
        finally:       
            self._currentGoalIDLock.release()

    def _callbackStatus(self, data):
        self._statusListLock.acquire()
        try:
            self._statusList = data.status_list
        finally:
            self._statusListLock.release()

    @property
    def statusListInfo(self):
        listStatus = list()    
        self._statusListLock.acquire()
        try: 
            for status in self._statusList:
                listStatus.append("Goal ID %s - status: %s" % (
                    status.goal_id.id, 
                    list(ActionStatus)[status.status].name
                ))
        finally:   
            self._statusListLock.release()
        return listStatus

    @property
    def state(self):
        states = list(ActionStatus)
        return states[self._actionClient.get_state()]
                    
    @property
    def ready(self):
        return self.isQueueFree() and self.state.terminal
    
    def isQueueFree(self):
        self._statusListLock.acquire()
        response = True
        try:
            if len(self._statusList):
                states = list(ActionStatus)
                for goal in self._statusList:
                    state = states[goal.status]
                    if not state.terminal:
                        response = False
        except Exception as e:
            response = False    #default for security reason
        finally:
            self._statusListLock.release()
        return response        
    
    def go(self, plan, wait=False, callback=None, force=False):
    
        if callback is not None and not callable(callback):
            raise ValueError("The callback is not a function")
        
        state = self.state
        if state.terminal or force:
            if force and not state.terminal:
                self._actionClient.cancel_goal()
            
            #Initializing trajectory from plan
            goal = joint_trj2follow_joint_trj(plan)
            goal.trajectory.header.stamp = rospy.Time()
            self.currentTrajectory = plan
            self.currentTrajectory.header.stamp = self._clock.value
            
            if wait:
                self._loginfo("Sending synchronous goal")
                self._actionClient.send_goal_and_wait(goal)
            else:
                self._loginfo("Sending asynchronous goal")
                self._actionClient.send_goal(goal, done_cb=callback)
            return True
        else:
            self._logwarn("The previous Goal has not been completed yet. Current status: %s" % state.name)
            return False

    def plan(self):
        return self._moveitGroup.plan()
               
    @property
    def currentPose(self):
        """return: geometry_msgs.msg.Pose"""
        return self._moveitGroup.get_current_pose()

    def setTargetPose(self, pose):
        """pose: geometry_msgs.msg.Pose"""
        self._moveitGroup.set_pose_target(pose)
    
    def setRandomPose(self):
        self._moveitGroup.set_random_target()
    
    def _loginfo(self, string):
        self._logger.info("[%s] %s" % (self.id, string))
    
    def _logwarn(self, string):
        self._logger.warn("[%s] %s" % (self.id, string))
