#!/usr/bin/env python
import math
import sys
import os
import time

from threading import Lock
from rosgraph.masterapi import Error

parentDir = os.path.dirname(os.path.dirname(__file__))
os.chdir(parentDir)
sys.path.append(parentDir)

import rospy
from moveit_msgs.srv import GetStateValidityRequest, GetStateValidity
from moveit_msgs.msg import RobotState

from utils.math_functions import invLinInt, vecAngleLinInt
from utils.logging import Logger, Level

class StateChecker(object):
    def __init__(self, wlogger=True):
        
        if wlogger:
            self._logger = Logger(id=self.__class__.__name__)
        else:
            self._logger = Logger(id=self.__class__.__name__, level = Level.error)
            
        self._joints = [
            ('shoulder_pan_joint', (-math.pi, math.pi)), 
            ('shoulder_lift_joint', (-math.pi, math.pi)), 
            ('elbow_joint', (-math.pi, math.pi)), 
            ('wrist_1_joint', (-math.pi, math.pi)), 
            ('wrist_2_joint', (-math.pi, math.pi)), 
            ('wrist_3_joint', (-math.pi, math.pi))
        ]
        self.macroGroupID = 'dual_arm'
        self.groups = {
            'left_arm': list(map(lambda t: ('rleft_%s' % t[0], t[1]), self._joints)), 
            'right_arm': list(map(lambda t: ('rright_%s' % t[0], t[1]), self._joints))
        }
        self._logdebug("JointGroups: %s" % self.groups)

        self._logdebug("Init: init RobotState ")
        self._rs = RobotState()
        self._rs.joint_state.name = self._statePolicy({k:[t[0] for t in v] for k, v in self.groups.items()})
        self._logdebug("RobotState.joint_state.name: %s" % self._rs.joint_state.name)
        self._rs.joint_state.position = [0.0]*12

        self._logdebug("Init: init state check validity service ")
        self._validitySrv = rospy.ServiceProxy('/two_arms/check_state_validity', GetStateValidity)
        self._loginfo("Init: waiting for check_state_validity service...")
        self._validitySrv.wait_for_service()
        
        self.trajectories = None
        self.focus = None

    def setTrajectories(self, trajectories):
        self.trajectories = trajectories
        self._logdebug("SetTrajectories: %s" % dict(map(lambda t: (t[0], len(t[1].points)), self.trajectories.items())))

    def _statePolicy(self, groups):
        for k in self.groups.keys():
            if k not in groups.keys():
                return None
        return list(groups['left_arm']) + list(groups['right_arm'])
    
    def focusOn(self, groupID):
        self._loginfo("Focus on \'%s\'" % groupID)
        if groupID not in self.groups.keys():
            raise ValueError("Group ID \'%s\' not recognized" % groupID)
        self.focus = groupID

    def _jointLimits(self, groupID):
        if groupID not in self.groups.keys():
            return [t[1] for t in self._joints]
        return [t[1] for t in self.groups[groupID]]
    
    def jointLimits(self):
        return self._jointLimits(self.focus)
    
    def checkState(self, state, timestamp):
        """
            @Params:
                timestamp: rospy.Time
        """
        if self.focus is None:
            raise ValueError("No focus group is specified")
        globalState = {}
        globalState[self.focus] = state

        for groupID, jTraj in self.trajectories.items():
            if groupID == self.focus:
                continue
            idx = self.binarySearch(jTraj, 0, len(jTraj.points) - 2, timestamp)
            
            if idx < 0:
                self._logdebug("[%s] Trj: Relative timestamp idx not found" % groupID)
                if timestamp < jTraj.header.stamp + jTraj.points[0].time_from_start:
                    globalState[groupID] = jTraj.points[0].positions
                else:
                    globalState[groupID] = jTraj.points[-1].positions
            else:
                self._logdebug("[%s] Trj: Relative timestamp idx: %d" % (groupID, idx))

                l = invLinInt(
                    (jTraj.header.stamp + jTraj.points[idx].time_from_start).to_nsec(), 
                    (jTraj.header.stamp + jTraj.points[idx + 1].time_from_start).to_nsec(),
                    timestamp.to_nsec()
                )
                globalState[groupID] = vecAngleLinInt(
                    jTraj.points[idx].positions, 
                    jTraj.points[idx + 1].positions, 
                    l
                )

        return self.check(self._statePolicy(globalState), self.macroGroupID)
    
    def check(self, state, groupID, constraints = None):
        self._rs.joint_state.position = state
        gsvr = GetStateValidityRequest()
        gsvr.robot_state = self._rs
        gsvr.group_name = groupID
        if constraints != None:
            gsvr.constraints = constraints

        try:
            result = self._validitySrv.call(gsvr)
        except Error as e:
            self._logwarn("Exception: %s" % e)
            result = None
            
        self._logdebug("Check: Valid: %s, contactPts: %s, state: %s" % (result.valid, result.contacts, state))
        return result.valid

    def binarySearch(self, jTrj, low, high, timestamp):
        """
            @Params:
                jTrj: rospy.JointTrajectory
            @return: 
                idx: -1 if not found, such that pts[idx].duration + tStart <= timestamp <= pts[idx+1] + tStart
        """
        if low > high:
            return -1
        t0 = jTrj.header.stamp
        mid = (high + low) / 2
        tMidLow = t0 + jTrj.points[mid].time_from_start
        tMidHigh = t0 + jTrj.points[mid + 1].time_from_start
        if tMidLow <= timestamp <= tMidHigh:
            return mid 
        elif timestamp < tMidLow:
            return self.binarySearch(jTrj, low, mid - 1, timestamp)
        else:
            return self.binarySearch(jTrj, mid + 1, high, timestamp)

    def clone(self):
        newSC = StateChecker()
        newSC.setTrajectories(self.trajectories)
        newSC.focusOn(self.focus)
        return newSC

    def _loginfo(self, string):
        self._logger.info(string)
    
    def _logdebug(self, string):
        self._logger.debug(string)
    
    def _logwarn(self, string):
        self._logger.warn(string)
