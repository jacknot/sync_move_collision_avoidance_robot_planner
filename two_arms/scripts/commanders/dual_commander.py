#!/usr/bin/env python
from Queue import PriorityQueue
import time
import random
import sys
import os

import rospy
from std_msgs.msg import Bool

parentDir = os.path.dirname(os.path.dirname(__file__))
os.chdir(parentDir)
sys.path.append(parentDir)

from two_arms_msgs.msg import DualCommanderStatus, GroupCommanderStatus, GoalPose
from robot_state.state_checker import StateChecker
from commanders.group_commander import GroupCommander
from commanders.commander_return import RoutineResult
from status.action_status import ActionStatus
from status.trajectory_result import TrajectoryResult
from trajectory_rework.trajectory_reworker import TrajectoryReworker
from utils.logging import Logger, Level
from utils.clock_sub import Clock
from utils.probing import Chronometer
from utils.math_functions import ExponentialMovingAverage

class DualCommander(object):
    def __init__(self,
                    nodeName = "dual_commander", 
                    ns = "two_arms",
                    statusRate = 10,
                    max_retry = 5,
                ):
        self._logger = Logger(id=self.__class__.__name__, level=Level.info)

        self._statusRate = statusRate
        self.max_retry = max_retry
        self.ns = ns
        self._absNS = "/%s" % self.ns
        self.nodeName = nodeName
        self._absNode = "%s/%s" % (self._absNS, self.nodeName)

        self.groups = {
            'left_arm': GroupCommander(groupName="left_arm", ns=ns),
            'right_arm': GroupCommander(groupName="right_arm", ns=ns)
        }
        self.queue = PriorityQueue() # list()

        # Init topics
        self._clock = Clock()
        self._poseTopic = "%s/goal_pose" % self._absNode
        self._poseSub = rospy.Subscriber(self._poseTopic, GoalPose, self._callbackGoal)

        self._clearTopic = "%s/clear_queue" % self._absNode
        self._clearSub = rospy.Subscriber(self._clearTopic, Bool, self._callbackClear)

        self._statusTopic = "%s/status" % self._absNode
        self._statusPub = rospy.Publisher(self._statusTopic, DualCommanderStatus, queue_size = 1)
        self._statusTimer = rospy.Timer(rospy.Duration(float(1)/self._statusRate), self.publishState)

        self.stateChecker = StateChecker()
        self.trjReworker = TrajectoryReworker(stateChecker = self.stateChecker)
        
        self.timer = Chronometer()
        self._overhead = ExponentialMovingAverage(mean = rospy.Duration(secs=1))
        rospy.on_shutdown(self.close)
        
    def _callbackGoal(self, data):
        """
            @params: 
                data: GoalPose
        """
        self.queue.put((data.priority*(-1), self.max_retry, data))
    
    def _callbackClear(self, data):
        """Clear queue"""
        if data.data:
            self.queue = PriorityQueue()
            with self.queue.mutex:
                self.queue.queue.clear()
    
    def publishState(self, timerEvent):
        """
            Publish the current state
            @Param:
                timerEvent: richiesto dall'interfaccia rospy
        """
        status = DualCommanderStatus()

        status.left_arm = GroupCommanderStatus()
        status.left_arm.status = list(ActionStatus)[self.groups['left_arm'].state.code].name
        status.left_arm.is_moving = self.groups['left_arm'].state.moving
        status.left_arm.goals_list = self.groups['left_arm'].statusListInfo
        status.left_arm.current_pose = self.groups['left_arm'].currentPose

        status.right_arm = GroupCommanderStatus()
        status.right_arm.status = list(ActionStatus)[self.groups['right_arm'].state.code].name
        status.right_arm.is_moving = self.groups['right_arm'].state.moving
        status.right_arm.goals_list = self.groups['right_arm'].statusListInfo
        status.right_arm.current_pose = self.groups['right_arm'].currentPose

        status.queue = list(zip(*self.queue.queue))[2] if len(self.queue.queue) else list()
        self._statusPub.publish(status)

    def dispatch(self, extracted):
        self.timer.start()
        priority = extracted[0] #queue has priority in negative values
        retry = extracted[1]
        pose = extracted[2]
        
        if retry < 1:
            if priority*(-1) < 1:
                self._logwarn("Maximum number of retry reached for this pose: discarding pose")
                self.timer.reset()
                return RoutineResult.outOfRetry
            else:
                self._logwarn("Maximum number of retry reached for this pose: lowering priority [%d to %d]" % (priority*(-1), priority*(-1)-1))
                self.queue.put((priority+1, self.max_retry, pose))
                self.timer.reset()
                return RoutineResult.loweringPriority
        
        self._loginfo("Received: %s - dest: \'%s\', priority: %d[%d] - {%s,%s,%s:%s,%s,%s,%s}" % (
            pose.pose.header.seq, 
            "X" if pose.assign_to == "" else pose.assign_to,
            priority*(-1),
            retry,
            pose.pose.pose.position.x,
            pose.pose.pose.position.y,
            pose.pose.pose.position.z,
            pose.pose.pose.orientation.x,
            pose.pose.pose.orientation.y,
            pose.pose.pose.orientation.z,
            pose.pose.pose.orientation.w,
        ))
        
        ready = self._selectGroup(pose)
        self._loginfo("Assigned to \'%s\'" % ready)

        
        plan, remainingETA = self.defaultPlanner(ready, pose.pose)        
        if not len(plan.points):
            self._logwarn("Default planner has failed, back to queue")
            self.queue.put((priority, retry - 1, pose))
            self.timer.reset()
            return RoutineResult.defaultFailure
        
        plan, remainingETA, reworkDuration = self._reworkTrajectory(ready, plan)
        if plan is None:
            self._logwarn("Reworker has failed, back to queue ")
            self.queue.put((priority, retry - 1, pose))
            self.timer.reset()
            return RoutineResult.reworkerFailure
        elif remainingETA.to_sec() < 0.0:
            self._logwarn("Reworker took %fs more than expected, back to queue " % -remainingETA.to_sec())           
            self.queue.put((priority, retry - 1, pose))
            self.timer.stop()
            self._overhead.updateState(self.timer.duration - reworkDuration)
            return RoutineResult.timeout
        else:
            self._loginfo("Executing plan...")
            self.timer.stop()
            self._overhead.updateState(self.timer.duration - reworkDuration)
            if remainingETA.to_sec() > 0:
                self._loginfo("Waiting %fs" % (remainingETA.to_sec()))
                rospy.sleep(remainingETA)
            self.groups[ready].go(plan, wait=False, callback=self._callbackGoalReached(ready))
            return RoutineResult.success

    def routine(self):
        self._loginfo("........Waiting for a Pose........")
        r = self.dispatch(self.queue.get())
        self.queue.task_done()
        return r

    def defaultPlanner(self, groupID, pose):
        self._loginfo("Planning: default planner ")
        self.groups[groupID].setTargetPose(pose)
        return self.groups[groupID].plan().joint_trajectory, rospy.Duration()

    def _reworkTrajectory(self, groupID, plan):
        #if all(map(lambda t: t[1].state.terminal, self.groups.items())):
        #    return plan, rospy.Duration(), rospy.Duration()
        #self._loginfo("Other groups are moving, reworking trajectory ")

        self.stateChecker.setTrajectories(
            dict(map(lambda t: (t[0], t[1].currentTrajectory), self.groups.items()))
        )
        self.stateChecker.focusOn(groupID)

        estDuration = self.trjReworker.estimateDuration(plan, self._overhead.value)
        plan.header.stamp = self._clock.value
        if not self.trjReworker.reworkable(plan, estDuration):
            return None, rospy.Duration(), rospy.Duration()
	    
        plan = self.trjReworker.rework(plan, estDuration)
        self._loginfo("Rework took: %fs" % self.trjReworker.lastDuration.to_sec())
        return plan, self.trjReworker.estimatedDuration - self.trjReworker.lastDuration, self.trjReworker.lastDuration

    def _selectGroup(self, pose):
        while True:
            if pose.assign_to in self.groups.keys():
                if self.groups[pose.assign_to].ready:
                    return pose.assign_to
                    
                self._loginfo("Checked \'%s\', not ready, sleeping " % pose.assign_to)
            else:
                for name, group in random.shuffle(list(self.groups.items())): #in randomized order
                    if group.ready:
                        return name
                self._loginfo("Checked %s, not ready, sleeping " % (set(self.groups.keys())))
            time.sleep(0.5)

    def _callbackGoalReached(self, group):    
        def goalReached(status, result):
            self._loginfo("%s: Trajectory done | Status: %s - Result: %s" % (
                group, 
                list(ActionStatus)[status].name, 
                TrajectoryResult.get(result.error_code).text)
            )
        return goalReached
    
    def run(self):
        while not rospy.is_shutdown():
            self.routine()
            
    def close(self):
        self._loginfo("Closing down ")
        if hasattr(self, '_statusTimer'):
            self._statusTimer.shutdown()
    
    def _loginfo(self, string):
        self._logger.info(string)
    
    def _logwarn(self, string):
        self._logger.warn(string)
