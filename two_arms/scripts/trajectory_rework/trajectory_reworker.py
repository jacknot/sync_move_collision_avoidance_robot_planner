import math

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_rework import Node
#from trajectory_rework.rrt import RRT
#from trajectory_rework.rrt_star import RRTStar
from trajectory_rework.bidirectional_rrt import BidirectionalRRT

from utils.logging import Logger
from utils.math_functions import angleSawtooth, ExponentialMovingAverage
from utils.probing import Chronometer

class TrajectoryReworker(object):
    def __init__(self, 
                    stateChecker,
                    resolution = 0.1,
                ):
        """
            jointTrajectory: trajectory_msgs.msg.JointTrajectory
            toAvoid: list(trajectory_msgs.msg.JointTrajectory)
        """
        self._logger = Logger(id=self.__class__.__name__)
        self._loginfo("Init ")
        self.resolution = resolution
        self.stateChecker = stateChecker

        self._estimatedDuration = None
        self.startStamp = None
        self.result = None
        
        self.fromNode = None
        self.observedJointDiff = None
        self.observedDuration = None
        self.rrt = BidirectionalRRT(
            searchSpace = self.stateChecker.jointLimits(),
            resolution = self.resolution,
        )
        # Time estimation ------------
        self._timer = Chronometer()
        self._avgPointCheck = ExponentialMovingAverage(mean = rospy.Duration(secs=0.04))
        self._avgRRTRuns = ExponentialMovingAverage(mean = 1)
        self._rrtRuns = 0
    
    @property
    def lastDuration(self):
        return self._timer.duration
    
    @property
    def estimatedDuration(self):
        return self._estimatedDuration

    def _startStamp(self, header, duration):
        return header.stamp + duration

    def _setTrajectory(self, jT, estimatedDuration):
        """
            Set the target joint trajectory
            @Params:
                jT: rospy.JointTrajectory
        """
        self._estimatedDuration = estimatedDuration
        self.ogPtIDX = 0
        self.ogJointTrj = jT
        self._loginfo("Input pts: %d" % len(jT.points))

        self.result = JointTrajectory()
        self.result.header = jT.header
        self.result.joint_names = jT.joint_names
        self.startStamp = self._startStamp(self.result.header, estimatedDuration)

    def pop(self):
        """@return: Node, None if empty
        """
        if self.empty():
            return None
        popped = Node(
            self.ogJointTrj.points[self.ogPtIDX].positions,
            self.ogJointTrj.points[self.ogPtIDX].time_from_start
        )
        self.ogPtIDX += 1
        return popped

    def empty(self):
        return self.ogPtIDX >= len(self.ogJointTrj.points)

    def reworkable(self, jT, estimatedDuration):
        start = self._startStamp(jT.header, estimatedDuration)
        if not self.stateChecker.checkState(jT.points[0].positions, start + jT.points[0].time_from_start):
            self._logwarn("The initial point is not valid")
            return False
        elif not self.stateChecker.checkState(jT.points[-1].positions, start + jT.points[-1].time_from_start):
            self._logwarn("The final point is not valid")
            return False
        else:
            return True

    def rework(self, trj, estimatedDuration):
        self._timer.start()
        self.fromNode = None
        self.observedJointDiff = [ExponentialMovingAverage() for _ in range(len(trj.points[-1].positions))]
        self.observedDuration = ExponentialMovingAverage(mean = rospy.Duration())
        self.rrtRuns = 0
        self._setTrajectory(trj, estimatedDuration)

        result = self._rework()
        self._avgRRTRuns.updateState(self.rrtRuns)
        self._timer.stop()
        if result:
            self._avgPointCheck.updateState((self._timer.duration - self.rrtRuns * self.rrt.avgDuration) / float(len(result.points)))
        return result

    def estimateDuration(self, jointTrj, externalTime, tollerance = 0.05):
        """
            @Params:
                jointTrj: rospy.JointTrajectory
                externalTime: rospy.Duration
            @return:
                rospy.Duration: estimated total time
        """
        estimated = rospy.Duration()
        estimated += externalTime
        self._logdebug("Estimation: overhead: %f" % estimated.to_sec())
        estimated += (len(jointTrj.points) - 1) * self._avgPointCheck.value
        self._logdebug("Estimation: overhead + pointsCheck: %f" % estimated.to_sec())
        self._logdebug("Estimation: rrtAvgDuration: %f" % self.rrt.avgDuration.to_sec())
        self._logdebug("Estimation: avgRuns: %f" % self._avgRRTRuns.value)
        estimated += self._avgRRTRuns.value * self.rrt.avgDuration
        self._logdebug("Estimation: overhead + pointsCheck + rrtRuns: %f" % estimated.to_sec())
        estimated *= 1 + tollerance
        self._logdebug("Estimation: overhead + pointsCheck + rrtRuns + tollerance: %f" % estimated.to_sec())
        self._loginfo("Estimated time: %fs" % estimated.to_sec())
        return estimated

    def _rework(self):
        """
            @return: 
                JointTrajectory: goal trajectory
        """
        self._loginfo("Reworking ")
        self.fromNode = self.pop()
        self.result.points.append(self.fromNode.toPoint())
        while not self.empty():
            toNode = self.pop()
            self._computePath(self.fromNode, toNode)
            
            #self._loginfo("Looking for collisions in path: %s" % fromNode.path)
            collisions = self._collisions(self.fromNode, self.startStamp)
            self._logdebug("Collisions: %s, path: %s" % (collisions, self.fromNode.path))
            if len(collisions) == 0:
                toNode.parent = self.fromNode
                self.fromNode = toNode
                self.result.points.append(self.fromNode.toPoint())
            else:
                tmpNode = self.fromNode
                # Find consecutive collisions
                while not self.empty() and len(collisions) > 0 and max(collisions) == len(tmpNode.path) - 1:
                    tmpNode = toNode
                    toNode = self.pop()
                    self._computePath(tmpNode, toNode)
                    #self._loginfo("Looking for collisions in path: %s" % tmpNode.path)
                    collisions = self._collisions(tmpNode, self.startStamp)
                    self._logdebug("Collisions: %s, path: %s" % (collisions, tmpNode.path))

                self.rrtRuns += 1
                self.rrt.set(
                    start = self.fromNode,
                    goal = toNode,
                    expandJoints = list(map(lambda x: x.value, self.observedJointDiff)),
                    expandDuration = self.observedDuration.value,
                    startStamp = self.startStamp, 
                    stateChecker = self.stateChecker,
                )
                correction = self.rrt.run()
                if correction is None:
                    self._logwarn("Can't find a correction")
                    return None
                
                lastNode = self._fixRRTpath(
                    correction, 
                    self.fromNode.timeFromStart,
                    list(map(lambda x: x.value, self.observedJointDiff)),
                    self.observedDuration.value,
                    self.result
                )
                #for i in range(1, len(fixed)):
                #    self.result.points.append(fixed[i].toPoint())
                self.fromNode = lastNode
        self._loginfo("Path found, length: %d" % len(self.result.points))
        return self.result

    def _computePath(self, fromNode, toNode):
        for jDiff, obsJDiff in zip(fromNode.jointDiff(toNode), self.observedJointDiff):
            obsJDiff.updateState(abs(jDiff))
        if toNode.timeFromStart <= fromNode.timeFromStart:
            toNode.timeFromStart = fromNode.timeFromStart + self.observedDuration.value
        else:
            self.observedDuration.updateState(toNode.timeFromStart - fromNode.timeFromStart)
        fromNode.pathTo(
            toNode, 
            math.ceil(sum(map(lambda x: math.pow(x, 2), fromNode.jointDiff(toNode))) / float(math.pow(self.resolution, 2)))
        )

    def _collisions(self, node, startTimestamp):
        """@return: list()"""
        collisionIDXs = []
        for i, configAndtime in enumerate(node.path):
            if not self.stateChecker.checkState(configAndtime[0], startTimestamp + configAndtime[1]):
                collisionIDXs.append(i)
        return collisionIDXs

    def _fixRRTpath(self, rrtPath, startStamp, deltaJ, deltaT, result):
        """
            @Params:
                - rrtPath: trajectory_rework.Node[]
                - startStamp: rospy.Duration
                - deltaJ: float
                - deltaT: rospy.Duration
            @Return
                lastNode: trajectory_rework.Node: l'ultimo nodo generato da \'rrtPath\'
        """
        fromNode = rrtPath.pop(0)
        fromNode.timeFromStart = startStamp
        while len(rrtPath):
            toNode = rrtPath.pop(0)
            jDiffs = fromNode.jointDiff(toNode)
            nnodes = math.ceil(max(map(lambda t: abs(t[0] / float(t[1])), zip(jDiffs, deltaJ))))
            jDiffs = map(lambda x: x / float(nnodes), jDiffs)
            timeDiff = fromNode.timeDiff(toNode) / float(nnodes)
            for _ in range(int(nnodes)):
                middleNode = Node(
                    list(map(lambda t: angleSawtooth(t[0] + t[1]), zip(fromNode.config, jDiffs))), 
                    fromNode.timeFromStart + timeDiff
                )
                middleNode.parent = fromNode
                self.result.points.append(middleNode.toPoint())
                fromNode = middleNode
        self._logdebug("Fixed RRT Path: %s" % result)
        return fromNode

    def _loginfo(self, string):
        self._logger.info(string)
    
    def _logwarn(self, string):
        self._logger.warn(string)
        
    def _logdebug(self, string):
        self._logger.debug(string)
