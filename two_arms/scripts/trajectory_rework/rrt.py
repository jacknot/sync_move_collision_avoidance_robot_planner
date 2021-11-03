import math
import random
import time

import rospy

from utils.math_functions import angleSawtooth
from trajectory_rework import Node
from trajectory_rework.algorithm import Algorithm

class RRT(Algorithm):
    def __init__(self,
                    searchSpace,
                    resolution = 0.1,
                    maxIter = 500,
                    goalSampleRate = 5,
                    expandDistFactor = 0.4,
                    id=""
                ):
        """RRT for trajectoryNodes"""
        super(RRT, self).__init__()
        self.nodes = []
        self.maxIter = maxIter

        self.start = None
        self.end = None
        self.stateChecker = None

        self.searchSpace = searchSpace
        self.expandDist = 0
        self.expandJoints = []
        self.expandDuration = rospy.Duration()
        self.startStamp = rospy.Time()

        self._setResolution(resolution)
        self.goalSampleRate = goalSampleRate
        self.expandFactor = expandDistFactor
        
        self.id = id

    def _setResolution(self, res):
        self.resolution = res
        self._sqrdResolution = math.pow(res, 2)

    def set(self, start, goal, expandJoints, expandDuration, startStamp, stateChecker):
        self._loginfo("To compose: {%s }->{ %s}" % (start, goal))
        self.start = start
        self.end = goal
        self.stateChecker = stateChecker

        self.expandDist = self.expandFactor * self.start.distance(self.end)
        self.expandJoints = expandJoints
        self.expandDuration = expandDuration
        self.startStamp = startStamp
        self.nodes = []

    def _run(self):
        self._loginfo("Start planning ")
        self.nodes = [self.start]
        for i in range(self.maxIter):
            self.explore(i)
            
            if self.dist2goal(self.nodes[-1]) <= self.expandDist:
                self._loginfo("[%d/%d] Can reach end node" % (i+1, self.maxIter))

                final, valid = self.steer(self.nodes[-1], self.end, self.expandDist, self.expandJoints, self.expandDuration)
                if valid:
                    self._loginfo("[%d/%d] Final: %s \n[NO collision found, computing path...]" % (i+1, self.maxIter, final))
                    return self.composePath(len(self.nodes) - 1)
                else:
                    self._loginfo("[%d/%d] Final: Collision found " % (i+1, self.maxIter))
        self._logwarn("No path found, run failed")
        return None

    def explore(self, i):
        rnd = self.newRandomNode(t = i)
        self._loginfo("[%d/%d] Rnd: %s" % (i+1, self.maxIter, rnd))
        nearest = self.getNearestNode(self.nodes, rnd)
        
        new, valid = self.steer(nearest, rnd, self.expandDist, self.expandJoints, self.expandDuration)  
        if valid:
            self._loginfo("[%d/%d] After steer: New [valid path]: %s" % (i+1, self.maxIter, new))
            self.nodes.append(new)
            return new
        self._loginfo("[%d/%d] After steer: New: collisions in path " % (i+1, self.maxIter))         
        return None

    def steer(self, fromNode, toNode, extendLength = float('inf'), extendJoints = [float('inf')]*6, extendDuration = rospy.Duration()):
        """Steer in the direction of toNode by a given length&time"""
        newNode = Node(fromNode.config, fromNode.timeFromStart)
        
        d = newNode.distance(toNode)
        # Factor: resize the jointDiffs if d gets resized
        factor = 1 if d <= extendLength else extendLength / float(d)
        d = d if d <= extendLength else extendLength
        nnodes = math.ceil(d / float(self._sqrdResolution))
        self._loginfo("Steer: nnodes: %d" % int(nnodes))
        if int(nnodes) == 0:
            newNode.path = list(toNode.path)
            newNode.cost = toNode.cost
            newNode.parent = toNode.parent
            return newNode, True

        jointDiffs = map(lambda x: factor * x / float(nnodes), newNode.jointDiff(toNode))
        # timeFactor: extendDuration is relative to extendJoints. Needed to resize the extendDuration to the current differences
        timeFactor = max(map(lambda t: abs(t[0] / float(t[1])), zip(jointDiffs, extendJoints)))
        timeDiff = timeFactor * extendDuration / float(nnodes)

        valid = newNode.computePathAndFollow(jointDiffs, timeDiff, nnodes, self.stateChecker, self.startStamp)
        if not valid:
            return newNode, valid
            
        if newNode.distance(toNode) <= self._sqrdResolution:
            newNode.config = toNode.config
            newNode.timeFromStart = newNode.path[-1][-1] + timeDiff
        newNode.parent = fromNode
        return newNode, valid
    
    def newRandomNode(self, t = 0, dims = 3):
        if self.goalSampleRate < random.randint(0, 100):
            #jIDXs = range(dims)
            #stdDevs = [math.pi / 3.0 * (0.5 + 0.5 * (idx - max(jIDXs)/2.0) * (t/float(self.maxIter) - 0.5) * (0.9 - 0.1)) for idx in jIDXs]
            #rndConfig = [angleSawtooth(joint + random.normalvariate(0, sigma)) for joint, sigma 
            #    in zip(self.end.config[:5], stdDevs)] + [self.end.config[-1]]
            rndConfig = [random.uniform(extMin, extMax) 
                            for extMin, extMax in self.searchSpace[:dims]
                        ]
            rndConfig.extend(self.end.config[-dims:])
            rnd = Node(
                rndConfig,
                self.expandDuration
            )
        else:
            rnd = Node(self.end.config, self.end.timeFromStart)
        return rnd

    def getNearestNode(self, nodes, rndNode):
        minNode, minD = None, float('inf')
        for node in nodes:
            d = node.distance(rndNode)
            if d < minD:
                minNode, minD = node, d
        return minNode

    def composePath(self, goalInd):
        path = [self.end]
        node = self.nodes[goalInd]
        while node.parent is not None:
            path.insert(0, node)
            node = node.parent
        path.insert(0, node)
        return path
    
    def dist2goal(self, node):
        return self.end.distance(node)


    def _loginfo(self, string):
        string = string if not self.id else "[%s] %s" % (self.id, string)
        self._logger.info("[%s] %s" % (self.id, string))
    
    def _logwarn(self, string):
        string = string if not self.id else "[%s] %s" % (self.id, string)
        self._logger.warn("[%s] %s" % (self.id, string))
    
    def _logdebug(self, string):
        string = string if not self.id else "[%s] %s" % (self.id, string)
        self._logger.debug("[%s] %s" % (self.id, string))
