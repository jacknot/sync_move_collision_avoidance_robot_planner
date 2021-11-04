import math

from trajectory_msgs.msg import JointTrajectoryPoint

from utils.math_functions import signedAngleDiff, angleSawtooth

class Node(object):
    def __init__(self, config, timeFromStart):
        """
            timeFromStart: rospy.Duration
        """
        super(Node, self).__init__()
        self.config = config
        self.path = []
        self.timeFromStart = timeFromStart
        self.parent = None
        self.cost = 0.0

    def toPoint(self):
        pt = JointTrajectoryPoint()
        pt.positions = self.config
        pt.time_from_start = self.timeFromStart
        return pt

    def computePathAndFollow(self, jointDiffs, duration, nnodes, stateChecker, startStamp):
        self.path = [[self.config, self.timeFromStart]]

        for _ in range(0, int(nnodes)):
            self.config = [angleSawtooth(j + diff) for j, diff in zip(self.config, jointDiffs)]
            self.timeFromStart += duration
            if not stateChecker.checkState(self.config, startStamp + self.timeFromStart):
                return False
            self.path.append([self.config, self.timeFromStart])
            
        return True

    def jointDiff(self, node):
        return [signedAngleDiff(s, t) for s, t in zip(self.config, node.config)]

    def timeDiff(self, node):
        return node.timeFromStart - self.timeFromStart

    def pathTo(self, node, nnodes):
        self.path = [[self.config, self.timeFromStart]]
        jDiffs = self.jointDiff(node)
        
        jDiffs = list(map(lambda x: x / float(nnodes), jDiffs))
        tDiff = self.timeDiff(node) / float(nnodes)
        for _ in range(0, int(nnodes)):
            config, timeFromStart = self.path[-1]
            config = [angleSawtooth(j + jd) for j, jd in zip(config, jDiffs)]
            timeFromStart += tDiff
            self.path.append([config, timeFromStart])
        return jDiffs, tDiff

    def distance(self, node):
        return sum(map(lambda t: math.pow(signedAngleDiff(t[0], t[1]), 2), zip(self.config, node.config)))

    def __str__(self):
        nodeStr = "Node(%s, %s){ hash:%s }"
        parentStr = "/" if self.parent is None else "Node{ hash: %s }" % (hash(self.parent))
        return nodeStr % (self.config, self.timeFromStart, self.__hash__()) + " <- %s" % parentStr
    
    def __hash__(self):
        return hash(tuple(self.config))
