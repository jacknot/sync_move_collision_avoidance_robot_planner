from concurrent.futures import ThreadPoolExecutor, as_completed, wait
import math

import rospy

from trajectory_rework.rrt import RRT
#from trajectory_rework.rrt_star import RRTStar
from trajectory_rework.algorithm import Algorithm

class BidirectionalRRT(Algorithm):
    def __init__(self,
                    searchSpace,
                    resolution = 0.1,
                    maxIter = 500,
                    goalSampleRate = 7,
    ):
        super(BidirectionalRRT, self).__init__()
        self._searchSpace = searchSpace
        self._setResolution(resolution)
        self._maxIter = maxIter
        self._goalSampleRate = goalSampleRate

        self.start = None
        self.end = None
        self._stateChecker = None
        self.expandDuration = rospy.Duration()
        self.expandJoints = []
        self.startStamp = rospy.Time()

        self._pool = None
        self._start2endRRT = RRT(
            searchSpace = self._searchSpace,
            resolution = self._resolution,
            maxIter = self._maxIter,
            goalSampleRate = self._goalSampleRate, 
            id = "s2e"
        )
        self._end2startRRT = RRT(
            searchSpace = self._searchSpace,
            resolution = self._resolution,
            maxIter = self._maxIter,
            goalSampleRate = self._goalSampleRate, 
            id = "e2s"
        )
    
    def _setResolution(self, res):
        self._resolution = res
        self._sqrdRes = math.pow(res, 2)
    
    def set(self, start, goal, expandJoints, expandDuration, stateChecker, startStamp):
        self._loginfo("To compose: {%s }->{ %s}" % (start, goal))
        self.start = start
        self.end = goal
        self._stateChecker = stateChecker

        self.expandJoints = expandJoints
        self.expandDuration = expandDuration
        self.startStamp = startStamp
        
        self._start2endRRT.set(
            self.start,
            self.end,
            self.expandJoints,
            self.expandDuration,
            self.startStamp,
            self._stateChecker.clone()
        )
        self._end2startRRT.set(
            self.end,
            self.start,
            self.expandJoints,
            self.expandDuration,
            self.startStamp, 
            self._stateChecker.clone()
        )

    def _run(self):
        self._pool = ThreadPoolExecutor(max_workers = 2)
        try:
            self._start2endRRT.nodes = [self._start2endRRT.start]
            self._end2startRRT.nodes = [self._end2startRRT.start]
            for i in range(self._maxIter):
                start2end = self._pool.submit(self._start2endRRT.explore, i)
                end2start = self._pool.submit(self._end2startRRT.explore, i)
                for f in as_completed([start2end, end2start]):
                    self._loginfo("[%d/%d] Done: %s" % (i+1, self._maxIter, f))
                start2end = start2end.result()
                end2start = end2start.result()

                path, found = self._findIntersection(
                    start2end, self._start2endRRT.nodes,
                    end2start, self._end2startRRT.nodes,
                )
                if found:
                    return path
            self._logwarn("No path found, run failed")
            return None
        finally:
            self._pool.shutdown()

    def _findIntersection(self, s2eNew, s2eNodes, e2sNew, e2sNodes):
        path, found = None, False
        s2eIDX, e2sIDX = 0, 0
        s2eLEN, e2sLEN = len(s2eNodes), len(e2sNodes)
        e2sCheck = s2eNew is not None and e2sIDX < e2sLEN
        s2eCheck = e2sNew is not None and s2eIDX < s2eLEN
        done = (not s2eCheck) and (not e2sCheck)
        while not done:
            if e2sCheck:
                e2sNode = e2sNodes[e2sIDX]
                if s2eNew.distance(e2sNode) <= self._sqrdRes:
                    self._loginfo("Found path {start2end -> end2start} to endpoint")
                    result = self._composePath(e2sNode)
                    result.reverse()
                    return self._composePath(s2eNew, result), True
                e2sIDX += 1
                e2sCheck = e2sIDX < e2sLEN
            if s2eCheck:
                s2eNode = s2eNodes[s2eIDX]
                if e2sNew.distance(s2eNode) <= self._sqrdRes:
                    self._loginfo("Found path {end2start -> start2end} to endpoint ")
                    result = self._composePath(e2sNew)
                    result.reverse()
                    return self._composePath(s2eNode, result), True
                s2eIDX += 1
                s2eCheck = s2eIDX < s2eLEN
            done = (not s2eCheck) and (not e2sCheck)
        return path, found

    def _composePath(self, node, nodeList = []):
        while node.parent is not None:
            nodeList.insert(0, node)
            node = node.parent
        nodeList.insert(0, node)
        return nodeList
