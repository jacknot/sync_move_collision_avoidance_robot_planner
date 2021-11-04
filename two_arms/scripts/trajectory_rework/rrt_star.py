import math

from trajectory_rework.rrt import RRT

class RRTStar(RRT):
    def __init__(self,
                    searchSpace,
                    resolution = 0.1, 
                    maxIter = 500,
                    goalSampleRate = 7,
                    connectCircleDist = sum([math.pow(math.pi / float(2), 2) for _ in range(6)]), 
                    searchUntilMaxIter = False,
                    id=""
                ):
        super(RRTStar, self).__init__(searchSpace, resolution, maxIter, goalSampleRate, id = id)
        self.connectCircleDist = connectCircleDist
        self.searchUntilMaxIter = searchUntilMaxIter

    def _run(self):
        self._loginfo("Start planning ")
        self.nodes = [self.start]
        for i in range(self.maxIter):
            new = self.explore(i)

            if ((not self.searchUntilMaxIter) and new):  # if reaches goal
                lastIDX = self.searchBestGoalNode()
                if lastIDX is not None:
                    self._loginfo("Found path to end point ")
                    return self.composePath(lastIDX)

        self._loginfo("Reached max iteration ")
        lastIDX = self.searchBestGoalNode()
        if lastIDX is not None:
            return self.composePath(lastIDX)
        self._logwarn("No path found, run failed")
        return None

    def explore(self, i):
        rnd = self.newRandomNode(t = i)
        self._loginfo("[%d/%d] Rnd: %s" % (i+1, self.maxIter, rnd))
        nearest = self.getNearestNode(self.nodes, rnd)

        new, valid = self.steer(nearest, rnd, self.expandDist, self.expandJoints, self.expandDuration)
        new.cost = nearest.cost + new.distance(nearest)
        self._loginfo("[%d/%d] After steer: New: %s" % (i+1, self.maxIter, new))
        if valid:
            self._loginfo("[%d/%d] New: no collision found" % (i+1, self.maxIter))
            nearIDXs = self.findNearNodes(new)
            self._loginfo("[%d/%d] New: nearIDXs: %s" % (i+1, self.maxIter, nearIDXs))
            updatedParentNode = self.chooseParent(new, nearIDXs)
            self._loginfo("[%d/%d] New: updateParent: %s" % (i+1, self.maxIter, updatedParentNode))
            if updatedParentNode:
                self._loginfo("[%d/%d] Rewiring " % (i+1, self.maxIter))
                self.rewire(updatedParentNode, nearIDXs)
                self.nodes.append(updatedParentNode)
                self._loginfo("[%d/%d] New: updated: %s " % (i+1, self.maxIter, updatedParentNode))
                return updatedParentNode
            else:
                self.nodes.append(new)
                return new
        return None

    def chooseParent(self, newNode, nearIDXs):
        """
        Computes the cheapest point to newNode contained in the list
        nearIDXs and set such a node as the parent of newNode.
            Arguments:
            --------
                newNode, Node
                    randomly generated node with a path from its neared point
                    There are not coalitions between this node and th tree.
                nearIDXs: list
                    Indices of indices of the nodes what are near to newNode
            Returns.
            ------
                Node, a copy of newNode
        """
        if not nearIDXs:
            return None
        # search nearest cost in nearIDXs
        #minIDX = -1
        minT = None
        minCost = float('inf')
        for i in nearIDXs:
            near = self.nodes[i]
            t, valid = self.steer(near, newNode, self.expandDist, self.expandJoints, self.expandDuration)
            if t and valid:
                cost = self.computeCost(near, newNode)
                if cost < minCost:
                    minT, minCost = t, cost

        if minCost == float('inf'):
            self._logwarn("There is no good path, can't choose parent ")
            return None
        minT.cost = minCost
        return minT

    def searchBestGoalNode(self):
        """
            Search for the new best safe (valid) goal index
        """
        minIDX, minCost = -1, float('inf')
        for i, n in enumerate(self.nodes):
            if self.dist2goal(n) <= self.expandDist:
                _, valid = self.steer(self.nodes[i], self.end, self.expandDist, self.expandJoints, self.expandDuration)
                if valid:
                    if self.nodes[i].cost <= minCost:
                        minIDX, minCost = i, self.nodes[i].cost
        if minIDX < 0:
            return None
        return minIDX

    def findNearNodes(self, newNode):
        """
        1) defines a ball centered on newNode
        2) Returns all nodes of the three that are inside this ball
            Arguments:
            ---------
                newNode: Node
                    new randomly generated node, without collisions between
                    its nearest node
            Returns:
            -------
                list
                    List with the indices of the nodes inside the ball of
                    radius r
        """
        nnode = len(self.nodes) + 1
        r = min(self.expandDist, math.pow(self.connectCircleDist, 2)) 
        r *= math.sqrt(math.log(nnode) / float(nnode))
        # if expand_dist exists, search vertices in a range no more than
        # expand_dist
        #self._loginfo("Expand Dist: %f, sqrt; %f" % (self.expandDist, math.sqrt(self.expandDist)))
        self._logdebug("R: %f, pow(R, 2): %f" % (r, math.pow(r, 2)))
        #if hasattr(self, 'expandDist'):
        #    r = min(math.pow(r, 2), self.expandDist)
        return [i for i, node in enumerate(self.nodes) if node.distance(newNode) <= r]

    def rewire(self, newNode, nearIDXs):
        """
            For each node in nearIDXs, this will check if it is cheaper to
            arrive to them from newNode.
            In such a case, this will re-assign the parent of the nodes in
            nearIDXs to newNode.
            Parameters:
            ----------
                newNode, Node
                    Node randomly added which can be joined to the tree
                nearIDXs, list of uints
                    A list of indices of the self.newNode which contains
                    nodes within a circle of a given radius.
            Remark: parent is designated in chooseParent.
        """
        for i in nearIDXs:
            nearNode = self.nodes[i]
            edgeNode, valid = self.steer(newNode, nearNode, self.expandDist, self.expandJoints, self.expandDuration)
            if not edgeNode:
                continue
            edgeNode.cost = self.computeCost(newNode, nearNode)

            if valid and edgeNode.cost < nearNode.cost:
                nearNode.config = edgeNode.config
                nearNode.path = edgeNode.path
                nearNode.cost = edgeNode.cost
                nearNode.timeFromStart = edgeNode.timeFromStart
                nearNode.parent = edgeNode.parent
                self.propagateCostToLeaves(newNode)

    def computeCost(self, fromNode, toNode):
        return fromNode.cost + fromNode.distance(toNode)

    def propagateCostToLeaves(self, parentNode):
        for node in self.nodes:
            if node.parent == parentNode:
                node.cost = self.computeCost(parentNode, node)
                self.propagateCostToLeaves(node)               
