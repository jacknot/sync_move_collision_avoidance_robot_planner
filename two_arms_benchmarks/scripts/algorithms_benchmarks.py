#!/usr/bin/env python
import sys
import os 
import time

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

parentDir = os.path.dirname(os.path.dirname(__file__))
two_arms_pkg = os.path.join(os.path.dirname(parentDir), "two_arms/scripts")
sys.path.append(two_arms_pkg)

from commanders.dual_commander import DualCommander

from utils import checkPlanningSceneVersion
from utils.logging import Logger
from utils.math_functions import ExponentialMovingAverage

from two_arms_msgs.msg import GoalPose

from trajectory_rework import Node
from trajectory_rework.rrt import RRT
from trajectory_rework.rrt_star import RRTStar
from trajectory_rework.bidirectional_rrt import BidirectionalRRT
from trajectory_rework.bidirectional_rrt_star import BidirectionalRRTStar

sys.path.append(parentDir)
os.chdir(parentDir)

class AlgorithmsBenchmark(object):
    def __init__(self):
        self.__init()
         
        self._loginfo("Adding underground box")
        self.__init_ground()

        self._loginfo("DualCommander started, ready to receive Poses")

    def __init(self):
        if len(sys.argv) == 1:
            ns = "two_arms"
        else:
            ns = sys.argv[1]       

        robot_desc_param = "robot_description"
        
        abs_robot_desc = "/%s/%s" % (ns, robot_desc_param)

        ## Initialize `moveit_commander` and a `rospy` node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('arms_commander', anonymous=False)

        ## Instantiate a RobotCommander object: outer-level interface to the robot
        self._robot = moveit_commander.RobotCommander(robot_description=abs_robot_desc, ns=ns)

        ## Instantiate a PlanningSceneInterface object: interface to the world surrounding the robot
        self._scene = moveit_commander.PlanningSceneInterface(ns=ns)

        checkPlanningSceneVersion(self._scene)

        ## Instantiate DualCommander
        self.dualCommander = DualCommander(ns=ns)

        ## Instantiate a DisplayTrajectory publisher, which is used later to publish trajectories for RViz to visualize:
        display_trajectory = rospy.Publisher(
            '/%s/move_group/display_planned_path' % ns, 
            moveit_msgs.msg.DisplayTrajectory, 
            queue_size=20
        )
        
        self._ground_box_name = "ground_box"       
        
	self.HALFZ_POSITION = [0.4, -1.2, 0.6, -1.8, 1.7, 1.5]

	fileName = "algorithmBenchmark.csv"
	fileFolder = "results"

	if not os.path.exists(fileFolder):
            os.makedirs(fileFolder)

	self.filePath = "%s/%s" % (fileFolder, fileName)

    

    def _wait_for_state_update(self, name, is_known=False, is_attached=False, timeout=4):
        """
        Ensuring Collision Updates Are Receieved
        If the Python node dies before publishing a collision object update message, the message could get lost and the box 	will not appear
        """
        start = rospy.get_time()
        seconds = rospy.get_time()

        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = self._scene.get_attached_objects([name])
            actual_is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            actual_is_known = name in self._scene.get_known_object_names()

            # Test if we are in the expected state
            if (actual_is_attached == is_attached) and (actual_is_known == is_known):
                return True

            # Waiting to spawn
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # Timed out
        self._logwarn("Box creation timed out: box not added to Planning Scene")
        return False

    def __init_ground(self, timeout=4):
        """Add ground to the planning scene"""
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self._robot.get_planning_frame()
        box_pose.pose.position.z = -0.025

        if self._ground_box_name not in self._scene.get_known_object_names():
            rospy.sleep(2)
            self._scene.add_box(self._ground_box_name, box_pose, size=(2, 2, 0.02))
            return self._wait_for_state_update(name=self._ground_box_name, is_known=True, timeout=timeout)  
        else:
            self._loginfo("Box already in the Planning scene, skipping")
            return True           
     
    def moveToHalfZ(self, group):        
        pose = group[1].currentPose
        goalPose = GoalPose()
        pose.pose.position.z = pose.pose.position.z / 2
        goalPose.pose = pose
        goalPose.assign_to = group[0]            
        self.dualCommander.dispatch((0, 5, goalPose))            
        

    def testAlgorithm(self, algorithm):   
              
        self.dualCommander.stateChecker.focusOn("right_arm")
        self.dualCommander.stateChecker.setTrajectories(
            dict(map(lambda t: (t[0], t[1].currentTrajectory), self.dualCommander.groups.items()))
        ) 
              
        observedJointDiff = self.dualCommander.trjReworker.observedJointDiff
        observedDuration = self.dualCommander.trjReworker.observedDuration
        
        self._loginfo("Setting the algorithm")
        algorithm.set(
                    start = Node([0]*6, rospy.Duration()),
                    goal = Node(self.HALFZ_POSITION, rospy.Duration()),
                    expandJoints = list(map(lambda x: x.value, observedJointDiff)),
                    expandDuration = observedDuration.value,
                    startStamp = rospy.Time(),
                    stateChecker = self.dualCommander.stateChecker,
                )
                
        self._loginfo("Running the algorithm")
        t0 = time.time()
        correction = algorithm.run()
        delta = time.time() - t0

        if correction is None:
            self._logwarn("Can't find a correction")
        else:
            self._loginfo("Found a correction")
        
        return correction, delta




    def _loginfo(self, string):
        rospy.loginfo("[AlgorithmsBenchmark] %s" % string)
        
    def _logwarn(self, string):
        rospy.logwarn("[AlgorithmsBenchmark] %s" % string)

def main():
    try:
        ab = AlgorithmsBenchmark()             

        ab.moveToHalfZ(ab.dualCommander.groups.items()[1])
        rospy.sleep(6)
        algorithms = {
                "RRT": RRT(ab.dualCommander.stateChecker.jointLimits()),
                "RRT*": RRTStar(ab.dualCommander.stateChecker.jointLimits()),
                "Bidirectional RRT": BidirectionalRRT(ab.dualCommander.stateChecker.jointLimits()),
                "Bidirectional RRT*": BidirectionalRRTStar(ab.dualCommander.stateChecker.jointLimits())
            }
            
        for name, algorithm in algorithms.items():
            ab._loginfo("Testing the algorithm: %s" % name)
            for i in range(5):		
                correction, delta = ab.testAlgorithm(algorithm)
		with open(ab.filePath, 'a+') as f:
		     f.write("%s,%d,%s,%f\n" % (name, i, len(correction) != 0, delta))
        
        
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    except EOFError:
        print("Script has been terminated")
        return
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
