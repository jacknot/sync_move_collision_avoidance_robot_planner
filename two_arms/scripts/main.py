#!/usr/bin/env python
import sys

import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from utils import checkPlanningSceneVersion
from utils.logging import Logger
from commanders.dual_commander import DualCommander

class Main(object):
    def __init__(self):
        self.__init_log()
        self.__init()
         
        self._loginfo("Adding underground box")
        self.__init_ground()

        self._loginfo("DualCommander started, ready to receive Poses")

    def __init_log(self):
        Logger.setFileNameToNow()
        #Logger.setLevel(Level.warning)
        self._logger = Logger(id='main')
        self._logger.timestamp()

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

    def _loginfo(self, string):
        self._logger.info(string)
        
    def _logwarn(self, string):
        self._logger.warn(string)

def main():
    try:
        m = Main()
        m.dualCommander.run()
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
