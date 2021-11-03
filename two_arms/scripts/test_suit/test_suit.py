#!/usr/bin/env python
import sys
import os

import rospy
import moveit_commander

parentDir = os.path.dirname(os.path.dirname(__file__))
os.chdir(parentDir)
sys.path.append(parentDir)

from random_pose import randomPoses
from two_arms_msgs.msg import GoalPose
from utils.logging import Level, Logger

class TestSuit(object):
    def __init__(self, topic = '/two_arms/dual_commander/goal_pose'):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('goal_spawner', anonymous=True)

        Logger.setFileNameToCurrent()
        self._logger = Logger(id=self.__class__.__name__, level=Level.info)
        self.topic = topic
        self.pub = rospy.Publisher(self.topic, GoalPose, queue_size=10)
        self.suit = {           
            "Synchronized Move, no collisions": self.testSyncMove,
            "End Poses in Conflict": self.testEndPoseConflict,
            "Collision during Trajectory Execution": self.testConflictDuringTrajectory,
            "Random Poses": self.testRandomPoses,
            "SingleArm": self.testSingleArm
        }
        
        self.startString = "======= Press `Enter` to start (press ctrl-d to exit) ..."
        self.endString = "======= Press `Enter` to END test (press ctrl-d to exit) ..."

    def homePosition(self, arm="", priority=0):
        """Return the arms to the home position"""
        self._logger.info("##### Taking arms back to home position #########")
        left = GoalPose()
        left.pose.header.frame_id = '/world'
        left.pose.pose.position.x = -0.15535
        left.pose.pose.position.y = 0.19145
        left.pose.pose.position.z = 0.931409
        left.pose.pose.orientation.x = -0.5
        left.pose.pose.orientation.y =  0.5
        left.pose.pose.orientation.z = 0.5
        left.pose.pose.orientation.w = 0.5
        left.priority = priority
        left.assign_to = "left_arm"

        right = GoalPose()
        right.pose.header.frame_id = '/world'
        right.pose.pose.position.x = 0.34465
        right.pose.pose.position.y = 0.19145
        right.pose.pose.position.z = 0.931409
        right.pose.pose.orientation.x = -0.5
        right.pose.pose.orientation.y = 0.5
        right.pose.pose.orientation.z = 0.5
        right.pose.pose.orientation.w = 0.5
        right.priority = priority
        right.assign_to = "right_arm"
        
        if arm == "left":
            self.pub.publish(left)
        elif arm == "right":
            self.pub.publish(right)
        
        if arm == "":
            self.pub.publish(left)
            self.pub.publish(right)

    def testSingleArm(self, priority=1, home_priority=0):
        """Test: Move a single arm"""
        print("Test: Single Arm")
        print self.startString
        raw_input()
        self._logger.info("##### Test: SingleArm #########")
        # send poses
        left = GoalPose()
        left.pose.header.frame_id = '/world'
        left.pose.pose.position.x = 0.236538923956
        left.pose.pose.position.y = -0.159493007015
        left.pose.pose.position.z = 0.785985721476
        left.pose.pose.orientation.x = -0.130346570758
        left.pose.pose.orientation.y =  0.694989044153
        left.pose.pose.orientation.z = 0.447585374313
        left.pose.pose.orientation.w = 0.547418790964
        left.assign_to = "left_arm"
        left.priority = priority

        self.pub.publish(left)
        print self.endString
        raw_input()
        self.homePosition(arm="left", priority=home_priority)

    def testSyncMove(self, priority=1, home_priority=0):
        """Test: synchronous movement of the arms, no collisions"""
        print("Test: SyncMove, no collisions")
        print self.startString
        raw_input()
        self._logger.info("##### Test: synchronized movement, no collisions #########")
        # send poses
        left = GoalPose()
        left.pose.header.frame_id = '/world'
        left.pose.pose.position.x = 0.236538923956
        left.pose.pose.position.y = -0.159493007015
        left.pose.pose.position.z = 0.785985721476
        left.pose.pose.orientation.x = -0.130346570758
        left.pose.pose.orientation.y =  0.694989044153
        left.pose.pose.orientation.z = 0.447585374313
        left.pose.pose.orientation.w = 0.547418790964
        left.assign_to = "left_arm"
        left.priority = priority

        right = GoalPose()
        right.pose.header.frame_id = '/world'
        right.pose.pose.position.x = -0.255030430118
        right.pose.pose.position.y = -0.432199623901
        right.pose.pose.position.z = 0.635366288183
        right.pose.pose.orientation.x = -0.690886645336
        right.pose.pose.orientation.y = -0.15058433947
        right.pose.pose.orientation.z = 0.5
        right.pose.pose.orientation.w = 0.5
        right.assign_to = "right_arm"
        right.priority = priority
        
        self.pub.publish(left)
        self.pub.publish(right)
        print self.endString
        raw_input()
        self.homePosition(priority=home_priority)
        
    def testEndPoseConflict(self, priority=1, home_priority=0):
        """Test: poses in input generate a state where the arms are in collision"""
        print("Test: EndPoseConflict")
        print self.startString
        raw_input()
        self._logger.info("##### Test: arms in collision in final states #########")
        # send poses
        left = GoalPose()
        left.pose.header.frame_id = '/world'
        left.pose.pose.position.x = 0.236538923956
        left.pose.pose.position.y = -0.159493007015
        left.pose.pose.position.z = 0.785985721476
        left.pose.pose.orientation.x = -0.130346570758
        left.pose.pose.orientation.y =  0.694989044153
        left.pose.pose.orientation.z = 0.447585374313
        left.pose.pose.orientation.w = 0.547418790964
        left.assign_to = "left_arm"
        left.priority = priority

        right = GoalPose()
        right.pose.header.frame_id = '/world'
        right.pose.pose.position.x = -0.109478460902
        right.pose.pose.position.y = -0.0524714553686
        right.pose.pose.position.z = 0.876740975934
        right.pose.pose.orientation.x = -0.694989044152
        right.pose.pose.orientation.y = 0.130346570757
        right.pose.pose.orientation.z = 0.547418790961
        right.pose.pose.orientation.w = 0.447585374319
        right.assign_to = "right_arm"
        right.priority = priority

        self.pub.publish(left)
        self.pub.publish(right)
        print self.endString
        raw_input()
        self.homePosition(priority=home_priority)

    def testConflictDuringTrajectory(self, priority=1, home_priority=0):
        """Test: synchronous movement of the arms, collisions during movement, but not in final state"""
        print("Test: CollisionDuringTrajectory")
        print self.startString
        raw_input()
        self._logger.info("##### Test: synchronous movement, collisions during movement (not in final state) #########")
        # send poses
        left = GoalPose()
        left.pose.header.frame_id = '/world'
        left.pose.pose.position.x = 0.220543287075
        left.pose.pose.position.y = 0.700011390808
        left.pose.pose.position.z = 0.159791830592
        left.pose.pose.orientation.x = 0.314170970281
        left.pose.pose.orientation.y =  -0.633479756137
        left.pose.pose.orientation.z = -0.677661196837
        left.pose.pose.orientation.w = 0.201928953594
        left.assign_to = "left_arm"
        left.priority = priority

        right = GoalPose()
        right.pose.header.frame_id = '/world'
        right.pose.pose.position.x = 0.0235709897741
        right.pose.pose.position.y = 0.472595394674
        right.pose.pose.position.z = 0.776648836724
        right.pose.pose.orientation.x = -0.5
        right.pose.pose.orientation.y = 0.5
        right.pose.pose.orientation.z = 0.0250250544223
        right.pose.pose.orientation.w = 0.70666381445
        right.assign_to = "right_arm"
        right.priority = priority
        
        self.pub.publish(left)
        self.pub.publish(right)
        print self.endString
        raw_input()
        self.homePosition(priority=home_priority)
        
    def testRandomPoses(self, priority=1, home_priority=0):
        """Test: synchronous movement of the arms, random poses"""
        print("Test: Random Poses")
        print self.startString
        raw_input()
        self._logger.info("##### Test: Random poses #########")
        # send poses
        
        left_pose, right_pose = randomPoses()

        left = GoalPose()
        left.pose.header.frame_id = '/world'
        left.pose = left_pose
        left.assign_to = "left_arm"
        left.priority = priority

        right = GoalPose()
        right.pose.header.frame_id = '/world'
        right.pose = right_pose
        right.assign_to = "right_arm"
        right.priority = priority
        
        print("**Generated LEFT:\n%s\n**Generated RIGHT:\n%s" % (left_pose.pose, right_pose.pose)) 
        
        self.pub.publish(left)
        self.pub.publish(right)
        print self.endString
        raw_input()
        self.homePosition(priority=home_priority)

    def completeSuit(self):
        self._logger.info("Starting complete Test Suit")
        priority=len(self.suit)*2-1
        for test in self.suit.values():
            test(priority=priority, home_priority=priority-1)
            priority = priority-2
            
    def _loginfo(self, string):
        self._logger.info(string)
    
    def _logwarn(self, string):
        self._logger.warn(string)

def main():
    test = TestSuit()
    try:
        i=-1
        testList = list(test.suit)
        N_SPECIAL_TESTS = 2
        while True:
            print "Tests:\n\t0 - Complete suit"
            print "\t1 - Starting position"
            for t in range(len(testList)):
                print "\t%d - %s" % (t+N_SPECIAL_TESTS, testList[t])
            i = raw_input("Press a key to run a test (ctrl-d to exit): ")
           
            try:
	        i = int(i)
            except ValueError:
               continue

            if i not in range(len(test.suit)+N_SPECIAL_TESTS):
               continue

            if i == 0:
               test.completeSuit()
            if i == 1:
               test.homePosition()
            else:
               choosenTest = test.suit[testList[i-N_SPECIAL_TESTS]]
               choosenTest()

    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    except EOFError:
        return
    finally:
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    main()
