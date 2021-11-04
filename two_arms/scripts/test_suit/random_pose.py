#!/usr/bin/env python
import rospy

from joints_to_cartesian_srv import JtcSrv
from robot_state.state_checker import StateChecker

from random import uniform, randrange
from math import pi

def generateJoints():
	joints_states = list()
	joints_states.append(uniform(-pi, pi))
	joints_states.append(uniform(-pi, pi))
	joints_states.append(uniform(-pi, pi))
	joints_states.append(uniform(-pi, pi))
	joints_states.append(uniform(-pi, pi))
	joints_states.append(uniform(-pi, pi))
	return joints_states

def randomPoses():
    jtc = JtcSrv()
    sc = StateChecker()

    while True:
        leftState = generateJoints()
        rightState = generateJoints()
        if sc.check(leftState + rightState, "dual_arm"):
            break 

    left = jtc.jointsToCartesian(leftState, "rleft")
    right = jtc.jointsToCartesian(rightState, "rright")

    return left, right
