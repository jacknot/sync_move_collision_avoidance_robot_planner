#!/usr/bin/env python
import rospy
from rosgraph_msgs.msg import Clock as ClockMsg

class Clock(object):
    def __init__(self, ):
        # Sub to clock
        self._clockSub = rospy.Subscriber("/clock", ClockMsg, self._callbackClock)
        self._clockRO = rospy.Time()
        
    def _callbackClock(self, data):
        self._clockRO = data.clock
    
    @property
    def value(self):
        return self._clockRO
        