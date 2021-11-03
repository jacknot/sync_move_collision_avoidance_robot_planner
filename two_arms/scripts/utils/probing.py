import time

import rospy

class Chronometer(object):
    def __init__(self):
        self._t0 = 0
        self._T = 0
        self._duration = None
    
    def start(self):
        self._T = 0
        self._duration = None
        self._t0 = time.time()
    
    def stop(self):
        self._T = time.time()

    def reset(self):
        self._t0 = 0
        self._T = 0
        self._duration = None

    @property
    def duration(self):
        if self._duration is None:
            self._duration = rospy.Duration(secs=self._T - self._t0)
        return self._duration
