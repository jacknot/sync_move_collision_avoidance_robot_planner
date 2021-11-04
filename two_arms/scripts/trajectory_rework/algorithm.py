import rospy

from utils.logging import Logger
from utils.math_functions import ExponentialMovingAverage
from utils.probing import Chronometer

class Algorithm(object):
    def __init__(self):
        self._logger = Logger(id = self.__class__.__name__)
        self._timer = Chronometer()
        self._avgDuration = ExponentialMovingAverage(mean = rospy.Duration(secs = 3))
    
    @property
    def avgDuration(self):
        return self._avgDuration.value

    def run(self):
        self._timer.start()
        result = self._run()
        self._timer.stop()
        self._avgDuration.updateState(self._timer.duration)
        return result
    
    def _run(self):
        raise NotImplementedError("Algorithm is abstract")
    
    def _loginfo(self, string):
        self._logger.info(string)
    
    def _logwarn(self, string):
        self._logger.warn(string)
    
    def _logdebug(self, string):
        self._logger.debug(string)