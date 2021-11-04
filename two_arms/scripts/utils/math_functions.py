import math

import rospy

linInt = lambda a, b, l: a*(1 - l) + b*l
invLinInt = lambda a, b, c_l: (c_l - a) / float(b - a)
vecLinInt = lambda A, B, l: [linInt(a, b, l) for a, b in zip(A, B)]
def timeLinInt(ta, tb, l):
    """
        @params:
            ta: rospy.Duration | rospy.Time
            tb: rospy.Duration | rospy.Time
        @return:
            rospy.Duration
    """
    return rospy.Duration( nsecs = linInt(ta.to_nsec(), tb.to_nsec(), l) )

sawtooth = lambda x: x - math.floor(x)
angleSawtooth = lambda x: 2*math.pi*sawtooth(x / float(2*math.pi) - 0.5) - math.pi
def signedAngleDiff(source, target):
    """
        @params:
            source: in [-pi, pi]
            target: in [-pi, pi]
        @return:
            angle: in [-pi, pi], + if anticlockwise, - if clockwise
    """
    a = target - source
    a -= 2*math.pi if a > math.pi else 0
    a += 2*math.pi if a < -math.pi else 0
    return a
angleLinInt = lambda source, target, l: angleSawtooth(source + l*signedAngleDiff(source, target))
vecAngleLinInt = lambda source, target, l: [angleLinInt(s, t, l) for s, t in zip(source, target)]

class CumulativeMovingAverage(object):
    def __init__(self, mean = 0, n = 0):
        self.mean = mean
        self.n = n
    
    @property
    def value(self):
        return self.mean
    
    def updateState(self, value):
        self.n += 1
        self.mean = self.mean + (value - self.mean) / float(self.n)

class ExponentialMovingAverage(CumulativeMovingAverage):
    def __init__(self, mean = 0, n = 0, alpha = lambda x: 2/float(x + 1)):
        """
            @Params:
                alpha: function to compute alpha given N_previous
        """
        super(ExponentialMovingAverage, self).__init__(mean=mean, n=n)
        self._alpha = alpha
    
    @property
    def alpha(self):
        return self._alpha(self.n)

    def updateState(self, value):
        self.n += 1
        a = self.alpha
        self.mean = a*value + (1 - a)*self.mean
        