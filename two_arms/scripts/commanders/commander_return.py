from enum import Enum

class RoutineResult(Enum):
    success = (0, -1)
    timeout = (-1, 0)
    defaultFailure = (-2, 1)
    reworkerFailure = (-3, 1)
    outOfRetry = (-4, 2)
    loweringPriority = (-5, 3)
    def __init__(self, code, severity):
        self.code = code
        self.severity = severity