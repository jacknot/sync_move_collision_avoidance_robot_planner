from enum import Enum

class TrajectoryResult(Enum):
    successful = (0, "Successful")
    goal_tolerance_violated = (-5, "Goal tolerance violated")
    path_tolerance_violated = (-4, "Path tolerance violated")
    old_header_timestamp = (-3, "Old header timestamp")
    invalid_joints = (-2, "Invalid joints")
    invalid_goal = (-1, "Invalid goal")
    
    def __init__(self, code, text):
        self._code = code
        self._text = text
    @property
    def code(self):
        return self._code
    @property
    def text(self):
        return self._text      
    @staticmethod 
    def get(index):
        if index not in range(0, len(TrajectoryResult)*(-1), -1):
            return None
            
        listTR = list(TrajectoryResult)      
        return listTR[index + len(listTR) - 1]
    def __int__(self):
        return self._code
