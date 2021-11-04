from enum import Enum

class ActionStatus(Enum):
    pending = (0, False, False)     # The goal has yet to be processed by the action server
    active = (1, True, False)       # The goal is currently being processed by the action server
    preempted = (2, False, True)    # The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
    succeeded = (3, False, True)    # The goal was achieved successfully by the action server (Terminal State)
    aborted = (4, False, True)      # The goal was aborted during execution by the action server due to some failure (Terminal State)
    rejected = (5, False, True)     # The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
    preempting = (6, True, False)   # The goal received a cancel request after it started executing and has not yet completed execution
    recalling = (7, True, False)    # The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
    recalled = (8, False, True)     # The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
    lost = (9, False, True)         # An action client can determine that a goal is LOST. This should not be sent over the wire by an action server (Not tracking a goal)
    def __init__(self, code, moving, terminal):
        self._code = code
        self._moving = moving
        self._terminal = terminal
    @property
    def code(self):
        return self._code
    @property
    def moving(self):
        return self._moving
    @property
    def terminal(self):
        return self._terminal
    def __int__(self):
        return self._code
