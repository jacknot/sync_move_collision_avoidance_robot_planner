from enum import IntEnum
import os
from datetime import datetime

import rospy

LOG_FORMAT = "%s:%s: %s"
ROS_FORMAT = "[%s] %s"
LOG_DIR = "../logs"
LOG_FILE_FORMAT = "%s.log"
DATETIME_FORMAT = "%Y-%m-%d_%H-%M-%S.%f"

class Level(IntEnum):
    MAX = 3

    debug = 0,
    info = 1,
    warn = 2,
    error = 3       

class Logger(object):

    logDir = LOG_DIR
    logFile = LOG_FILE_FORMAT % 'log'
    logFile = "%s/%s" % (logDir, logFile)
    _level = Level.info
    
    def __init__(self, id = '', format = LOG_FORMAT, level = Level.info):
        self._id = id
        self._format = format       
        Logger._level = level

        if not os.path.exists(Logger.logDir):
            os.makedirs(Logger.logDir)

        self._error = self._fileAndConsole(rospy.logerr, "ERROR")  
        self._warn = self._null
        self._info = self._null   
        self._debug = self._null       
        
	
        if level < Level.MAX:
            self._warn = self._fileAndConsole(rospy.logwarn, "WARN")  
            if level < Level.MAX - 1:
                self._info = self._fileAndConsole(rospy.loginfo, "INFO")  
                if level < Level.MAX - 2:
                    self._debug = self._fileAndConsole(rospy.logdebug, "DEBUG")          

    def _null(self, string):
        pass

    @staticmethod
    def setFileName(name):
        Logger.logFile = LOG_FILE_FORMAT % name
        Logger.logFile = "%s/%s" % (Logger.logDir, Logger.logFile)
        
    @staticmethod
    def setFileNameToNow():
        Logger.logFile = LOG_FILE_FORMAT % datetime.now().strftime(DATETIME_FORMAT)
        Logger.logFile = "%s/%s" % (Logger.logDir, Logger.logFile)
        
    @staticmethod
    def setFileNameToCurrent():
        logList = os.listdir(Logger.logDir)
        logList.sort()
        Logger.logFile = "%s/%s" % (Logger.logDir, logList[-1])       
        
    def _fileAndConsole(self, rospy_log, level):  
        def _genFun(string):
            rospy_log(ROS_FORMAT % (self._id, string))
            with open(Logger.logFile, 'a') as f:
                f.write(self._format % (level, self._id, string) + '\n')
        return _genFun
        
    def timestamp(self):
        with open(Logger.logFile, 'a') as f:
           f.write('\n' + "="*12 + "%s" % datetime.now() + "="*12 + '\n')

    def info(self, string):
        self._info(string)
    
    def warn(self, string):
        self._warn(string)
        
    def debug(self, string):
        self._debug(string)
        
    def error(self, string):
        self._error(string)
