#!/usr/bin/env python

import rospy
import moveit_commander

import sys
import os
import time

from csv import reader

parentDir = os.path.dirname(os.path.dirname(__file__))
two_arms_pkg = os.path.join(os.path.dirname(parentDir), "two_arms/scripts")
sys.path.append(two_arms_pkg)

from robot_state.state_checker import StateChecker

stateChecker = StateChecker(wlogger=False)

sys.path.append(parentDir)
os.chdir(parentDir)

class StateCheckBenchmarkPy(object):
    def __init__(self):
        self._sc = stateChecker
        self._milestone = 5000

    def run(self):
        bench_string = []
        count = 0
	
        with open('joints_list/joints_list.csv', 'r') as read_obj:  
            joints_list = reader(read_obj)
            for row in joints_list:
                if count % self._milestone == 0:
                    print("[%d%%] Iteration %d out of 100.000" % (count / 1000, count))
                    
                float_row = [float(i) for i in row]
                t0 = time.time()
                valid = self._sc.check(float_row, 'dual_arm')
                t1 = time.time() - t0
                bench_string.append(";".join(row) + (";%d" % valid) + (";%f" % t1))
                count = count + 1
	
	print("[100%] Done!")	
        if not os.path.exists("results"):
            os.makedirs("results")

        out = open('results/python_benchmark.csv','w+')
	out.write("\n".join(bench_string))
	out.close()

def main():
    try:
        scb = StateCheckBenchmarkPy()
        scb.run()
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return
    except EOFError:
        print("Script has been terminated")
        return
    finally:
        moveit_commander.roscpp_shutdown()


if __name__ == '__main__':
    main()
