#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from planner.srv import *
import pickle

class RoboAgent:
    def __init__(self):
        self.name = 'ag1'
        resp = self.send_request("requirements_name: %s" % self.name)
        if resp != 'Sleep':
            print('%s is Ready!' % self.name)
            agent_info = pickle.loads(resp.encode())
            self.problem = Problem(agent_info[0])
            print(self.problem)
            print(agent_info[1])
        else:
            print('%s not in the Task requirements!' % self.name)

    def send_request(self, req):
        rospy.wait_for_service('manager_service')
        try:
            request_manager = rospy.ServiceProxy('manager_service', AddRobotMessage)
            resp1 = request_manager(req)
            return resp1.response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

class Problem:
    def __init__(self, dictionary):
        self.name = None
        self.__dict__.update(dictionary)

    def __str__(self):
        return self.name

    def __repr__(self):
        return self.name

if __name__ == "__main__":
    ag = RoboAgent()