#!/usr/bin/env python3

from __future__ import print_function

import importlib
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
            self.agpath = agent_info[1]
            self.agtype = agent_info[2]
            self.agents = agent_info[3]
            self.backward = agent_info[4]
            self.subsearch=agent_info[5]
            self.solution = self.search_solution()
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

    def search_solution(self):
        # init agent
        class_ = getattr(importlib.import_module(self.agpath), self.agtype)
        workman = class_()
        workman.initialize(self.name, self.agents, self.problem, self.backward, self.subsearch)
        task, new_signs = workman.get_task()
        print('Amount of new signs: {0}'.format(new_signs))
        resp = self.send_request("requirements_major: name:{0} new_signs:{1}".format(self.name, str(new_signs)))
        if resp == self.name:
            print('%s major' % self.name)
        else:
            print('%s is not major' % self.name)



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