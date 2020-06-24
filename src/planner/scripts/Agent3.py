#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from planner.srv import *


class Agent:
    def __init__(self):
        self.name = 'ag3'
        resp = self.send_request("requirements_name: %s" % self.name)
        if resp == 'Ready':
            print('%s is Ready!' % self.name)
        else:
            print('%s not in the Task requirements!' % self.name)

    def send_request(self, req):
        rospy.wait_for_service('manager_service')
        try:
            request_manager = rospy.ServiceProxy('manager_service', AddRobotMessage)
            resp1 = request_manager(req)
            return resp1.response
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


if __name__ == "__main__":
    ag = Agent()