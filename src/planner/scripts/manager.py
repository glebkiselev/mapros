#!/usr/bin/env python

from __future__ import print_function

from planner.srv import AddRobotMessage, AddRobotMessageResponse
import rospy
import pickle

class Manager:
    def __init__(self, problem, agpath = 'mapspatial.agent.planning_agent', TaskType = 'spatial', backward = False, subsearch = 'greedy'):
        self.agents = problem['agents']
        self.problem = problem
        self.agpath = agpath
        self.agtype = 'SpAgent'
        self.backward = backward
        self.subsearch = subsearch
        self.TaskType = TaskType
        self.finish = False
        rospy.init_node('Manager')
        s = rospy.Service('manager_service', AddRobotMessage, self.handle_robot_request)
        print('Agents: {0}'.format(self.agents))
        print("Ready to handle robot request.")
        rospy.spin()


    def handle_robot_request(self, req):
        print("Request was %s" % req.request)
        request = req.request
        if 'requirements_name:' in request:
            name = request.split(' ')[-1]
            if name in self.agents:
                #agent_info = pickle.dumps([self.agpath, self.agtype, self.agents, self.problem, self.backward, self.subsearch], 0).decode()
                agent_info = pickle.dumps(
                    [self.problem, self.agpath, self.agtype, self.agents, self.backward, self.subsearch], 0).decode()
                return AddRobotMessageResponse(agent_info)
            else:
                return AddRobotMessageResponse('Sleep')
    def get_solution(self):
        while not self.finish:
           pass
        return 'finished'

class Problem:
    def __init__(self, dictionary):
        self.__dict__.update(dictionary)