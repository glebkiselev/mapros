#!/usr/bin/env python

from __future__ import print_function

from planner.srv import AddRobotMessage, AddRobotMessageResponse
import rospy
import pickle
import re

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
        self.major_search = {}
        rospy.init_node('Manager')
        agent_choose = rospy.Service('manager_service', AddRobotMessage, self.plan_synthesis)
        print('Agents: {0}'.format(self.agents))
        print("Ready to handle robot request.")
        rospy.spin()


    def plan_synthesis(self, req):
        print("Request was %s" % req.request)
        request = req.request
        if 'requirements_name:' in request:
            name = request.split(' ')[-1]
            if name in self.agents:
                agent_info = pickle.dumps(
                    [self.problem, self.agpath, self.agtype, self.agents, self.backward, self.subsearch], 0).decode()
                return AddRobotMessageResponse(agent_info)
            else:
                return AddRobotMessageResponse('Sleep')
        elif 'requirements_major:' in request:
            name = re.findall('name:\w*', request)[0].split(':')[1]
            new_signs = eval(re.findall('new_signs:\w*', request)[0].split(':')[1])
            self.major_search[name] = new_signs
            if len(self.major_search) == len(self.agents):
                max_exp = 0
                for new_signs in self.major_search.values():
                    if new_signs > max_exp: max_exp = new_signs
                major = [name for name, new_signs in self.major_search.items() if new_signs == max_exp][0]
                print('major is %s' % major)
                return AddRobotMessageResponse(major)




    # def choose_major(self, req):
    #     print("Request was %s" % req.request)
    #     request = req.request
    #     if 'requirements_name:' in request:
    #         name = request.split(' ')[-1]
    #         if name in self.agents:
    #             agent_info = pickle.dumps(
    #                 [self.problem, self.agpath, self.agtype, self.agents, self.backward, self.subsearch], 0).decode()
    #             return AddRobotMessageResponse(agent_info)
    #         else:
    #             return AddRobotMessageResponse('Sleep')


    def get_solution(self):
        while not self.finish:
           pass
        return 'finished'

class Problem:
    def __init__(self, dictionary):
        self.__dict__.update(dictionary)