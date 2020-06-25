#!/usr/bin/env python

from __future__ import print_function

import time

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
        self.solution = None
        self.subtasks = {}
        self.solved = {}
        rospy.init_node('Manager')
        agent_choose = rospy.Service('manager_service', AddRobotMessage, self.plan_synthesis)
        print('Agents: {0}'.format(self.agents))
        print("Ready to handle robot request.")
        rospy.spin()


    def plan_synthesis(self, req):
        #print("Request was %s" % req.request)
        request = req.request
        if 'requirements_name:' in request:
            print('Request was ' + request)
            name = request.split(': ')[-1]
            if name in self.agents:
                agent_info = pickle.dumps(
                    [self.problem, self.agpath, self.agtype, self.agents, self.backward, self.subsearch], 0).decode()
                return AddRobotMessageResponse(agent_info)
            else:
                return AddRobotMessageResponse('Sleep')
        elif 'requirements_major:' in request:
            print('Request was ' + request)
            name = re.findall('name:\w*', request)[0].split(':')[1]
            new_signs = eval(re.findall('new_signs:\w*', request)[0].split(':')[1])
            self.major_search[name] = new_signs
            while not len(self.major_search) == len(self.agents):
                time.sleep(1)
            else:
                max_exp = 0
                for new_signs in self.major_search.values():
                    if new_signs > max_exp: max_exp = new_signs
                major = [name for name, new_signs in self.major_search.items() if new_signs == max_exp][0]
                print('major is {0} answered to {1}'.format(major, name))
                return AddRobotMessageResponse(major)
        elif 'requirements_solution:' in request:
            print('Request was requirements_solution')
            self.solution = re.split(': ', request)[1]
            return AddRobotMessageResponse('STOP')
        elif 'requirements_wait_sol:' in request:
            print('Request was ' + request)
            ag_name = re.split(': ', request)[1]
            if self.solution:
                return AddRobotMessageResponse('STOP: %s' % self.solution)
            while not ag_name in self.subtasks or self.solution:
                time.sleep(1)
            else:
                if ag_name in self.subtasks:
                    subtask = self.subtasks[ag_name]
                    self.subtasks.pop(ag_name)
                    return AddRobotMessageResponse(subtask)
                elif self.solution:
                    return AddRobotMessageResponse('STOP: %s' % self.solution)
        elif 'requirements_sub:' in request:
            print('Request was requirements_sub')
            req = re.split(': ', request)[1]
            subtask = pickle.loads(req.encode())
            print('act_agent is %s' % subtask[0])
            print('sub is %s' % len(subtask[1]))
            print('map is %s' % len(subtask[2]))
            self.subtasks[subtask[0]] = req
            while not subtask[0] in self.solved:
                time.sleep(1)
            else:
                solv = self.solved[subtask[0]]
                self.solved.pop(subtask[0])
                return AddRobotMessageResponse(pickle.dumps(solv, 0).decode())
        elif 'requirements_solved:' in request:
            print('Request was requirements_solved')
            req = re.split(': ', request)[1]
            subtask = pickle.loads(req.encode())
            self.solved[subtask[0]] = subtask[1]
            return AddRobotMessageResponse('saved')

class Problem:
    def __init__(self, dictionary):
        self.__dict__.update(dictionary)