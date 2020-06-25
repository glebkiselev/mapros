#!/usr/bin/env python3

from __future__ import print_function

import importlib
import logging
import os
import re
from copy import deepcopy, copy

import rospy
from planner.srv import *
import pickle

class RoboAgent:
    def __init__(self):
        self.name = 'ag2'
        resp = self.send_request("requirements_name: %s" % self.name)
        if resp != 'Sleep':
            print('%s is Ready!' % self.name)
            agent_info = pickle.loads(resp.encode())
            self.problem = Problem(agent_info[0])
            #print(self.problem.__dict__)
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

        # search scenario
        delim = '/'
        task_paths = self.problem.task_file.split(delim)[1:-1]
        path = ''.join([delim + el for el in task_paths])

        try:
            pddl_task = path + delim + 'scenario' + delim + task_paths[-1] + '.pddl'
            open(pddl_task)
        except FileNotFoundError:
            type = self.problem.name.split(' ')[0]
            pddl_task = os.getcwd() + delim + 'src' + delim + 'benchmarks' + delim + type + delim \
                        + task_paths[-2] + delim + task_paths[-1] + delim + 'scenario' + delim + task_paths[
                            -1] + '.pddl'

        # CALL mapplanner and get pddl solution.
        flag = True
        solutions = []
        self_solutions = []
        if resp == self.name:
            print('%s major' % self.name)
            map = {}
            subtasks = workman.get_scenario(pddl_task, task_paths[-2])
            for sub in subtasks:
                solution = {}
                self_sol = {}
                act_agent = sub[0][-1]
                if act_agent == self.name or act_agent == 'I':
                    if map:
                        workman.change_start(map, sub[0][1])
                    workman.load_subtask(sub)
                    subtask_solution, map = workman.search_solution()
                    if isinstance(subtask_solution[0], list):
                        subtask_solution = subtask_solution[0]
                    minor_message = []
                    for action in subtask_solution:
                        minor_message.append(
                            (None, action[1], None, None, (None, None), (None, None), deepcopy(action[6])))
                    solution[sub[0]] = subtask_solution
                    self_sol[sub[0]] = minor_message
                    solutions.append(self_sol)
                    self_solutions.append((self_sol, solution))
                else:
                    subpl = pickle.dumps((act_agent, sub, map), 0).decode()
                    ag_solution = self.send_request("requirements_sub: %s" %subpl)
                    ag_solution = pickle.loads(ag_solution.encode())
                    solution[sub[0]] = ag_solution
                    solutions.append(solution)
                    map = ag_solution[1]
            # send final solutions
            print('sending final solution by %s' % self.name)
            sol_to_send = pickle.dumps(solutions, 0).decode()
            server_answer = self.send_request("requirements_solution: {0}".format(sol_to_send))

        else:
            major_agent = resp
            while flag:
                req = self.send_request("requirements_wait_sol: %s" %self.name)
                if 'STOP:' in req:
                    subtask = re.split(': ', req)[1]
                    major_solutions = pickle.loads(subtask.encode())
                    if major_solutions:
                        major_agent_sign = workman.task.signs[major_agent]
                        for subplan in major_solutions:
                            solution = {}
                            for act_descr, ag_solution in subplan.items():
                                if act_descr[1] == 'I':
                                    act_descr_new = (act_descr[0], major_agent_sign.name)
                                elif act_descr[1] == self.name:
                                    act_descr_new = (act_descr[0], 'I')
                                else:
                                    act_descr_new = act_descr
                                solution[act_descr_new] = ag_solution
                                solutions.append(solution)
                        logging.info("Конечное решение получено агентом {0}".format(self.name))
                    else:
                        logging.debug('Агент {0} не смог получить решение от управляющего агента'.format(self.name))
                    flag = False
                else:
                    subtask = pickle.loads(req.encode())
                    if subtask[1]:
                        workman.change_start(subtask[1], subtask[0][0][1])
                    workman.load_subtask(subtask[0])
                    subtask_solution, map = workman.search_solution()
                    if isinstance(subtask_solution[0], list):
                        subtask_solution = subtask_solution[0]
                    solution = {}
                    pddl_name = (subtask[0][0][0], 'I')
                    solution[pddl_name] = subtask_solution
                    self_sol = {}
                    major_message = []
                    for action in subtask_solution:
                        major_message.append(
                            (None, action[1], None, None, (None, None), (None, None), deepcopy(action[6])))
                    self_sol[subtask[0][0]] = major_message
                    self_solutions.append((self_sol, solution))
                    if subtask_solution:
                        sol = pickle.dumps((self.name, major_message, map), 0).decode()
                        server_answer = self.send_request("requirements_solved: {0}".format(sol))
                    else:
                        logging.info("Агент {0} не смог синтезировать план".format(self.name))



        #
        # for ind, act1 in enumerate(copy(solutions)):
        #     for act1_name, act1_map in act1.items():
        #         for act2, self_act in self_solutions:
        #             flag = False
        #             for act2_name, act2_map in act2.items():
        #                 if act1_name[0] == act2_name[0]:
        #                     if act1_map == act2_map:
        #                         solutions[ind] = self_act
        #                         flag = True
        #                         break
        #             if flag:
        #                 break
        # file_name = workman.task.save_signs(solutions)
        file_name = True

        if file_name:
            logging.info('Агент ' + self.name + ' закончил работу')


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