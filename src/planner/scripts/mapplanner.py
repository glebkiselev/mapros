#!/usr/bin/env python3

import logging
import json
import argparse, os, sys
from config_master import create_config, get_config
from mapcore.planning.mapplanner import MapPlanner as MPcore
from manager import Manager
from mapspatial.parsers.spatial_parser import Problem

SOLUTION_FILE_SUFFIX = '.soln'

import platform

if platform.system() != 'Windows':
    delim = '/'
else:
    delim = '\\'

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("process-main")



class MapPlanner(MPcore):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.subsearch = kwargs['Settings']['subsearch']

    def find_domain(self, domain, path, number):
        """
        Domain search function
        :param path: path to current task
        :param number: task number
        :return:
        """
        if 'spatial' in self.TaskType:
            ext = '.json'
            path += 'task' + number + delim
        elif self.TaskType == 'htn':
            ext = '.hddl'
        else:
            ext = '.pddl'
        task = 'task' + number + ext
        domain = 'domain' + ext
        if not domain in os.listdir(path):
            domain2 = self.search_upper(path, domain)
            if not domain2:
                raise Exception('domain not found!')
            else:
                domain = domain2
        else:
            domain = path + domain
        if not task in os.listdir(path):
            raise Exception('task not found!')
        else:
            problem = path + task

        return domain, problem


    def _parse_spatial(self):
        """
        spatial Parser
        :param domain_file:
        :param problem_file:
        :return:
        """
        logging.debug('Распознаю проблему {0}'.format(self.problem))
        with open(self.problem) as data_file1:
            problem_parsed = json.load(data_file1)
        logging.debug('Распознаю домен {0}'.format(self.domain))
        with open(self.domain) as data_file2:
            signs_structure = json.load(data_file2)

        logging.debug('{0} найдено объектов'.format(len(problem_parsed['global-start']['objects'])))
        logging.debug('{0} найдено предикатов'.format(len(signs_structure['predicates'])))
        logging.debug('{0} найдено действий'.format(len(signs_structure['actions'])))
        logging.info('Карта содержит {0} неперемещаемых препятствий'.format(len(problem_parsed['map']['wall'])))
        logging.info('Размер карты {0}:{1}'.format(problem_parsed['map']['map-size'][0], problem_parsed['map']['map-size'][1]))
        problem = Problem(signs_structure, problem_parsed, self.problem)
        # to send it with ROS
        return problem.__dict__


    def search(self):
        """
        spatial - json-pddl- based plan search
        :return: the final solution
        """
        problem = self._parse_spatial()
        logger.info('Пространственная проблема получена и распознана')
        manager = Manager(problem, self.agpath, TaskType=self.TaskType, backward=self.backward, subsearch = self.subsearch)
        #solution = manager.handle_robot_request
        solution = None
        return solution

if __name__ == "__main__":
    task_num = '1'
    task_type = 'spatial'
    if platform.system() != 'Windows':
        delim = '/'
    else:
        delim = '\\'
    argparser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    argparser.add_argument(dest='problem', nargs='?')
    argparser.add_argument(dest='agpath', nargs='?')
    argparser.add_argument(dest='agtype', nargs='?')
    argparser.add_argument(dest='backward', nargs='?')
    argparser.add_argument(dest='config_path', nargs='?')
    args = argparser.parse_args(sys.argv[1:])
    if args.problem and args.agpath and args.agtype:
        if not args.config_path:
            path = create_config(benchmark=os.path.abspath(args.problem), delim=delim,
                                 task_type=task_type, agpath=args.agpath, agtype=args.agtype, backward=args.backward)
        else:
            path = args.config_path
    else:
        if not args.config_path:
            path = create_config(task_num=task_num, delim=delim, backward='False', task_type=task_type)
        else:
            path = args.config_path

    # after 1 time creating config simply send a path
    planner = MapPlanner(**get_config(path))
    solution = planner.search()