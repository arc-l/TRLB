from math import sqrt
from copy import deepcopy
import os
import sys

sys.path.append(os.path.abspath(os.path.dirname(__file__)))
from tools.util import deepcopy_dict


class partial_fmRS(object):
    '''
    Input: 
    start_poses: dict[obj_id]=(x,y)
    goal_poses: dict[obj_id]=(x,y)
    radius

    Output:
    is_monotone: bool partial=goal
    is_frozen bool: partial=start
    partial_plan
    '''
    def __init__(self, start_poses, goal_poses, radius):
        self.is_monotone = False
        self.num_collision = 0
        self.is_frozen = False
        self.partial_plan = []
        self.obstacle_set = set() # fixed disk centers
        self.radius = radius
        self.partial_arr = deepcopy_dict(start_poses)
        self.start_poses = {}
        self.goal_poses = {}
        for i in start_poses.keys():
            if start_poses[i] == goal_poses[i]:
                self.obstacle_set.add(tuple(start_poses[i]))
            else:
                self.start_poses[i] = start_poses[i]
                self.goal_poses[i] = goal_poses[i]

        self.parent = {}
        self.ordering = []

        # get a partial plan
        self.partial_plan = self.move_objects()

        # check monotonicity
        if (len(self.partial_plan) == len(self.start_poses)):
            self.is_monotone = True
        else:
            self.is_monotone = False

        # check frozen or not
        if (len(self.partial_plan) == 0):
            self.is_frozen = True
        else:
            self.is_frozen = False

        # construct the current arrangement
        self.construct_partial_arr()

    def construct_partial_arr(self):
        for obj in self.partial_plan:
            self.partial_arr[obj] = self.goal_poses[obj]


    def move_objects(self):
        # construct DG[obj_id] = set()
        self.DG = {} 
        self.construct_DG()

        removed_obj_list = []
        GO_ON = True
        while(GO_ON):
            new_obj_list = []
            for i in self.DG.keys():
                # sth that can directly move
                if( len(self.DG[i])==0):
                    new_obj_list.append(i)
            for i in new_obj_list:
                del self.DG[i]
            if len(new_obj_list)==0:
                GO_ON = False
            else:
                removed_obj_list = removed_obj_list + new_obj_list
                new_obj_set = set(new_obj_list)
                for key in self.DG.keys():
                    self.DG[key] = self.DG[key].difference(new_obj_set)
        return removed_obj_list

    def construct_DG(self):
        self.DG = {}
        for goal_obj, goal_center in self.goal_poses.items():
            self.DG[goal_obj] = set()
            for start_obj, start_center in self.start_poses.items():
                if start_obj == goal_obj:
                    continue
                self.num_collision += 1
                if (sqrt((start_center[0]-goal_center[0])**2+
                (start_center[1]-goal_center[1])**2) <= 2*self.radius):
                    self.DG[goal_obj].add(start_obj)
