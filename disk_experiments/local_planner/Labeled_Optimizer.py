from math import sqrt
import os
import sys
from numpy import array
from numpy.linalg import norm
from ujson import loads, dumps

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from primitive_plan.Labeled_DFS_DP import DFS_DP_Search
from primitive_plan.FVS_DP_Solver import FVS_Plan_Search
from primitive_plan.random_order import random_order
from buffer_allocation.Buffer_Generation import Buffer_Generation
from buffer_allocation.Buffer_Sampling import Buffer_Sampling
from buffer_allocation.Robust_Buffer_Sampling import Buffer_Sampling as Robust_Buffer_Sampling
from tools.util import deepcopy_dict



class One_Shot_Planner(object):
    '''
    Input:
    Initial_arrangement: dict[obj id]=(x,y)
    final_arrangement: dict[obj id]=(x,y)
    Environment Height
    Environment Width
    radius
    
    Output:
    The whole plan: [[objID,start_pose, goal_pose]]
    '''
    def __init__(self, start_arr, goal_arr, Height, Width, radius, sampling = False, primitive_plan = 'RB', robust_sampling = False):
        self.start_arr = deepcopy_dict(start_arr)
        self.goal_arr = deepcopy_dict(goal_arr)
        self.Height = Height
        self.Width = Width
        self.radius = radius
        self.sampling = sampling
        self.primitive_plan = primitive_plan
        self.robust_sampling = robust_sampling

        # Solution
        self.action_list = []
        self.num_collision = 0

        self.construct_DG()
        timeout = 100*len(self.start_arr)
        RB=0 # set the lowerbound of RB
        while 1:
            if self.primitive_plan == 'RB':
                _, action_sequence, _, _ = DFS_DP_Search(self.DG, RB=RB)
            elif self.primitive_plan == 'FVS':
                action_sequence = FVS_Plan_Search(self.DG)
            elif self.primitive_plan == 'random':
                action_sequence = random_order(self.DG)
            
            # obj_schedule, buffers = self.process_constraints(action_sequence)
            # isfeasible, buffer_locations = self.quadratic_optimization( obj_schedule, buffers)
            Problem_Partition = self.problem_partitioning( action_sequence)

            _, self.action_list, self.isfeasible = self.call_subproblems(action_sequence, Problem_Partition, 
            start_arr, goal_arr)


            if self.isfeasible:
                self.construct_plan()
                break
            else:
                timeout -= 1
                if timeout <=0:
                    break


    def problem_partitioning(self, action_sequence):
        Partitions = []
        current_buffers = []
        partial_action_sequence = []
        for action in action_sequence:
            if action[1]=='b': # inside a subproblem or going into a subproblem
                current_buffers.append(action[0])
                partial_action_sequence.append(action)
            elif (len(current_buffers)==0): # outside a problem
                continue
            else: # (len(current_buffers)>0) inside a problem or going out of a subp
                if action[0] in current_buffers:
                    current_buffers.remove(action[0])
                partial_action_sequence.append(action)
            if (len(current_buffers)==0) and (len(partial_action_sequence)>0): # just gone out a subp
                Partitions.append(partial_action_sequence)
                partial_action_sequence = []
        return Partitions


    def call_subproblems(self, action_sequence, Problem_Partition, start_arr, goal_arr):
        current_index = 0
        action_list = []
        isfeasible = True
        current_arr = deepcopy_dict(start_arr)
        if len(Problem_Partition) == 0:
            for action in action_sequence:
                objID = action[0]
                start_pose = start_arr[objID]
                goal_pose = goal_arr[objID]
                action_list.append([objID, start_pose, goal_pose])
            return goal_arr, action_list, isfeasible
        for subproblem in Problem_Partition:
            start_index = action_sequence.index(subproblem[0])
            end_index = action_sequence.index(subproblem[-1])

            while current_index < start_index: # move objects to the goal before solving the first subproblem
                objID = action_sequence[current_index][0]
                action_list.append([objID, start_arr[objID], goal_arr[objID]])
                current_arr[objID] = goal_arr[objID]
                current_index += 1

            goal_objs = [ action_sequence[i][0] for i in range(start_index)]
            current_arr = {}
            for obj in start_arr.keys():
                if obj in goal_objs:
                    current_arr[obj] = goal_arr[obj]
                else:
                    current_arr[obj] = start_arr[obj]
            partial_schedule, partial_buffers = self.process_constraints(subproblem)
            if self.robust_sampling:
                BG = Robust_Buffer_Sampling(
                    subproblem, partial_buffers, partial_schedule, 
                    current_arr, goal_arr, self.Height, self.Width, self.radius
                )
            elif self.sampling:
                BG = Buffer_Sampling(
                    subproblem, partial_buffers, partial_schedule, 
                    current_arr, goal_arr, self.Height, self.Width, self.radius
                )
            else:
                BG = Buffer_Generation(subproblem, partial_buffers, partial_schedule, 
                current_arr, goal_arr, self.Height, self.Width, self.radius
                )
            if self.sampling:
                self.num_collision = BG.num_collision


            if self.robust_sampling:
                action_list = action_list + BG.action_list
                isfeasible = BG.isfeasible
                current_arr = BG.final_arr
                if isfeasible:
                    current_index = end_index+1
                else:
                    break
            else:
                buffer_locations = BG.buffer_locations
                # print "buffer locations"
                # for key, value in buffer_locations.items():
                #     print key, value

                if BG.trouble_action_index == float('inf'):
                    isfeasible = True 
                    trouble_action = []
                else:
                    isfeasible = False
                    trouble_action = subproblem[BG.trouble_action_index]

                for action in subproblem:
                    if action == trouble_action:
                        break
                    objID = action[0]
                    if action[1] == 'b': # move to buffer
                        start_pose = start_arr[objID]
                        buffer_pose = buffer_locations[objID]
                        action_list.append((objID, start_pose, buffer_pose))
                        current_arr[objID] = buffer_pose
                    elif objID in buffer_locations: # move from buffer
                        buffer_pose = buffer_locations[objID]
                        goal_pose = goal_arr[objID]
                        action_list.append((objID, buffer_pose, goal_pose))
                        current_arr[objID] = goal_pose
                    else: # directly to goal
                        start_pose = start_arr[objID]
                        goal_pose = goal_arr[objID]
                        action_list.append([objID, start_pose, goal_pose])
                        current_arr[objID] = goal_pose
                if isfeasible:
                    current_index = end_index+1
                else:
                    break
        if isfeasible:
            while current_index < len(action_sequence): # move objects to the goal after solving subproblems
                objID = action_sequence[current_index][0]
                action_list.append((objID, start_arr[objID], goal_arr[objID]))
                current_arr[objID] = goal_arr[objID]
                current_index += 1
        else:
            action_list = []

        return current_arr, action_list, isfeasible

            
    def process_constraints(self, action_sequence):
        time_stamp = 0
        obj_schedule = {} # dict[obj_id] = [starting_timestamp, goal_timestamp]
        buffers = []
        for action in action_sequence:
            if action[1]=='b':
                obj_schedule[action[0]] = [time_stamp, -1]
                buffers.append(action[0])
            elif action[0] in obj_schedule:
                obj_schedule[action[0]][1] = time_stamp
            else:
                obj_schedule[action[0]] = [time_stamp, time_stamp]
            time_stamp += 1
        return obj_schedule, buffers


    def construct_plan(self):
        # remove useless actions
        Epsilon = 1.0
        real_action_list = loads(dumps(self.action_list))
        for ii, action in enumerate(self.action_list[:-1]):
            p1 = action[1]
            p2 = action[2]
            if (norm(array(p1[:2])-array(p2[:2]))<Epsilon):
                if action in real_action_list: 
                    real_action_list.remove(action)
            elif action[0] == self.action_list[ii+1][0]:
                if (action in real_action_list) and (self.action_list[ii+1] in real_action_list):
                    index = real_action_list.index(action)
                    real_action_list.remove(action)
                    real_action_list.remove(self.action_list[ii+1])
                    real_action_list.insert(
                        index, (action[0], p1, self.action_list[ii+1][2])
                    )
        self.action_list = real_action_list
        

    def construct_DG(self):
        self.DG = {}
        for goal_obj, goal_center in self.goal_arr.items():
            self.DG[goal_obj] = set()
            for start_obj, start_center in self.start_arr.items():
                if start_obj == goal_obj:
                    continue
                if (sqrt((start_center[0]-goal_center[0])**2+
                (start_center[1]-goal_center[1])**2) <= 2*self.radius):
                    self.DG[goal_obj].add(start_obj)


    # def process_constraints(self, action_sequence):
    #     time_stamp = 0
    #     obj_schedule = {} # dict[obj_id] = [starting_timestamp, goal_timestamp]
    #     buffers = []
    #     for action in action_sequence:
    #         if action[1]=='b':
    #             obj_schedule[action[0]] = [time_stamp, -1]
    #             buffers.append(action[0])
    #         elif action[0] in obj_schedule:
    #             obj_schedule[action[0]][1] = time_stamp
    #         else:
    #             obj_schedule[action[0]] = [time_stamp, time_stamp]
    #         time_stamp += 1
    #     return obj_schedule, buffers


    # def quadratic_optimization(self, obj_schedule, buffers):
    #     m = gp.Model()
    #     m.setParam('OutputFlag', 0)
    #     m.setParam('Threads', 1)
    #     m.setParam('NonConvex', 2)

    #     b = len(buffers)

    #     ### variables
    #     x = m.addVars(b, vtype=gp.GRB.CONTINUOUS, lb=self.radius, ub=self.Width-self.radius)
    #     y = m.addVars(b, vtype=gp.GRB.CONTINUOUS, lb=self.radius, ub=self.Height-self.radius)

    #     ### Constraints
    #     for buffer_index, buffer in enumerate(buffers):
    #         buffer_start = obj_schedule[buffer][0]
    #         buffer_end = obj_schedule[buffer][1]            

    #         # fixed poses
    #         for obj_id, schedule in obj_schedule.items():
    #             if schedule[1] < buffer_end: # goal pose is the obs
    #                 m.addQConstr(
    #                     (x[buffer_index]-self.goal_arr[obj_id][0])*(x[buffer_index]-self.goal_arr[obj_id][0]) + 
    #                     (y[buffer_index]-self.goal_arr[obj_id][1])*(y[buffer_index]-self.goal_arr[obj_id][1]) >= 
    #                     4*self.radius**2 )
    #             if schedule[0] > buffer_start: # start pose is the obs
    #                 m.addQConstr(
    #                     (x[buffer_index]-self.start_arr[obj_id][0])*(x[buffer_index]-self.start_arr[obj_id][0]) + 
    #                     (y[buffer_index]-self.start_arr[obj_id][1])*(y[buffer_index]-self.start_arr[obj_id][1]) >= 
    #                     4*self.radius**2 )
    #         for later_buffer_index, later_buffer in enumerate(buffers[(buffer_index+1):]):
    #             later_buffer_start = obj_schedule[later_buffer][0]
    #             if later_buffer_start < buffer_end: # have overlaps
    #                 m.addQConstr(
    #                     (x[buffer_index]-x[later_buffer_index])*(x[buffer_index]-x[later_buffer_index]) + 
    #                     (y[buffer_index]-y[later_buffer_index])*(y[buffer_index]-y[later_buffer_index]) >= 
    #                     4*self.radius**2
    #                 )

    #     m.optimize()
        
    #     buffer_locations = {}
    #     try:
    #         for buffer_id, buffer_obj_id in enumerate(buffers):
    #             buffer_locations[buffer_obj_id] = (x[buffer_id].x, y[buffer_id].x)
    #         isfeasible = True
    #     except Exception:
    #         isfeasible = False

    #     return isfeasible, buffer_locations
