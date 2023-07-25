from math import sqrt,pi
import os
import sys
from numpy import array,average

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from Vertex_Weighted_TRLB.Labeled_DFS_DP import DFS_DP_Search
from Vertex_Weighted_TRLB.FVS_DP_Solver import FVS_Plan_Search
from Vertex_Weighted_TRLB.Buffer_Sampling import Buffer_Sampling
from tools.util import deepcopy_dict, polysCollide, poly_stick, poly_disc
from tools.general_objects import get_poly, get_AABB, get_area

class Labeled_Partial_Optimizer(object):
    '''
    Input: sampling: use sampling based buffer generation or not 
    '''
    def __init__(self, start_arr, goal_arr, Height, Width, Length, ratio, primitive_plan):
        self.Height = Height
        self.Width = Width
        self.Length = Length
        self.ratio = ratio
        self.sampling = True
        self.primitive_plan = primitive_plan

        # records and results
        self.new_arr = {} # resulting arrangement
        self.transitions = []
        self.isfeasible = False
        self.num_collision = 0
        self.max_node = 0
        self.total_node = 0
        self.FVS = 0 # FVS amount when TBM is chosen
        
        epsilon = 1e-4
        real_start_arr = {}
        real_goal_arr = {}

        for obj in start_arr.keys():
            # check whether they are at the goal
            if (sqrt((start_arr[obj][0]-goal_arr[obj][0])**2+
                (start_arr[obj][1]-goal_arr[obj][1])**2+
                (start_arr[obj][2]-goal_arr[obj][2])**2) >= epsilon):
                real_start_arr[obj] = start_arr[obj]
                real_goal_arr[obj] = goal_arr[obj]
        
        DG = self.construct_DG(real_start_arr, real_goal_arr)
        Weights = self.construct_weight_graph(start_arr, real_start_arr)

        #TODO - Check 
        if self.primitive_plan == 'RBM':
            _, action_sequence, self.max_node, self.total_node = DFS_DP_Search(DG,Weights)
        elif self.primitive_plan == 'TBM':
            action_sequence,self.FVS = FVS_Plan_Search(DG, Weights, return_FVS=True)


        # print action_sequence
        # for action in action_sequence:
        #     print action

        Problem_Partition = self.problem_partitioning( action_sequence)

        self.new_arr, self.transitions, self.isfeasible = self.call_subproblems(action_sequence, Problem_Partition, 
        start_arr, goal_arr)

    
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
            if self.sampling:
                BG = Buffer_Sampling(
                    subproblem, partial_buffers, partial_schedule, 
                    current_arr, goal_arr, self.Height, self.Width, self.Length, self.ratio
                )


            if self.sampling:
                self.num_collision += BG.num_collision
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

    def construct_DG(self, start_arr, goal_arr):
        DG = {}
        for goal_obj, goal_pose in goal_arr.items():
            DG[goal_obj] = set()
            if goal_pose[3] == 'cuboid':
                goal_poly = array(
                    poly_stick([goal_pose[0],goal_pose[1]], goal_pose[2], 
                    goal_pose[4], goal_pose[5])
                    )
            elif goal_pose[3] == 'disc':
                goal_poly = array(
                    poly_disc([goal_pose[0],goal_pose[1]], goal_pose[2], 
                    goal_pose[4], goal_pose[5])
                    )
            else:
                goal_poly = array(get_poly(goal_pose))
            for start_obj, start_pose in start_arr.items():
                if start_pose[3] == 'cuboid':
                    start_poly = array(
                        poly_stick([start_pose[0],start_pose[1]], start_pose[2], 
                        start_pose[4], start_pose[5])
                    )
                elif start_pose[3] == 'disc':
                    start_poly = array(
                        poly_disc([start_pose[0],start_pose[1]], start_pose[2], 
                        start_pose[4], start_pose[5])
                    )
                else:
                    start_poly = array(get_poly(start_pose))
                if polysCollide(start_poly, goal_poly):
                    DG[goal_obj].add(start_obj)
        return DG
        
        
    def construct_weight_graph(self, start_arr:dict,real_start_arr:dict,flag='coll'):
        '''
        generate weights based on aspect ratio and area
        flag: 
            coll: collision possibility
            eff: size
        '''
        if self.primitive_plan == 'RBM':
            Weights = {
                k:int(round(real_start_arr[k][6])) for k in real_start_arr.keys()
            }
        elif self.primitive_plan == 'TBM':
            Weights = {
                k:real_start_arr[k][6] for k in real_start_arr.keys()
            }
        else:
            raise ValueError('primitive plan type not found')
        return Weights
        
    def weight_graph_collion_possibility(self, start_arr:dict,real_start_arr:dict):
        '''
        deprecated codes
        generate weight graph collision possibility
        '''
        areas = {}
        cs = {} # circumference
        for objID,pose in start_arr.items():
            if pose[3] == 'cuboid':
                areas[objID] = pose[4]*pose[5]
                cs[objID] = 2*(pose[4]+pose[5])
            elif pose[3] == 'disc':
                areas[objID] = pi/4.0*pose[4]*pose[5]
                a,b = pose[4]/2,pose[5]/2
                h = ((a-b)**2)/((a+b)**2)
                cs[objID] = pi*(a+b)*(1+3*h/(10+sqrt(4-3*h)))
            else:
                areas[objID] = get_area(pose)
                AABB = get_AABB(pose)
                cs[objID] = 2*(abs(AABB[0][0]-AABB[0][1])+abs(AABB[0][0]-AABB[1][0]))
        avg_area = average(list(areas.values()))
        avg_r = sqrt(avg_area/pi)
        abs_p = {}
        for objID, pose in start_arr.items():
            abs_p[objID] = (areas[objID]+avg_area+avg_r*cs[objID])/\
                ((1000-2*sqrt(avg_r))**2)
        # print(areas)
        # normalize
        min_p = min(list(abs_p.values()))
        Weights = {k:max(1,int(abs_p[k]/min_p)) for k in abs_p.keys() if k in real_start_arr}
        # print("Weights",Weights)
        return Weights    
    
    
    def weight_graph_effort(self,start_arr:dict, real_start_arr:dict):
        '''
        deprecated codes
        evaluate the total effort
        '''
        areas = {}
        for objID,pose in start_arr.items():
            if pose[3] == 'cuboid':
                areas[objID] = pose[4]*pose[5]
            elif pose[3] == 'disc':
                areas[objID] = pi/4.0*pose[4]*pose[5]
            else:
                areas[objID] = get_area(pose)
        min_area = min(list(areas.values()))
        Weights = {k:max(1,int(areas[k]/min_area)) for k in areas.keys() if k in real_start_arr}
        return Weights