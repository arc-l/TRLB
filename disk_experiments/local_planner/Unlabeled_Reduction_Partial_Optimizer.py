from math import sqrt
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from primitive_plan.Unlabeled_DFS_DP import DFS_DP_Search as Unlabeled_DFS_DP_Search
from buffer_allocation.Buffer_Generation import Buffer_Generation
from buffer_allocation.Buffer_Sampling import Buffer_Sampling
from tools.util import deepcopy_dict

class Unlabeled_Reduction_Partial_Optimizer(object):
    def __init__(self, start_arr, goal_arr, Height, Width, radius, sampling=False):
        self.start_arr = start_arr
        self.goal_arr = goal_arr
        self.Height = Height
        self.Width = Width
        self.radius = radius
        self.sampling = sampling
        
        self.new_arr = {} # resulting arrangement
        self.transitions = [] # action list
        self.isfeasible = False # whether the goal arr is reachable
        
        # check which objects are still away from goal
        epsilon = 1e-4
        real_start_arr = {}
        real_goal_arr = {}

        for obj in start_arr.keys():
            # check whether they are at the goal
            if (sqrt((start_arr[obj][0]-goal_arr[obj][0])**2+
                (start_arr[obj][1]-goal_arr[obj][1])**2) >= epsilon):
                real_start_arr[obj] = start_arr[obj]
                real_goal_arr[obj] = goal_arr[obj]

        Bi_DG = self.construct_Unlabeled_DG(real_start_arr, real_goal_arr)

        _, goal_pose_ordering = Unlabeled_DFS_DP_Search(Bi_DG)
        start_poses_ordering, matchup = self.construct_matchup(goal_pose_ordering, Bi_DG, set(real_start_arr.keys()))

        unlabeled_goal_arr = deepcopy_dict(goal_arr)
        for start in real_start_arr.keys():
            unlabeled_goal_arr[start] = goal_arr[matchup[start]]

        action_sequence = self.construct_action_sequence(start_poses_ordering, goal_pose_ordering, Bi_DG)

        Problem_Partition = self.problem_partitioning( action_sequence)

        self.new_arr, self.transitions, self.isfeasible = self.call_subproblems(action_sequence, Problem_Partition, 
        start_arr, unlabeled_goal_arr)
    
    def construct_Unlabeled_DG(self, start_arr, goal_arr):
        Bi_DG = {} # dict[goal]:set(starts)
        for goal_obj, goal_center in goal_arr.items():
            Bi_DG[goal_obj] = set()
            for start_obj, start_center in start_arr.items():
                if (sqrt((start_center[0]-goal_center[0])**2+
                (start_center[1]-goal_center[1])**2) <= 2*self.radius):
                    Bi_DG[goal_obj].add(start_obj)
        return Bi_DG

    def construct_matchup(self, goal_pose_ordering, Bi_DG, remain_start_poses):
        start_poses_ordering = []
        current_buffer = []
        for goal in goal_pose_ordering:
            moving_objects = []
            for start in Bi_DG[goal]:
                if start in remain_start_poses:
                    moving_objects.append(start)
                    remain_start_poses.remove(start)
            if (len(moving_objects)==0):
                if (len(current_buffer)>0):
                    obj = current_buffer.pop(0)
                    start_poses_ordering.append(obj)
                continue
            start_poses_ordering.append(moving_objects[0])
            for start in moving_objects[1:]:
                current_buffer.append(start)
        start_poses_ordering = start_poses_ordering + current_buffer + list(remain_start_poses)

        matchup = {}
        for index, start in enumerate(start_poses_ordering):
            matchup[start] = goal_pose_ordering[index]

        return start_poses_ordering, matchup

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
                current_arr[objID] = goal_pose
            return current_arr, action_list, isfeasible
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
                BG = Buffer_Sampling(subproblem, partial_buffers, partial_schedule, 
                current_arr, goal_arr, self.Height, self.Width, self.radius
                )
            else:
                BG = Buffer_Generation(subproblem, partial_buffers, partial_schedule, 
                current_arr, goal_arr, self.Height, self.Width, self.radius
                )

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
                    action_list.append([objID, start_pose, buffer_pose])
                    current_arr[objID] = buffer_pose
                elif objID in buffer_locations: # move from buffer
                    buffer_pose = buffer_locations[objID]
                    goal_pose = goal_arr[objID]
                    action_list.append([objID, buffer_pose, goal_pose])
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
                action_list.append([objID, start_arr[objID], goal_arr[objID]])
                current_arr[objID] = goal_arr[objID]
                current_index += 1

        return current_arr, action_list, isfeasible

    def construct_action_sequence(self, start_poses_ordering, goal_pose_ordering, Bi_DG):        
        action_sequence = []
        current_buffer = set()
        current_moved_start = set()
        for index, goal in enumerate(goal_pose_ordering):
            start = start_poses_ordering[index]
            current_obstacles = Bi_DG[goal].difference(current_moved_start)
            if start in current_buffer:
                for obs in current_obstacles:
                    action_sequence.append((obs, 'b'))
                    current_moved_start.add(obs)
                    current_buffer.add(obs)
                action_sequence.append((start, 'g'))
                current_buffer.remove(start)
            else:
                for obs in current_obstacles.difference({start}):
                    action_sequence.append((obs, 'b'))
                    current_moved_start.add(obs)
                    current_buffer.add(obs)
                # check whether the start obj is colliding with other goal poses
                CollidingOther = False
                for goal in goal_pose_ordering[(index+1):]:
                    if start in Bi_DG[goal]:
                        CollidingOther = True
                if CollidingOther:
                    action_sequence.append((start, 'g'))
                current_moved_start.add(start)
        return action_sequence

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
