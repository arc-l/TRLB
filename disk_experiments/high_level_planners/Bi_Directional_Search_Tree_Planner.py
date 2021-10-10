from numpy import array
from numpy import average
from numpy.linalg import norm
from random import choice, uniform
from math import log1p, log10, sqrt
import sys
import os
from ujson import loads, dumps

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from tools.util import deepcopy_dict
from local_planner.Labeled_Partial_Optimizer import Labeled_Partial_Optimizer
from local_planner.Labeled_Partial_Optimizer_PP import Labeled_Partial_Optimizer_PP

class Bi_Directional_Search_Tree_Planner(object):
    '''
    A Bi-RRT based structure to find a valid in-place buffer plan
    Input:
    Initial_arrangement: dict[obj id]=(x,y)
    final_arrangement: dict[obj id]=(x,y)
    Environment Height
    Environment Width
    radius
    sampling: use sampling to generate buffers. 
        Otherwise, Gurobi will be used to solve an optimization problem
    primitive plan: "RB", "FVS", "random"
    robust sampling: break two step assumption or not
    PP: use preprocess or not

    Output:
    The whole plan: [[objID,start_pose, goal_pose]]
    '''
    def __init__(self, start_arr, goal_arr, Height, Width, radius, sampling=False, primitive_plan = 'RB', robust_sampling = False, PP = False):
        self.start_arr = deepcopy_dict(start_arr)
        self.goal_arr = deepcopy_dict(goal_arr)
        self.numObjs = len(self.start_arr)
        self.Height = Height
        self.Width = Width
        self.radius = radius
        self.sampling = sampling
        self.primitive_plan = primitive_plan
        self.robust_sampling = robust_sampling
        self.PP = PP

        # Solution
        self.action_list = []
        self.num_collision = 0
        self.max_node = 0
        self.total_node = 0

        # Tree initialization
        self.treeL = {}
        self.treeR = {}
        
        
        # root
        self.treeL["L0"] = ArrNode(self.start_arr, "L0")
        self.treeL["L0"].visited_times += 1
        self.treeL["L0"].total_convinience += 1
        self.treeL["L0"].top_k_rewards.append(1.0)
        self.treeL_total_visited_times = 1
        self.treeR["R0"] = ArrNode(self.goal_arr, "R0")
        self.treeR["R0"].visited_times += 1
        self.treeR["R0"].total_convinience += 1
        self.treeR["R0"].top_k_rewards.append(1.0)
        self.treeR_total_visited_times = 1

        self.isConnected = False
        current_id = "L0"
        while not self.isConnected:
            # Left development
            current_id = choice(list(self.treeL.keys()))
            # current_id = self.choose_node_to_develop('Left')
            isnew, new_nodeID, Nearest_nodeID = self.growSubTree(current_id, "R0",side='Left')
            if self.isConnected:
                break
            if isnew:
                self.growSubTree( Nearest_nodeID, new_nodeID, side='Right')
            if self.isConnected:
                break
            # Right development
            current_id = choice(list(self.treeR.keys()))
            # current_id = self.choose_node_to_develop('Right')
            isnew, new_nodeID, Nearest_nodeID = self.growSubTree(current_id, "L0",side='Right')
            if self.isConnected:
                break
            if isnew:
                self.growSubTree( Nearest_nodeID, new_nodeID, side='Left')
                
        self.total_node += (len(self.treeL) + len(self.treeR))
        self.construct_plan()


    def growSubTree(self, startID, goalID, side='Left'):
        '''
        Input:
        
        Output:
        return is_new_node_or_not, generated_nodeID, nearest_nodeID
        '''
        if side == 'Left':
            start_arr = self.treeL[startID].arrangement
            goal_arr = self.treeR[goalID].arrangement
            
            if self.PP:
                partial_planner = Labeled_Partial_Optimizer_PP(start_arr, goal_arr, self.Height, self.Width, self.radius, sampling=self.sampling, primitive_plan = self.primitive_plan, robust_sampling=self.robust_sampling)
            else:    
                partial_planner = Labeled_Partial_Optimizer(start_arr, goal_arr, self.Height, self.Width, self.radius, sampling=self.sampling, primitive_plan = self.primitive_plan, robust_sampling=self.robust_sampling)

            self.isConnected = partial_planner.isfeasible
            self.num_collision += partial_planner.num_collision
            self.max_node = max(self.max_node, partial_planner.max_node+len(self.treeL)+len(self.treeR))
            self.total_node += partial_planner.total_node

            # check same side
            isnew, Left_old_nodeID = self.isNewArr(partial_planner.new_arr, side='Left')
            if not isnew:
                return isnew, Left_old_nodeID, Left_old_nodeID
            else:
                nodeID = "L"+str(len(self.treeL))
                self.treeL[nodeID] = ArrNode(partial_planner.new_arr, nodeID)
                self.node_analysis(nodeID, 'Left', startID, partial_planner.transitions)
                if self.isConnected:
                    self.bridge = [nodeID, goalID]
                else:
                    # check opposite
                    dist, Right_old_nodeID = self.check_nearest_neighbor(partial_planner.new_arr, side='Right')
                    if dist == 0:
                        self.isConnected = True
                        self.bridge = [nodeID, Right_old_nodeID]
                    return True, nodeID, Right_old_nodeID

        # side=Right        
        else:
            start_arr = self.treeR[startID].arrangement
            goal_arr = self.treeL[goalID].arrangement
            
            if self.PP:
                partial_planner = Labeled_Partial_Optimizer_PP(start_arr, goal_arr, self.Height, self.Width, self.radius, sampling=self.sampling, primitive_plan = self.primitive_plan, robust_sampling=self.robust_sampling)
            else:    
                partial_planner = Labeled_Partial_Optimizer(start_arr, goal_arr, self.Height, self.Width, self.radius, sampling=self.sampling, primitive_plan = self.primitive_plan, robust_sampling=self.robust_sampling)

            self.isConnected = partial_planner.isfeasible
            self.num_collision += partial_planner.num_collision
            self.max_node = max(self.max_node, partial_planner.max_node+len(self.treeL)+len(self.treeR))
            self.total_node += partial_planner.total_node

            # check same side
            isnew, Right_old_nodeID = self.isNewArr(partial_planner.new_arr, side='Right')
            if not isnew:
                return isnew, Right_old_nodeID, Right_old_nodeID
            else:
                nodeID = "R"+str(len(self.treeR))
                self.treeR[nodeID] = ArrNode(partial_planner.new_arr, nodeID)
                self.node_analysis(nodeID, 'Right', startID, partial_planner.transitions)
                if self.isConnected:
                    self.bridge = [goalID,nodeID]
                else:
                    # check opposite
                    dist, Left_old_nodeID = self.check_nearest_neighbor(partial_planner.new_arr, side='Left')
                    if dist == 0:
                        self.isConnected = True
                        self.bridge = [ Left_old_nodeID, nodeID]
                    return True, nodeID, Left_old_nodeID
        return True, None, None


    def construct_plan(self):
        self.action_list = []
        # Left plan
        current_leftID = self.bridge[0]
        while current_leftID in self.treeL:
            parent_nodeID = self.treeL[current_leftID].parent
            self.action_list = self.treeL[current_leftID].path_from_parent + self.action_list
            # print('+++++++++')
            # print(current_leftID)
            # for action in self.treeL[current_leftID].path_from_parent:
            #     print(action)
            # print(parent_nodeID)
            # print('---------')
            current_leftID = parent_nodeID
        
        # Right plan
        current_rightID = self.bridge[1]
        while current_rightID in self.treeR:
            parent_nodeID = self.treeR[current_rightID].parent
            self.action_list = self.action_list + list(reversed([(p[0], p[2], p[1]) for p in self.treeR[current_rightID].path_from_parent]))
            # print('+++++++++')
            # print(current_rightID)
            # for action in self.treeR[current_rightID].path_from_parent:
            #     print(action)
            # print(parent_nodeID)
            # print('---------')
            current_rightID = parent_nodeID
        
        # remove useless actions
        Epsilon = 1.0
        self.action_list = [(action[0], tuple(action[1]), tuple(action[2])) for action in self.action_list]
        real_action_list = [(action[0], tuple(action[1]), tuple(action[2])) for action in self.action_list]
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

        return self.action_list


    def isNewArr(self,new_arr, side="Left"):
        new_arr_list = []
        for i in sorted(new_arr.keys()):
            new_arr_list = new_arr_list + [new_arr[i][0], new_arr[i][1]]
        new_arr_vec = array(new_arr_list)

        if side == "Left":
            for nodeID, node in self.treeL.items():
                dist = norm(new_arr_vec-node.arr_vec, 2)
                if dist<1:
                    return False, nodeID

        else:
            for nodeID, node in self.treeR.items():
                dist = norm(new_arr_vec-node.arr_vec, 2)
                if dist<1:
                    return False, nodeID
        return True, None


    def check_nearest_neighbor(self, new_arr, side='Left'):
        Epsilon = 1.0
        Min_dist = float('inf')
        if side=='Left':
            for nodeID, node in self.treeL.items():
                dist = len(new_arr)
                for i in range(len(node.arrangement)):
                    if (
                        (new_arr[i][0]-node.arrangement[i][0])**2+(new_arr[i][1]-node.arrangement[i][1])**2<Epsilon
                        ):
                        dist -= 1
                if dist < Min_dist:
                    Min_dist = dist
                    NN_nodeID = nodeID
                    if dist ==0:
                        return Min_dist, NN_nodeID
        else:
            for nodeID, node in self.treeR.items():
                dist = len(new_arr)
                for i in range(len(node.arrangement)):
                    if (
                        (new_arr[i][0]-node.arrangement[i][0])**2+(new_arr[i][1]-node.arrangement[i][1])**2<Epsilon
                        ):
                        dist -= 1
                if dist < Min_dist:
                    Min_dist = dist
                    NN_nodeID = nodeID
                    if dist ==0:
                        return Min_dist, NN_nodeID
        return Min_dist, NN_nodeID
    

    def node_analysis(self, new_node_ID, treeside, parentID, transitions):
        if treeside == 'Left':
            num_actions_from_root = len(transitions) + self.treeL[parentID].num_actions_from_root
            num_at_goals = self.count_at_goals(self.treeL[new_node_ID].arrangement, self.goal_arr)
            if num_at_goals == 0:
                num_at_goals += 1
            self.treeL[new_node_ID].set_information(parentID, transitions, num_actions_from_root, num_at_goals)
            self.update(new_node_ID, 'Left', num_at_goals/num_actions_from_root)
        else:
            num_actions_from_root = len(transitions) + self.treeR[parentID].num_actions_from_root
            num_at_goals = self.count_at_goals(self.treeR[new_node_ID].arrangement, self.start_arr)
            if num_at_goals == 0:
                num_at_goals += 1
            self.treeR[new_node_ID].set_information(parentID, transitions, num_actions_from_root, num_at_goals)
            self.update(new_node_ID, 'Right', num_at_goals/num_actions_from_root)


    def update(self,new_node_ID, treeside, convinience):
        discount = 1.0
        current_node_ID = new_node_ID
        if treeside == 'Left':
            self.treeL_total_visited_times += 1
            while self.treeL[current_node_ID].parent != None:
                convinience *= discount
                current_node_ID = self.treeL[current_node_ID].parent
                self.treeL[current_node_ID].visited_times += 1
                self.treeL[current_node_ID].total_convinience += convinience
                self.treeL_total_visited_times += 1
                self.treeL[current_node_ID].add_new_reward(convinience)
        else:
            self.treeR_total_visited_times += 1
            while self.treeR[current_node_ID].parent != None:
                convinience *= discount
                current_node_ID = self.treeR[current_node_ID].parent
                self.treeR[current_node_ID].visited_times += 1
                self.treeR[current_node_ID].total_convinience += convinience
                self.treeR_total_visited_times += 1
                self.treeR[current_node_ID].add_new_reward(convinience)


    def count_at_goals(self, curr_arr, goal_arr):
        Epsilon = 1e0
        count = 0
        for i, p in curr_arr.items():
            gp = goal_arr[i]
            if ((p[0]-gp[0])**2+(p[1]-gp[1])**2<Epsilon):
                count += 1
        return count


    def choose_node_to_develop(self, treeside): # UCB
        c = 0
        BestNodeID = 0
        BestReward = -float('inf')
        if treeside == 'Left':
            for k, node in self.treeL.items():
                expected_reward = average(node.top_k_rewards)
                # reward = (node.total_convinience/node.visited_times) + c*sqrt(2*log10(self.treeL_total_visited_times)/node.visited_times)
                reward = expected_reward + c*sqrt(2*log10(self.treeL_total_visited_times)/node.visited_times)
                if reward >= BestReward:
                    BestReward = reward
                    BestNodeID = k
        else:
            for k, node in self.treeR.items():
                expected_reward = average(node.top_k_rewards)
                # reward = (node.total_convinience/node.visited_times) + c*sqrt(2*log10(self.treeR_total_visited_times)/node.visited_times)
                reward = expected_reward + c*sqrt(2*log10(self.treeR_total_visited_times)/node.visited_times)
                if reward >= BestReward:
                    BestReward = reward
                    BestNodeID = k
        # print(BestReward)
        return BestNodeID


class ArrNode(object):
    def __init__(self, arrangement, node_id):
        self.k=2
        self.arrangement = arrangement
        arr_list = []
        for i in sorted(arrangement.keys()):
            arr_list = arr_list + [arrangement[i][0], arrangement[i][1]]
        self.arr_vec = array(arr_list)
        self.node_id = node_id
        self.parent = None
        self.path_from_parent = [] 
        self.num_actions_from_root = 0
        self.num_at_goals = 0
        self.top_k_rewards = []
        self.total_convinience = 0
        self.visited_times = 0

    
    def set_information(self, parentID, transitions, num_actions_from_root, num_at_goals):
        self.set_parent(parentID)
        self.set_path(transitions)
        self.num_actions_from_root = num_actions_from_root
        self.num_at_goals = num_at_goals
        self.total_convinience += (num_at_goals/num_actions_from_root)
        self.visited_times += 1
        self.add_new_reward(num_at_goals/num_actions_from_root)


    def set_parent(self, parentID):
        self.parent = parentID
    

    def set_path(self, object_ordering):
        self.path_from_parent = object_ordering

    
    def add_new_reward(self, convinience):
        if len(self.top_k_rewards) < self.k:
            self.top_k_rewards.append(convinience)
            self.top_k_rewards.sort(reverse=True)
        else:
            self.top_k_rewards.sort(reverse=True)
            if self.top_k_rewards[-1] > convinience:
                self.top_k_rewards.pop()
                self.top_k_rewards.append(convinience)
                self.top_k_rewards.sort(reverse=True)