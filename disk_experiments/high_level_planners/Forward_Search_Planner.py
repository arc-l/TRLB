from numpy import array
from numpy.linalg import norm
import random
import os
import sys
from ujson import loads, dumps

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from local_planner.Labeled_Partial_Optimizer import Labeled_Partial_Optimizer
from local_planner.Labeled_Partial_Optimizer_PP import Labeled_Partial_Optimizer_PP

class Forward_Search_Planner(object):
    '''
    A tree-based structure to find a valid in-place buffer plan
    Input:
    Initial_arrangement: dict[obj id]=(x,y)
    final_arrangement: dict[obj id]=(x,y)
    Environment Height
    Environment Width
    radius
    sampling: use sampling to generate buffers
    primitive plan: RB/FVS/rand
    
    Output:
    The whole plan: [[objID,start_pose, goal_pose]]
    '''
    def __init__(self, start_arr, goal_arr, Height, Width, radius, sampling=False, primitive_plan = 'RB', robust_sampling = False, PP=False):
        self.start_arr = { k:loads(dumps(start_arr[k])) for k in start_arr.keys()}
        self.goal_arr = { k:loads(dumps(goal_arr[k])) for k in goal_arr.keys()}
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

        # Tree initialization
        self.tree = {}
        self.tree_Parents = {}
        
        # root
        self.tree["L0"] = ArrNode(self.start_arr, "L0")
        self.tree["R0"] = ArrNode(self.goal_arr, "R0")

        self.isConnected = False
        current_id = "L0"
        while not self.isConnected:
            current_id = random.choice(list(self.tree.keys()))
            while current_id == "R0":
                current_id = random.choice(list(self.tree.keys()))
            self.growSubTree(current_id, "R0")
        
        self.construct_plan()

    def growSubTree(self, startID, goalID):
        start_arr = self.tree[startID].arrangement
        goal_arr = self.tree[goalID].arrangement
        
        if self.PP:
            partial_planner = Labeled_Partial_Optimizer_PP(start_arr, goal_arr, self.Height, self.Width, self.radius, sampling=self.sampling, primitive_plan = self.primitive_plan, robust_sampling=self.robust_sampling)
        else:    
            partial_planner = Labeled_Partial_Optimizer(start_arr, goal_arr, self.Height, self.Width, self.radius, sampling=self.sampling, primitive_plan = self.primitive_plan, robust_sampling=self.robust_sampling)

        self.isConnected = partial_planner.isfeasible

        if self.isNewArr(partial_planner.new_arr):
            nodeID = "L"+str(len(self.tree_Parents)+1)
            self.tree[nodeID] = ArrNode(partial_planner.new_arr, nodeID)
            self.tree[nodeID].set_parent(startID)
            self.tree_Parents[nodeID] = startID
            self.tree[nodeID].set_path(partial_planner.transitions)
            if self.isConnected:
                self.bridge = [nodeID, "R0"]
            return nodeID
        return -1

    def construct_plan(self):
        self.action_list = []
        # Left plan
        current_leftID = self.bridge[0]
        while current_leftID in self.tree_Parents:
            # self.action_list.insert(0, current_leftID)
            parent_nodeID = self.tree_Parents[current_leftID]
            self.action_list = self.tree[current_leftID].path_from_parent + self.action_list
            current_leftID = parent_nodeID
        
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

    def isNewArr(self,new_arr):
        new_arr_list = []
        for i in sorted(new_arr.keys()):
            new_arr_list = new_arr_list + [new_arr[i][0], new_arr[i][1]]
        new_arr_vec = array(new_arr_list)

        for node in self.tree.values():
            if node.node_id[0]=="R":
                continue
            if norm(new_arr_vec-node.arr_vec, 2)<1:
                return False
        return True


    



class ArrNode(object):
    def __init__(self, arrangement, node_id):
        self.arrangement = arrangement
        arr_list = []
        for i in sorted(arrangement.keys()):
            arr_list = arr_list + [arrangement[i][0], arrangement[i][1]]
        self.arr_vec = array(arr_list)
        self.node_id = node_id
        self.parent = None
        self.path_from_parent = [] 
    
    def set_parent(self, parentID):
        self.parent = parentID
    
    def set_path(self, object_ordering):
        self.path_from_parent = object_ordering


