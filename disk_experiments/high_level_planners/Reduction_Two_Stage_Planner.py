from numpy import array
from numpy.linalg import norm
from random import choice
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from local_planner.Labeled_Partial_Optimizer import Labeled_Partial_Optimizer
from local_planner.Unlabeled_Reduction_Partial_Optimizer import Unlabeled_Reduction_Partial_Optimizer as Unlabeled_Partial_Optimizer
from tools.util import deepcopy_dict

class Reduction_Two_Stage_Planner(object):
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
    def __init__(self, start_arr, goal_arr, Height, Width, radius, sampling = False):
        self.start_arr = deepcopy_dict(start_arr)
        self.goal_arr = deepcopy_dict(goal_arr)
        self.Height = Height
        self.Width = Width
        self.radius = radius
        self.sampling = sampling

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
            current_id = random.choice(self.tree.keys())
            while current_id == "R0":
                current_id = random.choice(self.tree.keys())
            self.growSubTree(current_id, "R0")
        
        self.construct_plan()


    def growSubTree(self, startID, goalID):
        start_arr = self.tree[startID].arrangement
        goal_arr = self.tree[goalID].arrangement
        
        unlabeled_planner = Unlabeled_Partial_Optimizer(start_arr, goal_arr, 
        self.Height, self.Width, self.radius, sampling=self.sampling)

        transitions = unlabeled_planner.transitions

        if unlabeled_planner.isfeasible:
            labeled_planner = Labeled_Partial_Optimizer(unlabeled_planner.new_arr, goal_arr, self.Height, self.Width, self.radius, sampling=self.sampling)
            new_arr = labeled_planner.new_arr
            transitions = transitions + labeled_planner.transitions
            self.isConnected = labeled_planner.isfeasible
        else:
            new_arr = unlabeled_planner.new_arr

        if self.isNewArr(new_arr):
            nodeID = "L"+str(len(self.tree_Parents)+1)
            self.tree[nodeID] = ArrNode(new_arr, nodeID)
            self.tree[nodeID].set_parent(startID)
            self.tree_Parents[nodeID] = startID
            self.tree[nodeID].set_path(transitions)
            if self.isConnected:
                self.bridge = [nodeID, "R0"]
            return nodeID
        return -1


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


    def construct_plan(self):
        self.action_list = []
        # Left plan
        current_leftID = self.bridge[0]
        while current_leftID in self.tree_Parents:
            parent_nodeID = self.tree_Parents[current_leftID]
            self.action_list = self.tree[current_leftID].path_from_parent + self.action_list
            current_leftID = parent_nodeID

        return self.action_list

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
