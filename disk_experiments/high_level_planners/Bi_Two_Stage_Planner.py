from numpy import array
from numpy.linalg import norm
from random import choice
import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from local_planner.Labeled_Partial_Optimizer import Labeled_Partial_Optimizer
from local_planner.Unlabeled_Partial_Optimizer import Unlabeled_Partial_Optimizer
from tools.util import deepcopy_dict


class Bi_Two_Stage_Planner(object):
    '''
    A Bi-RRT based structure to find a valid in-place buffer plan
    Input:
    Initial_arrangement: dict[obj id]=(x,y)
    final_arrangement: dict[obj id]=(x,y)
    Environment Height
    Environment Width
    radius
    
    Output:
    The whole plan: [[objID,start_pose, goal_pose]]
    '''
    def __init__(self, start_arr, goal_arr, Height, Width, radius, sampling = False, primitive_plan = 'RB'):
        self.start_arr = deepcopy_dict(start_arr)
        self.goal_arr = deepcopy_dict(goal_arr)
        self.numObjs = len(self.start_arr)
        self.Height = Height
        self.Width = Width
        self.radius = radius
        self.sampling = sampling
        self.primitive_plan = primitive_plan

        # Solution
        self.action_list = []

        # Tree initialization
        self.treeL = {}
        self.treeL_Parents = {}
        self.treeR = {}
        self.treeR_Parents = {}
        
        # root
        self.treeL["L0"] = ArrNode(self.start_arr, "L0")
        self.treeR["R0"] = ArrNode(self.goal_arr, "R0")

        self.isConnected = False
        current_id = "L0"
        while not self.isConnected:
            # Left development
            current_id = choice(self.treeL.keys())
            isnew, new_nodeID, Nearest_nodeID = self.growSubTree(current_id, "R0",side='Left')
            if self.isConnected:
                break
            if isnew:
                _, _, _ = self.growSubTree( Nearest_nodeID, new_nodeID, side='Right')
            if self.isConnected:
                break
            # Right development
            current_id = choice(self.treeR.keys())
            isnew, new_nodeID, Nearest_nodeID = self.growSubTree(current_id, "L0",side='Right')
            if self.isConnected:
                break
            if isnew: 
                _, _, _ = self.growSubTree( Nearest_nodeID, new_nodeID, side='Left')

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
            
            unlabeled_planner = Unlabeled_Partial_Optimizer(start_arr, goal_arr, 
            self.Height, self.Width, self.radius, 
            sampling=self.sampling
            )

            transitions = unlabeled_planner.transitions

            if unlabeled_planner.isfeasible:
                labeled_planner = Labeled_Partial_Optimizer(
                    unlabeled_planner.new_arr, 
                    goal_arr, 
                    self.Height, 
                    self.Width, 
                    self.radius, 
                    sampling=self.sampling, 
                    primitive_plan = self.primitive_plan)
                new_arr = labeled_planner.new_arr
                transitions = transitions + labeled_planner.transitions
                self.isConnected = labeled_planner.isfeasible
            else:
                new_arr = unlabeled_planner.new_arr

            # check same side
            isnew, Left_old_nodeID = self.isNewArr(new_arr, side='Left')
            if not isnew:
                return isnew, Left_old_nodeID, Left_old_nodeID
            else:
                nodeID = "L"+str(len(self.treeL_Parents)+1)
                self.treeL[nodeID] = ArrNode(new_arr, nodeID)
                self.treeL[nodeID].set_parent(startID)
                self.treeL_Parents[nodeID] = startID
                self.treeL[nodeID].set_path(transitions)
                if self.isConnected:
                    self.bridge = [nodeID, goalID]
            
                # check opposite
                dist, Right_old_nodeID = self.check_nearest_neighbor(new_arr, side='Right')
                if dist == 0:
                    self.isConnected = True
                    self.bridge = [nodeID, Right_old_nodeID]
                return True, nodeID, Right_old_nodeID

        # side=Right        
        else:
            start_arr = self.treeR[startID].arrangement
            goal_arr = self.treeL[goalID].arrangement
            
            unlabeled_planner = Unlabeled_Partial_Optimizer(
                start_arr, 
                goal_arr, 
                self.Height, 
                self.Width, 
                self.radius,
                sampling=self.sampling
                )

            transitions = unlabeled_planner.transitions

            if unlabeled_planner.isfeasible:
                labeled_planner = Labeled_Partial_Optimizer(unlabeled_planner.new_arr, goal_arr, self.Height, self.Width, self.radius, sampling=self.sampling, primitive_plan = self.primitive_plan)
                new_arr = labeled_planner.new_arr
                transitions = transitions + labeled_planner.transitions
                self.isConnected = labeled_planner.isfeasible
            else:
                new_arr = unlabeled_planner.new_arr

            # check same side
            isnew, Right_old_nodeID = self.isNewArr(new_arr, side='Right')
            if not isnew:
                return isnew, Right_old_nodeID, Right_old_nodeID
            else:
                nodeID = "R"+str(len(self.treeR_Parents)+1)
                self.treeR[nodeID] = ArrNode(new_arr, nodeID)
                self.treeR[nodeID].set_parent(startID)
                self.treeR_Parents[nodeID] = startID
                self.treeR[nodeID].set_path(transitions)
                if self.isConnected:
                    self.bridge = [goalID,nodeID]
            
                # check opposite
                dist, Left_old_nodeID = self.check_nearest_neighbor(new_arr, side='Left')
                if dist == 0:
                    self.isConnected = True
                    self.bridge = [ Left_old_nodeID, nodeID]
                return True, nodeID, Left_old_nodeID

    def construct_plan(self):
        self.action_list = []
        # Left plan
        current_leftID = self.bridge[0]
        while current_leftID in self.treeL_Parents:
            parent_nodeID = self.treeL_Parents[current_leftID]
            self.action_list = self.treeL[current_leftID].path_from_parent + self.action_list
            current_leftID = parent_nodeID
        
        # Right plan
        current_rightID = self.bridge[1]
        while current_rightID in self.treeR_Parents:
            parent_nodeID = self.treeR_Parents[current_rightID]
            self.action_list = self.action_list + list(reversed([(p[0], p[2], p[1]) for p in self.treeR[current_rightID].path_from_parent]))
            current_rightID = parent_nodeID
        
        # remove useless actions
        Epsilon = 1.0
        check_index = 0
        while check_index<len(self.action_list):
            is_useful = True
            obj = self.action_list[check_index][0]
            p1 = self.action_list[check_index][1]
            p2 = self.action_list[check_index][2]
            if (norm(array(p1[:2])-array(p2[:2]))<Epsilon):
                del self.action_list[check_index]
                check_index -= 1
                check_index = max(check_index,0)
                continue
            elif(check_index<len(self.action_list)-1) and (obj==self.action_list[check_index+1][0]):
                inserted_action = (self.action_list[check_index][0],self.action_list[check_index][1],self.action_list[check_index+1][2])
                del self.action_list[check_index]
                del self.action_list[check_index]
                self.action_list.insert(
                    check_index, inserted_action
                )
                continue
            else:
                check_index += 1
    
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


