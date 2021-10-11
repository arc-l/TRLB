import sys
import numpy as np
from shapely.geometry import Polygon
import random
import os
import math

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from tools.util import poly_disk, isCollisionFree, poly_stick
from fmrs.partial_fmRS import partial_fmRS

# Disable
def blockPrint():
    sys.stdout = open(os.devnull, 'w')

# Restore
def enablePrint():
    sys.stdout = sys.__stdout__


class FMRS_Planner(object):
    '''
    Input:
    Initial_arrangement: dict[obj id]=(x,y,d)
    final_arrangement: dict[obj id]=(x,y,d)
    Height
    Width
    Density
    WL_radio width/length
    
    Output:
    The whole plan
    '''

    def __init__(self, init_arr, final_arr, Height, Width, Density, WL_ratio=0.3):
        self.init_arr = init_arr
        self.final_arr = final_arr
        self.numObjs = len(self.init_arr)
        self.ratio = WL_ratio # Width/Height=0.3
        self.Length = math.sqrt( Density * Width * Height/self.numObjs/self.ratio)

        # Solution
        self.action_list = []

        # BiRRT tree initialization
        self.treeL = {}
        self.treeR = {}
        self.trees = {}
        self.trees["Left"] = self.treeL
        self.trees["Right"] = self.treeR
        self.treeL_Parents = {}
        self.treeR_Parents = {}
        self.bridge = ["", ""] # A node that has two names "L_i", "R_j"(connecting left and right)

        # roots
        self.treeL["L0"] = ArrNode(self.init_arr, "L0")
        self.treeR["R0"] = ArrNode(self.final_arr, "R0")

        self.isConnected = False

        # initial connection attempt
        partial_arr_id, is_monotone, _ = self.growSubTree(self.treeL["L0"], self.treeR["R0"], "Left")
        if is_monotone:
            self.isConnected = True
            self.bridge[0] = partial_arr_id
            self.bridge[1] = "R0"
            self.construct_plan()
        else:
            partial_arr_id, is_monotone, _ = self.growSubTree(self.treeR["R0"], self.treeL[partial_arr_id], "Right")

        while not self.isConnected:
            random_arr = random_arrangement(self.init_arr.keys(), Density, Height, Width, self.ratio)
            random_node = ArrNode(random_arr, "Random")
            nearest_ID = self.closestNode(random_arr, "Left")
            left_partial_arr_id, _, _ = self.growSubTree(self.treeL[nearest_ID], random_node, "Left")
            nearest_ID = self.closestNode(self.treeL[left_partial_arr_id].arrangement, "Right")
            right_partial_arr_id, is_monotone, _ = self.growSubTree(self.treeR[nearest_ID], self.treeL[left_partial_arr_id], "Right")

            if is_monotone:
                self.isConnected = True
                self.bridge[0] = left_partial_arr_id
                self.bridge[1] = right_partial_arr_id
                self.construct_plan()


        
    def closestNode(self,random_arr, treeSide):
        random_arr_list = []
        for i in sorted(random_arr.keys()):
            random_arr_list = random_arr_list + [random_arr[i][0], random_arr[i][1]]
        random_arr_vec = np.array(random_arr_list)
        shortest_distance = float('inf')
        nearest_neighbor = None

        if treeSide == "Left":
            for node_id, node in self.treeL.items():
                arr_list = []
                for i in sorted(random_arr.keys()):
                    arr_list = arr_list + [node.arrangement[i][0], node.arrangement[i][1]]
                distance = np.linalg.norm(np.array(arr_list)-random_arr_vec, 2)
                if distance <= shortest_distance:
                    shortest_distance = distance
                    nearest_neighbor = node_id
        else:
            for node_id, node in self.treeR.items():
                arr_list = []
                for i in random_arr.keys():
                    arr_list = arr_list + [node.arrangement[i][0], node.arrangement[i][1]]
                distance = np.linalg.norm(np.array(arr_list)-random_arr_vec, 2)
                if distance <= shortest_distance:
                    shortest_distance = distance
                    nearest_neighbor = node_id
        return nearest_neighbor


    def growSubTree(self,init_node, final_node, treeside):
        if treeside == "Left": # develop left
            start_poses = init_node.arrangement
            goal_poses = final_node.arrangement
        
            planner = partial_fmRS(
                start_poses, goal_poses, self.ratio, self.Length
            )

            # if the planner cannot make progress
            if planner.is_frozen:
                return init_node.node_id, planner.is_monotone, planner.is_frozen
            
            nodeID = "L"+str(len(self.treeL_Parents)+1)
            partial_arr_node = ArrNode(planner.partial_arr, nodeID)
            self.treeL[nodeID] = partial_arr_node
            partial_arr_node.set_parent(init_node.node_id)
            partial_arr_node.set_path(planner.partial_plan)
            self.treeL_Parents[nodeID] = init_node.node_id
            return nodeID, planner.is_monotone, planner.is_frozen
        
        else: # develop right
            start_poses = init_node.arrangement
            goal_poses = final_node.arrangement
        
            planner = partial_fmRS(
                start_poses, goal_poses, self.ratio, self.Length
            )

            # if the planner cannot make progress
            if planner.is_frozen:
                return init_node.node_id, planner.is_monotone, planner.is_frozen

            nodeID = "R"+str(len(self.treeR_Parents)+1)
            partial_arr_node = ArrNode(planner.partial_arr, nodeID)
            self.treeR[nodeID] = partial_arr_node
            partial_arr_node.set_parent(init_node.node_id)
            partial_arr_node.set_path(planner.partial_plan)
            self.treeR_Parents[nodeID] = init_node.node_id
            return nodeID, planner.is_monotone, planner.is_frozen

    def construct_plan(self):
        self.action_list = []
        # Left plan
        current_leftID = self.bridge[0]
        while current_leftID in self.treeL_Parents:
            # self.action_list.insert(0, current_leftID)
            parent_nodeID = self.treeL_Parents[current_leftID]
            for objID in reversed(self.treeL[current_leftID].path_from_parent):
                start_pose = self.treeL[parent_nodeID].arrangement[objID]
                goal_pose = self.treeL[current_leftID].arrangement[objID]
                self.action_list.insert(0,[objID,start_pose, goal_pose])
            current_leftID = parent_nodeID
        # self.action_list.insert(0, current_leftID)

        # Right plan
        current_rightID = self.bridge[1]
        while current_rightID in self.treeR_Parents:
            # self.action_list.append( current_rightID)
            parent_nodeID = self.treeR_Parents[current_rightID]
            for objID in reversed(self.treeR[current_rightID].path_from_parent):
                start_pose = self.treeR[current_rightID].arrangement[objID]
                goal_pose = self.treeR[parent_nodeID].arrangement[objID]
                self.action_list.append([objID,start_pose, goal_pose])
            current_rightID = parent_nodeID
        # self.action_list.append( current_rightID)
        # for action in self.action_list:
        #     print action
        # print "num actions: ", len(self.action_list)
        return self.action_list

def random_arrangement( obj_keys, Density, HEIGHT, WIDTH, ratio):
    numObjs = len(obj_keys)
    Length = math.sqrt( Density * WIDTH * HEIGHT/numObjs/ratio)
    ### Objs ###
    Success = False
    while not Success:
        Success = True
        points = []
        objects = []
        for i in range(numObjs):
            direction = np.random.uniform(low=0.0, high=math.pi)
            polygon = np.array(poly_stick([0,0], direction, Length, ratio*Length))
            isfree = False
            timeout = 10
            while not isfree and timeout > 0:
                timeout -= 1
                point = (
                    random.uniform(0 - min(polygon[:, 0]), WIDTH - max(polygon[:, 0])), 
                    random.uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1])),
                            direction
                )
                isfree = isCollisionFree(polygon, point[:2], objects)

            if timeout <= 0:
                # print("Failed to generate!", numObjs, Density)
                Success = False

            points.append(point)
            objects.append(Polygon(polygon + point[:2]))
    arr = {}
    for i,obj_id in enumerate(obj_keys):
        arr[obj_id] = points[i]
    return arr


class ArrNode(object):
    def __init__(self, arrangement, node_id):
        self.arrangement = arrangement
        self.node_id = node_id
        self.parent = None
        self.path_from_parent = [] 
    
    def set_parent(self, parentID):
        self.parent = parentID
    
    def set_path(self, object_ordering):
        self.path_from_parent = object_ordering