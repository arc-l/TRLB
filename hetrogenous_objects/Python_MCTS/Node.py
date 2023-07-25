from math import log1p, log2, log10, sqrt, pi, exp
from random import choice, uniform, choices
from shapely.geometry import Polygon
import numpy as np
import copy
import time
import ujson
import _pickle as cPickle

from tools.util import poly_stick, poly_disc, polysCollide, polysWithin
from tools.show_arrangement import show_grid, show_bounding_volume,show_poly_dict,test_buffer_allocation,show_quadtree
from Python_MCTS.QuadTree import QuadTree, Item
from tools.general_objects import get_poly,get_radius

def bv_find_r(t):
    """
    Returns the radius of the bound circle for the specified shape
    """
    if t[3] == 'cuboid':
        return sqrt( (t[4]/2)**2 + (t[5]/2)**2 )
    elif t[3] == 'disc':
        return max(t[4], t[5])/2
    else:
        return get_radius(t)

class Node(object):
    def __init__(
        self, nodeID, current_arr, target_arr, 
        radius, parent, Width, Height,
        action_from_parent = None,
        updated_objID = None,UCB_scalar=1.0,
        collision_check='naive',fineness=10,
        max_depth=4, buffer_strategy = 'naive',
        interval_size = 3
        ) -> None:
        super().__init__()
        
        # recording
        self.time_coll_setup = 0
        self.time_coll_check = 0
        self.num_collision = 0
        
        self.nodeID = nodeID
        self.current_arr = current_arr
        self.target_arr = target_arr
        self.unvisited_actions = set(self.objects_away_from_target())
        # self.unvisited_actions = set(self.current_arr.keys())
        self.children = {}
        self.parent = parent
        self.action_from_parent = action_from_parent # the object wanted to move
        self.updated_objID = updated_objID # the actually moved obj for the intended action
        self.visited_time = 0
        self.total_reward = 0
        self.radius = radius
        self.Width = Width
        self.Height = Height
        self.UCB_scalar = UCB_scalar
        self.collision_strategy = collision_check
        self.resolution = min(self.Width,self.Height)/fineness
        self.max_depth = max_depth # depth of the quadtree
        self.buffer_strategy = buffer_strategy
        
        # set up for sparse collision checker update
        if self.parent == None:
            self.depth = 0
        else:
            self.depth = self.parent.depth + 1
        self.interval_size = interval_size
        self.updating_obj_set = set()
        if self.updated_objID != None:
            self.updating_obj_set.add(self.updated_objID)
        
        self.last_update_parent_node = self.parent
        while (self.last_update_parent_node != None) and ((self.last_update_parent_node.depth % self.interval_size)>0):
            self.updating_obj_set.add(self.last_update_parent_node.updated_objID)
            self.last_update_parent_node = self.last_update_parent_node.parent
        

        start = time.time()
        self.update_object_info(parent)
        self.time_coll_setup += time.time() - start
        
    
    def objects_away_from_target(self):
        Epsilon = 1e0
        obj_list = []
        for ID, p in self.current_arr.items():
            tp = self.target_arr[ID]
            if (p[0]-tp[0])**2+(p[1]-tp[1])**2>Epsilon:
                obj_list.append(ID)
        return obj_list
        

    def UCB(self, use_heuristic):
        assert len(self.unvisited_actions) == 0
        BestAction = 0
        BestReward = -float('inf')
        for action, child in self.children.items():
            if use_heuristic == 0:
                Reward = child.total_reward/child.visited_time + self.UCB_scalar * \
                    sqrt(2*log10(self.visited_time)/child.visited_time)
            elif use_heuristic == 2:
                Reward = child.total_reward/child.visited_time + ((1 + self.target_arr[action][6])*self.UCB_scalar) * \
                    sqrt(2*log10(self.visited_time)/child.visited_time)
                # print(action, c)
            elif use_heuristic == 3:
                Reward = (1 + self.target_arr[action][6]/10)*(child.total_reward/child.visited_time + self.UCB_scalar * \
                    sqrt(2*log10(self.visited_time)/child.visited_time))
            elif use_heuristic == 4:
                Reward = (1 + self.target_arr[action][6]/10)*child.total_reward/child.visited_time + self.UCB_scalar * \
                    sqrt(2*log10(self.visited_time)/child.visited_time)
            else:
                raise ValueError('Invalid heuristic code')
                

            
            if Reward >= BestReward:
                BestReward = Reward
                BestAction = action
        return self.children[BestAction]


    def expansion(self):
        action = choice(list(self.unvisited_actions))
        # action = list(self.unvisited_actions)[0]
        self.unvisited_actions.remove(action)
        return self.Action_Parametriczation(action)


    def Action_Parametriczation(self, action):
        # obs_list = [p for i, p in self.current_arr.items() if i != action]
        if not self.obj_obj_collision_checking(
            action,self.target_arr[action],
            self.collision_strategy, 
            additional_obs_polys=[ self.obj_poly_dict[k] for k in self.updating_obj_set if k != action]
            ):
            return action, action, self.target_arr[action]
        else:
            moved_obj = self.Closest_Object(action)
            new_position = self.buffer_allocation(
                moved_obj,
                additional_obs_polys = [self.generate_object_polygon(self.target_arr[action])]+
                [ self.obj_poly_dict[k] for k in self.updating_obj_set if k != moved_obj]
            )
            return action, moved_obj, new_position


    def Closest_Object(self, obj):
        target = self.target_arr[obj]
        shortest_distance = float('inf')
        closest_obj = 0
        for ID, obs in self.current_arr.items():
            if ID == obj:
                continue
            dist = (obs[0]-target[0])**2+(obs[1]-target[1])**2
            
            if dist <= shortest_distance:
                closest_obj = ID
                shortest_distance = dist
        return closest_obj


    def buffer_allocation(self, moved_objID,additional_obs_polys=[]):
        '''
        additional_obs_polys: additional obstacles besides current objects and the boundary
        '''
        timeout = 1e2
        moved_object_pose = self.current_arr[moved_objID]
        
        while timeout > 0:
            timeout -= 1
            if self.buffer_strategy == 'naive':
                new_position = self.naive_buffer_suggestion(self.current_arr[moved_objID])
            elif self.buffer_strategy == 'quad':
                new_position = self.quad_tree_buffer_suggestion(self.current_arr[moved_objID])
            else:
                raise ValueError('Buffer allocation strategy not found!')
            new_poly = self.generate_object_polygon(new_position)
            if (
                self.is_object_in_workspace(new_poly)
                ) and (
                not self.obj_obj_collision_checking(
                    moved_objID,new_position,self.collision_strategy,additional_obs_polys
                    )):
                # test_buffer_allocation(self.current_arr,moved_objID,new_position)
                return new_position
        else:
            return None


    def naive_buffer_suggestion(self, moved_object_pose):
        '''
        uniform distribution for buffer pose
        '''
        return tuple(
                [
                    uniform(0, self.Width),
                    uniform(0, self.Height),
                    uniform(0, 2*pi)
                ]+list(moved_object_pose[3:])
            )
        

    def quad_tree_buffer_suggestion(self,moved_object_pose):
        ''' 
        give higher possibility for space with fewer objects 
        for the curr_tree, get children tree num_obj
        give possibility to child trees
        if the chosen one is None
        get bounding box
        uniformly sample in the bounding box
        '''
        if (self.depth%self.interval_size)==0:
            curr_tree = self.quadtree
        else:
            curr_tree = self.last_update_parent_node.quadtree
        while 1:
            subtrees = [curr_tree.nw, curr_tree.ne, curr_tree.se, curr_tree.sw]
            weights = []
            c = 0.5 # coefficient for weights
            for st in subtrees:
                if st == None:
                    weights.append(exp(-c*0))
                else:
                    weights.append(exp(-c*st.num_obj))
            selected_index = choices(
                [0,1,2,3], weights=weights,k=1
            )[0]
            # print(weights)
            # print(selected_index)
            if subtrees[selected_index] != None:
                curr_tree = subtrees[selected_index]
            elif selected_index == 0:
                x = uniform(curr_tree.l,curr_tree.cx)
                y = uniform(curr_tree.t,curr_tree.cy)
                break
            elif selected_index == 1:
                x = uniform(curr_tree.cx,curr_tree.r)
                y = uniform(curr_tree.t,curr_tree.cy)
                break
            elif selected_index == 2:
                x = uniform(curr_tree.cx,curr_tree.r)
                y = uniform(curr_tree.cy,curr_tree.b)
                break
            elif selected_index == 3:
                x = uniform(curr_tree.l,curr_tree.cx)
                y = uniform(curr_tree.cy,curr_tree.b)
                break
            else:
                raise ValueError('unexpected selected index')

        return tuple(
                [
                    x,
                    y,
                    uniform(0, 2*pi)
                ]+list(moved_object_pose[3:])
            )
                

    def is_object_in_workspace(self, obj_poly):
        '''
        check whether the object pose is inside the workspace
        '''
        start = time.time()
        # making sure that obj is not at the boundary
        workspace = Polygon(poly_stick((self.Width/2, self.Height/2), 0, self.Width, self.Height))

        ret = workspace.contains(Polygon(obj_poly))
        self.time_coll_check += time.time() - start
        self.num_collision += 1
        return ret
    
    
    def generate_object_polygon(self,pose):
        '''
        generate object polygon based on object pose (list of properties)
        '''
        if pose[3] == 'cuboid':
            obj_poly = np.array(
                poly_stick(
                    [pose[0], pose[1]], pose[2], pose[4], pose[5]
                )
            )
        elif pose[3] == 'disc':
            obj_poly = np.array(
                poly_disc(
                    [pose[0], pose[1]], pose[2], pose[4], pose[5]
                )
            )
        else:
            obj_poly = np.array(
                get_poly(pose)
            )
        return obj_poly
    
    
    def obj_obj_collision_checking(self, target_obj, pose, strategy='naive',additional_obs_polys=[]):
        '''
        collision check for object i at a certain location and other objects at current locations
        For two purposes:
        1. whether the goal pose is in collision right now
        2. whether a buffer location is in collision with other objects at current location
        '''
        start = time.time()
        obj_poly = self.generate_object_polygon(pose)
        for obs_poly in additional_obs_polys:
            self.num_collision += 1
            if polysCollide(obj_poly, obs_poly):
                self.time_coll_check += time.time() - start
                return True
        ignoring_objIDs = set([e for e in self.updating_obj_set])
        ignoring_objIDs.add(target_obj)
        if strategy == 'naive':
            ret = self.naive_collision_checking(target_obj, obj_poly)
        elif strategy == 'bv':
            ret = self.bounding_volume_collision_checking(target_obj,pose,obj_poly,ignoring_objIDs)
        elif strategy == 'grid':
            ret = self.space_partitioning_collision_checking(target_obj, obj_poly,ignoring_objIDs)
        elif strategy == 'quad':
            ret = self.quadtree_collision_checking(target_obj,obj_poly,ignoring_objIDs)
        else:
            raise ValueError("Strategy not defined.")
        self.time_coll_check += time.time() - start
        return ret
        
        
    def naive_collision_checking(self, objID, obj_poly):
        '''
        pairwise collision check
        '''
        for obsID, obs_poly in self.obj_poly_dict.items():
            self.num_collision += 1
            if (objID != obsID) and (polysCollide(obj_poly, obs_poly)):
                return True
        return False
        
        
    def bounding_volume_collision_checking(self, objID, pose, obj_poly,ignoring_objIDs=set()):
        '''
        use bounding disc to first do collision checking
        '''
        if (self.depth%self.interval_size)==0:
            obj_bv_dict = self.obj_bv_dict
        else:
            obj_bv_dict = self.last_update_parent_node.obj_bv_dict
        rad = bv_find_r(pose)
        for obsID in self.current_arr.keys():
            if obsID not in ignoring_objIDs:
                dist = sqrt( (pose[0]-obj_bv_dict[obsID][0])**2 + (pose[1]-obj_bv_dict[obsID][1])**2 )
                if dist > rad + obj_bv_dict[obsID][2]:
                    continue
                else:
                    self.num_collision += 1
                    if polysCollide(obj_poly,self.obj_poly_dict[obsID]):
                        return True
        return False

                    
    def space_partitioning_collision_checking(self, objID, obj_poly,ignoring_objIDs=set()):
        '''
        '''
        if (self.depth%self.interval_size)==0:
            cell_obj_table = self.cell_obj_table
        else:
            cell_obj_table = self.last_update_parent_node.cell_obj_table
        overlapping_cells = self.get_overlapping_cells(obj_poly,self.resolution)
        detection_set = set()
        for cell in overlapping_cells:
            detection_set = detection_set.union(cell_obj_table[cell])
        detection_set = detection_set.difference(ignoring_objIDs)
        for obsID in detection_set:
            self.num_collision += 1
            if polysCollide(obj_poly, self.obj_poly_dict[obsID]):
                return True
        return False


    def quadtree_collision_checking(self, objID, obj_poly,ignoring_objIDs=set()):
        ''' collision checking with existing quadtree '''
        if (self.depth%self.interval_size)==0:
            quadtree = self.quadtree
        else:
            quadtree = self.last_update_parent_node.quadtree
        item = self.get_AABB(-1,obj_poly)
        item_set = quadtree.hit(item)
        item_set = item_set.difference(ignoring_objIDs)
        for obsID in item_set:
            self.num_collision += 1
            if polysCollide(obj_poly, self.obj_poly_dict[obsID]):
                return True
        return False


    def get_overlapping_cells(self,obj_poly,resolution):
        '''
        get overlapping cells of bounding box
        '''
        mins = []
        maxs = []
        overlap_cells = []
        for i in range(2):
            mins.append(np.min(obj_poly[:,i]))
            maxs.append(np.max(obj_poly[:,i]))
            overlap_cells.append(
                list(
                    range(
                    int(mins[i]//resolution),int(maxs[i]//resolution+1)
                )
            ))
        return [(x,y) for x in overlap_cells[0] for y in overlap_cells[1]]


    def get_AABB(self,objID,obj_poly):
        return Item(
            objID, 
            np.min(obj_poly[:,0]),np.min(obj_poly[:,1]),
            np.max(obj_poly[:,0]),np.max(obj_poly[:,1])
            )


    def deepcopy_quadtree(self,T:QuadTree):
        ''' deepcopy quadtree '''
        if T == None:
            return T
        # new_T = ujson.loads(ujson.dumps(T))
        # new_T = copy.deepcopy(T)
        new_T = cPickle.loads( cPickle.dumps( T, -1))
        new_T.nw = self.deepcopy_quadtree(T.nw) 
        new_T.ne = self.deepcopy_quadtree(T.ne) 
        new_T.se = self.deepcopy_quadtree(T.se) 
        new_T.sw = self.deepcopy_quadtree(T.sw)
        return new_T
        

    def update_object_info(self,parent_node=None):
        '''
        update data structure in the purpose of collision checking
        obj_poly_dict = {objID:poly}
        obj_bv_dict = {objID:(x,y,r)}
        cell_obj_table = {cellID:set([objID])}
        obj_cell_table = {objID: [cellID]}
        quadtree
        '''
        # print('intended action')
        # print(self.action_from_parent)
        # print('updated object')
        # print(self.updated_objID)
        if parent_node == None:
        # if 1:
            # initialize everything
            
            # obj_poly_dict
            self.obj_poly_dict = {}
            for objID,pose in self.current_arr.items():
                self.obj_poly_dict[objID] = self.generate_object_polygon(pose)
            
            if self.collision_strategy == 'naive':
                self.obj_bv_dict = None
                self.cell_obj_table = None
                self.obj_cell_table = None
                self.quadtree = None
            elif self.collision_strategy == 'bv':
                self.cell_obj_table = None
                self.obj_cell_table = None
                self.quadtree = None
                # obj_bv_dict
                self.obj_bv_dict = {}
                for objID, pose in self.current_arr.items():
                    self.obj_bv_dict[objID] = (pose[0],pose[1],bv_find_r(pose))
            elif self.collision_strategy == 'grid':
                self.obj_bv_dict = None
                self.quadtree = None
                # cell_obj_table, obj_cell_table
                self.cell_obj_table = {}
                self.obj_cell_table = {}
                for x in range(int(self.Width//self.resolution+1)):
                    for y in range(int(self.Height//self.resolution+1)):
                        self.cell_obj_table[(x,y)] = set()
                for objID, poly in self.obj_poly_dict.items():
                    overlapping_cells = self.get_overlapping_cells(poly,self.resolution)
                    for cell in overlapping_cells:
                        self.cell_obj_table[cell].add(objID)
                    self.obj_cell_table[objID] = overlapping_cells
            elif self.collision_strategy == 'quad':
                self.obj_bv_dict = None
                self.cell_obj_table = None
                self.obj_cell_table = None
                items = {}
                for objID, poly in self.obj_poly_dict.items():
                    items[objID] = self.get_AABB(objID,poly)
                self.quadtree = QuadTree(items, self.max_depth,(0,0,self.Width,self.Height))
            else:
                raise ValueError('Strategy not defined.')
        elif (self.depth%self.interval_size)>0:
            ''' no need to update data structures(bv,grid,quadtree), only for polys '''
            # derive from parent node
            self.obj_poly_dict = {k:v for k,v in parent_node.obj_poly_dict.items()}
            self.obj_bv_dict = None
            self.cell_obj_table = None
            self.obj_cell_table = None
            self.quadtree = None
            objID = self.updated_objID
            new_obj_poly = self.generate_object_polygon(self.current_arr[objID])
            self.obj_poly_dict[objID] = new_obj_poly
        elif (self.depth%self.interval_size)==0:
            # derive from parent node
            self.obj_poly_dict = {k:v for k,v in parent_node.obj_poly_dict.items()}
            objID = self.updated_objID
            new_obj_poly = self.generate_object_polygon(self.current_arr[objID])
            self.obj_poly_dict[objID] = new_obj_poly
            if self.collision_strategy != 'naive': # some strutures need to be updated
                parent_node = self.last_update_parent_node
                if parent_node.obj_bv_dict != None:
                    self.obj_bv_dict = {k:v for k,v in parent_node.obj_bv_dict.items()}
                else:
                    self.obj_bv_dict = None
                if parent_node.cell_obj_table != None:
                    self.cell_obj_table = {k:set(list(v)) for k,v in parent_node.cell_obj_table.items()}
                    self.obj_cell_table = {k:list(v) for k,v in parent_node.obj_cell_table.items()}
                else:
                    self.cell_obj_table = None
                    self.obj_cell_table = None
                self.quadtree = self.deepcopy_quadtree(parent_node.quadtree)
                if self.collision_strategy == 'naive':
                    pass
                elif self.collision_strategy == 'bv':
                    for objID in self.updating_obj_set:
                        pose = self.current_arr[objID]
                        self.obj_bv_dict[objID] = (pose[0],pose[1],bv_find_r(pose))
                elif self.collision_strategy == 'grid':
                    for objID in self.updating_obj_set:
                        prev_overlapping_cells = self.obj_cell_table[objID]
                        new_overlapping_cells = self.get_overlapping_cells(new_obj_poly,self.resolution)
                        self.obj_cell_table[objID] = new_overlapping_cells
                        for cell in prev_overlapping_cells:
                            self.cell_obj_table[cell].remove(objID)
                        for cell in new_overlapping_cells:
                            self.cell_obj_table[cell].add(objID)
                elif self.collision_strategy == 'quad':
                    for objID in self.updating_obj_set:
                        old_item = self.get_AABB(objID,parent_node.obj_poly_dict[objID])
                        new_item = self.get_AABB(objID, self.obj_poly_dict[objID])
                        self.quadtree.remove(old_item)
                        self.quadtree.insert(new_item)
                else:
                    print(self.collision_strategy)
                    raise ValueError('Strategy not defined.')
            # self.plot_collision_settings()
        else:
            raise ValueError('Unexpected situation when updating object info')
        # self.plot_collision_settings()
            
                
    def plot_collision_settings(self):
        print(self.collision_strategy)
        show_quadtree(self.current_arr,self.quadtree,show_text=False)
        return
        if self.collision_strategy == 'bv':
            show_bounding_volume(
                self.current_arr, self.obj_bv_dict,
                self.Width, self.Height, show_text=True
            )
        elif self.collision_strategy == 'grid':
            show_grid( self.current_arr, self.cell_obj_table, self.resolution)
        elif self.collision_strategy == 'naive':
            print("It is passed")
            pass
        else:
            raise ValueError('Strategy not defined.')
        return 
            
            
    def test_buffer_suggestion(self,strategy='quad'):
        ''' test for quadtree buffer suggestion '''
        objID = choice(list(self.current_arr.keys()))
        Isvalid = True
        if strategy == 'quad':
            new_position = self.quad_tree_buffer_suggestion(self.current_arr[objID])
        else:
            new_position = self.naive_buffer_suggestion(self.current_arr[objID])
        new_poly = self.generate_object_polygon(new_position)
        if (
            self.is_object_in_workspace(new_poly)
            ) and (
            not self.obj_obj_collision_checking(
                objID,new_position,self.collision_strategy,[]
                )):
            Isvalid = True
        else:
            Isvalid = False
        # test_buffer_allocation(
        #     self.current_arr, objID, new_position, self.quadtree
        # )
        return Isvalid
        

def test_collision_checker():
    test_obj_1 = [100, 100, 0, 'cuboid', 20, 5, 1]
    test_obj_2 = [ [100, 120, 0, 'cuboid', 20, 5, 1] ]

    print("test_obj_1", test_obj_1)
    print("test_obj_2", test_obj_2)
    # print("Collision - ", Collision_Check(test_obj_1, test_obj_2) )
    
    
    