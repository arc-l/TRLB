from random import choice
import time
import copy
import threading
import numpy as np
from Modified_Rewards_Python_MCTS.Node import Node

class Py_MCTS(object):
    def __init__(
        self, source_arr, target_arr, radius, 
        Width, Height, heuristic=0,UCB_scalar=1.0,
        collision_check='naive',fineness=10,max_depth=4,
        buffer_strategy = 'naive',interval_size = 3,
        ) -> None:
        '''
        'heuristic':(int 0/2/3/4): different heuristic formulation
        'UCB_scalar':(float) UCB coefficients
        'collision_check':(str 'naive'/'bv'/'grid'/'quad') collision check strategy
        'fineness':(int) number of cells on both x and y in grid construction
        'max_depth':(int) max depth of the quad tree
        'buffer_strategy':(str 'naive'/'quad') buffer suggestion strategy
        'interval_size':(int) the interval between layers that store collision check structures
        '''
        self.source_arr = self.reweighting(source_arr)
        self.target_arr = self.reweighting(target_arr)

        self.settings = {
            'UCB_scalar':UCB_scalar,
            'collision_check':collision_check,
            'fineness':fineness, # for grid construction
            'max_depth':max_depth, # depth of the quad tree
            'buffer_strategy':buffer_strategy,
            'interval_size':interval_size # the interval between layers that store collision check structures
        }


        self.radius = radius
        self.Width = Width
        self.Height = Height
        self.root = Node(
            0, self.source_arr, self.target_arr, self.radius, 
            None, Width, Height,action_from_parent=None,
            updated_objID=None,**self.settings
            )
        self.heuristic = heuristic

    
        # outputs
        self.action_list = []
        self.isfeasible = False
        self.time_coll_setup = 0
        self.time_coll_check = 0
        self.num_collision = 0
        self.num_iter = 0
        # return

        Max_iter = 1e8
        num_iter = 0


        # use efforts instead of visited time to do penalty
        # use needed efforts to give rewards 
        total_rewards = sum([e[6] for e in self.source_arr.values()])
        while (num_iter < Max_iter):
            # if (num_iter % 1000)==0:
            #     print(num_iter)
            num_iter += 1
            current_node = self.selection()
            action, moved_obj, new_position = current_node.expansion()
            if new_position != None:
                new_node = self.Move(num_iter, action, moved_obj, new_position, current_node)
            else:
                new_node = Node(
                    num_iter,current_node.current_arr, self.target_arr, 
                    self.radius, current_node, self.Width, self.Height,
                    action_from_parent=action, updated_objID=moved_obj, 
                    **self.settings
                    )
            current_node.children[action] = new_node
            reward = self.reward_detection(new_node)
            self.back_propagation(new_node, reward)
            if len(new_node.unvisited_actions) == 0:
                self.isfeasible = True
                self.construct_plan(new_node)
                break
        # print('finish')
        
        # recording
        self.time_coll_setup, self.time_coll_check, \
        self.num_collision = self.DFS_for_record(self.root)
        self.num_iter = num_iter


    def reweighting(self,arr):
        '''
        reweighting an arr to [0,1]
        '''
        old_weights = { k:arr[k][6] for k in arr.keys()}
        max_weight = max(list(old_weights.values()))
        for k in arr.keys():
            arr[k][6] /= max_weight
        return arr


    def construct_plan(self, node: Node):
        self.action_list = []
        current_node = node
        while current_node.parent != None:
            parent_node = current_node.parent
            moved_object = current_node.updated_objID
            # print(current_node.nodeID, action)
            # current_node.show_arrangement()
            self.action_list.append(
                (moved_object, parent_node.current_arr[moved_object], current_node.current_arr[moved_object])
            )
            current_node = parent_node
        self.action_list.reverse()
        
        # print('try removing useless actions')
        # remove useless actions
        Epsilon = 1.0
        self.action_list = [(action[0], tuple(action[1]), tuple(action[2])) for action in self.action_list]
        ii = 0
        while ii <= (len(self.action_list)-2):
            action = self.action_list[ii]
            p1 = action[1]
            p2 = action[2]
            if (np.linalg.norm(np.array(p1[:3])-np.array(p2[:3]))<Epsilon): # tiny moves
                self.action_list.remove(action)
                # print('delete tiny motions')
            elif action[0] == self.action_list[ii+1][0]: # move the same object twice in a row
                final_pose = self.action_list[ii+1][2]
                del self.action_list[ii+1]
                del self.action_list[ii]
                self.action_list.insert(
                    ii, (action[0], p1, final_pose)
                )
                # print('delete successive motions')
            else:
                ii += 1


    def selection(self):
        current_node = self.root
        while len(current_node.unvisited_actions) == 0:
            current_node = current_node.UCB(use_heuristic=self.heuristic)
        return current_node


    def Move(self, nodeID, action, obj, target, current_node):
        current_arr = {i:p for i,p in current_node.current_arr.items() if i != obj}
        current_arr[obj] = target
        new_node = Node(
            nodeID, current_arr, self.target_arr, 
            self.radius, current_node, self.Width, self.Height,
            action_from_parent=action, updated_objID=obj,
            **self.settings
            )
        return new_node


    def reward_detection(self,node):
        Epsilon = 1e0
        reward = 0
        for ID, p in node.current_arr.items():
            tp = self.target_arr[ID]
            if (p[0]-tp[0])**2+(p[1]-tp[1])**2<=Epsilon:
                reward += p[6]
        return reward


    def back_propagation(self, node:Node, reward):
        current_node = node
        accumulative_effort = 0
        while current_node != None:
            if node.updated_objID == None:
                accumulative_effort += 1
            else:
                accumulative_effort += self.source_arr[node.updated_objID][6]
            current_node.visited_time += 1 
            current_node.needed_efforts += accumulative_effort # use needed efforts rather than visited times to penalize action
            current_node.total_reward += reward
            current_node = current_node.parent
            
            
    def DFS_for_record(self,node:Node):
        '''
        DFS to collect records
        '''
        time_coll_setup = node.time_coll_setup
        time_coll_check = node.time_coll_check
        num_collision = node.num_collision
        for child in node.children.values():
            child_time_coll_setup, child_time_coll_check, \
            child_counter_coll = self.DFS_for_record(child)
            
            time_coll_setup += child_time_coll_setup
            time_coll_check += child_time_coll_check
            num_collision += child_counter_coll
        return time_coll_setup, time_coll_check, num_collision
            
            
    def buffer_allocation_experiment(self):
        ''' buffer allocation performance '''
        self.avg_quad_buffer = 0
        self.avg_naive_buffer = 0
        
        quad_tree_counter_list = []
        for _ in range(50):
            counter = 0
            while 1:
                Isvalid = self.root.test_buffer_suggestion(strategy='quad')
                if Isvalid:
                    counter += 1
                    # print(counter)
                    break
                else:
                    counter += 1
            quad_tree_counter_list.append(counter)
        
        naive_tree_counter_list = []
        for _ in range(50):
            counter = 0
            while 1:
                Isvalid = self.root.test_buffer_suggestion(strategy='naive')
                if Isvalid:
                    counter += 1
                    # print(counter)
                    break
                else:
                    counter += 1
            naive_tree_counter_list.append(counter)
        self.avg_quad_buffer = np.average(quad_tree_counter_list)
        self.avg_naive_buffer = np.average(naive_tree_counter_list)