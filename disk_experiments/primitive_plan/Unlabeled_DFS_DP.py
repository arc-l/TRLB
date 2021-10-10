import sys
import os
from math import floor, log
from random import shuffle

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from tools.util import set2tuple, deepcopy_graph


class DFS_DP(object):
    '''
    Input: 
    dependency graph DG: DG[objID]=set(obj1, obj2, ...)
    MRB limit: int

    Output:
    self.ordering: goal pose filling order
    self.IsMonotone

    self.ordering = [] when problem is infeasible
    '''
    def __init__(self, Bi_Partite, limit):
        # DG: key: goal index; value: a set of neighbor starts
        self.limit = limit
        self.goal2start = deepcopy_graph(Bi_Partite)
        self.start2goal = self.reverse_graph(Bi_Partite)
        self.n = len(self.goal2start)
        self.goals = tuple(sorted(self.goal2start.keys()))
        
        # Solution
        self.ordering = []
        self.IsMonotone = False

        self.Dynamic_Programming()

        if len(self.ordering) > 0:
            self.IsMonotone = True

    def Dynamic_Programming(self):
        '''
        tree node: ordered tuple of filled goals
        '''
        parents = {}
        node_ordering = []
        self.explored = set()
        self.explored.add(())
        queue = []

        #first prune
        pruned_goals, current_AB, initial_AB = self.graph_pruning(tuple())
        if max(current_AB, initial_AB)<=self.limit:
            if pruned_goals != ():
                parents[pruned_goals] = ()
            self.explored.add(pruned_goals)
            for i in self.next_object(pruned_goals):
                queue.append((pruned_goals, i)) #(previous_set, new_goal)
        else:
            print( "impossible situation")
            return
            
        Found = False
        while((len(queue)>0) and (not Found)):
            plan = queue.pop(-1)
            old_node = plan[0]
            next_object = plan[1]
            new_node = set2tuple(set(old_node).union({next_object}))
            if new_node in self.explored:
                continue
            pruned_goals, current_AB, initial_AB = self.graph_pruning(new_node)
            if pruned_goals in self.explored:
                continue
            if max(current_AB, initial_AB)<=self.limit:
                parents[new_node] = old_node
                if pruned_goals != new_node:
                    parents[pruned_goals] = new_node
                self.explored.add(new_node)
                self.explored.add(pruned_goals)
                if pruned_goals == self.goals:
                    Found = True
                    break
                for i in self.next_object(pruned_goals):
                    queue.append((pruned_goals,i))
        
        if self.goals in self.explored:
            current_node = self.goals
            while current_node in parents:
                parent_node = parents[current_node]
                last_objects = list(set(current_node).difference(parent_node))
                node_ordering.append(last_objects)
                current_node = parent_node
            
            node_ordering = list(reversed(node_ordering))
            self.ordering = []
            for p in node_ordering:
                if len(p) == 1:
                    self.ordering.append(p[0])
                else:
                    self.ordering = self.ordering + self.free_nodes_detection(set2tuple(set(self.ordering)))
            # print self.ordering
            start_ordering, action_ordering = self.start_order_generation(self.ordering)
            # print start_ordering
    
    def start_order_generation(self,goal_ordering):
        start_ordering = []
        action_ordering = []
        buffer = []
        available_goals = 0
        for g in goal_ordering:
            available_goals += 1
            start_set = self.goal2start[g]
            moving_list = list((start_set.difference(set(start_ordering))).difference(set(buffer)))
            while (available_goals>0) and (len(moving_list)>0):
                s = moving_list.pop(0)
                start_ordering.append(s)
                action_ordering.append((s, 'g'))
                available_goals -= 1
            while (available_goals>0) and (len(buffer)>0):
                s = buffer.pop(0)
                start_ordering.append(s)
                action_ordering.append((s, 'g'))
                available_goals -= 1
            if len(moving_list) >0:
                buffer = buffer + moving_list
                for s in moving_list:
                    action_ordering.insert(len(action_ordering)-1, (s, 'b'))
        # print action_ordering

        # print "TB", len(action_ordering)-len(start_ordering)
        # print "RB", self.limit

        return start_ordering, action_ordering
    
    def free_nodes_detection(self, g_tuple):
        '''
        Based on the current set of goals, what kinds of objects can be deal with automatically. Return a removing sequence.
        '''
        partial_ordering = []
        pruning_goal2start = deepcopy_graph(self.goal2start) # the resulting graph after pruning
        pruning_start2goal = deepcopy_graph(self.start2goal) # the resulting graph after pruning
        g_set = set(g_tuple)
        transition_AB = -float('inf') # the max active buffer during pruning
        GO_ON = True # still have the potential to go on pruning
        while GO_ON:
            moved_start = set()
            for g in g_set:
                moved_start = moved_start.union( pruning_goal2start[g])
            transition_AB = max(transition_AB, len(moved_start)-len(g_set)) # the active buffer after the g_set
            for s in moved_start:
                del pruning_start2goal[s]
            new_g_set = set()
            for g in list(pruning_goal2start.keys()):
                if g in g_set:
                    del pruning_goal2start[g]
                    continue
                pruning_goal2start[g] = pruning_goal2start[g].difference(moved_start)
                if len(pruning_goal2start[g])==0:
                    new_g_set.add(g)
                elif len(pruning_goal2start[g])==1:
                    new_g_set.add(g)
            partial_ordering = partial_ordering + list(new_g_set)
            g_set = new_g_set
            if len(g_set) == 0:
                GO_ON = False
        
        return partial_ordering


    def graph_pruning(self, g_tuple):
        '''
        Based on the current set of goals, what kinds of objects can be deal with automatically.
        '''
        pruning_goal2start = deepcopy_graph(self.goal2start) # the resulting graph after pruning
        pruning_start2goal = deepcopy_graph(self.start2goal) # the resulting graph after pruning
        g_set = set(g_tuple)
        transition_AB = -float('inf') # the max active buffer during pruning
        GO_ON = True # still have the potential to go on pruning
        while GO_ON:
            moved_start = set()
            for g in g_set:
                moved_start = moved_start.union( pruning_goal2start[g])
            transition_AB = max(transition_AB, len(moved_start)-len(g_set)) # the active buffer after the g_set
            for s in moved_start:
                del pruning_start2goal[s]
            new_g_set = set()
            for g in list(pruning_goal2start.keys()):
                if g in g_set:
                    del pruning_goal2start[g]
                    continue
                pruning_goal2start[g] = pruning_goal2start[g].difference(moved_start)
                if len(pruning_goal2start[g])==0:
                    new_g_set.add(g)
                elif len(pruning_goal2start[g])==1:
                    new_g_set.add(g)
            g_set = new_g_set
            if len(g_set) == 0:
                GO_ON = False
        
        num_g = len(pruning_goal2start)
        num_s = len(pruning_start2goal)
        current_AB = num_g - num_s
        current_goal_tuple = set2tuple(set(self.goal2start.keys()).difference(pruning_goal2start.keys()))
        return ( current_goal_tuple, current_AB, transition_AB)

    def set2index(self, set_):
        index = 0
        for e in set_:
            index += (1<<e)
        return index

    def index2set(self, index):
        set_ = set()
        while index != 0:
            e = int(floor(log( index, 2)))
            set_.add(e)
            index -= (1<<e)
        return set_

            

    def reverse_graph(self, graph):
        r_graph = {}
        for key in graph.keys():
            r_graph[key] = set() # the reverse has the same number of vertices
        for (key, value) in graph.items():
            for v in value:
                r_graph[v].add(key)
        return r_graph

    def next_object(self, old_node):
        obj_list = list(self.goals)
        shuffle(obj_list)
        for i in obj_list:
            if i in old_node:
                pass
            else:
                yield i


def DFS_DP_Search(Bi_Partite):
    n = len(Bi_Partite)
    for limit in range(n):
        Solver = DFS_DP(Bi_Partite, limit)
        if Solver.IsMonotone:
            return limit, Solver.ordering
