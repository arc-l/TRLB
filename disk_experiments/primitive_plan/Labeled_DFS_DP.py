from random import shuffle
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from tools.util import set2tuple, deepcopy_graph
from tools.SCC_decomposition import SCC_decomposition, Graph_Decomposition, Topological_SCC_ordering

class DFS_DP(object):
    '''
    Input: 
    dependency graph DG: DG[objID]=set(obj1, obj2, ...)
    MRB limit: int

    Output:
    self.ordering: object ordering sorted by the time when objects arrive at the goal
    self.action_sequence: (objID, 'g'/'b') 'g' means to the goal, 'b' means to the buffer

    self.ordering = [] when problem is infeasible
    '''
    def __init__(self, DG, limit):
        self.limit = limit
        self.DG = deepcopy_graph(DG)
        self.n = len(self.DG)
        self.objects = set2tuple(self.DG.keys())

        # output
        self.ordering = [] # object ordering sorted by the time when objects arrive at the goal
        self.action_sequence = [] # (objID, 'g'/'b') 'g' means to the goal, 'b' means to the buffer
        self.TB = 0
        self.IsMonotone = False
        self.num_node = 0

        self.Dynamic_Programming()

        if len(self.ordering) > 0:
            self.IsMonotone = True

    def Dynamic_Programming(self):
        '''
        tree node: ordered tuple of goal objects
        '''
        parents = {}
        object_ordering = []
        self.explored = set()
        self.buffer_objects = {}
        queue = [()] 
        self.explored.add(())
        self.buffer_objects[()] = set()
        Found = False
        while((len(queue)>0) and (not Found)):
            old_node = queue.pop(-1)
            for next_object in self.next_object(old_node):
                new_node = set2tuple(set(old_node).union({next_object}))
                if new_node in self.explored:
                    continue
                if self.valid_transition(old_node, next_object, new_node):
                    parents[new_node] = old_node
                    queue.append(new_node)
                    if new_node == self.objects:
                        Found = True
                        break

        self.num_node = len(parents)+1

        if self.objects in self.explored:
            current_node = self.objects
            while current_node in parents:
                parent_node = parents[current_node]
                last_object = list(set(current_node).difference(parent_node))[0]
                object_ordering.append(last_object)
                current_node = parent_node

            self.TB = 0
            self.ordering = list(reversed(object_ordering))
            self.action_sequence = [(x,'g') for x in self.ordering]
            for (i, obs) in enumerate(self.ordering):
                for (j, obj) in enumerate(self.ordering[:i]):
                    if obs in self.DG[obj]:
                        self.TB += 1
                        index = self.action_sequence.index((obj, 'g'))
                        self.action_sequence.insert(index, (obs, 'b'))
                        break

            # print "ordering", self.ordering
            # print "actions", self.action_sequence
    
    def next_object(self, old_node):
        obj_list = list(self.objects)
        shuffle(obj_list)
        for i in obj_list:
            if i in old_node:
                pass
            else:
                yield i

    def valid_transition(self,old_node, next_object, new_node):
        old_buffer = self.buffer_objects[old_node]
        new_objects = set(self.DG[next_object]).difference(set(old_node))
        new_buffer = (old_buffer.union(new_objects)).difference(set([next_object]))

        if next_object in old_buffer:
            transition_cost = max(len(old_buffer), len(new_buffer)+1)
        else:
            transition_cost = max(len(old_buffer), len(new_buffer))

        if len(new_buffer) > self.limit:
            self.explored.add(new_node) # dead_end
            return False

        if transition_cost <= self.limit:
            self.buffer_objects[new_node] = new_buffer
            self.explored.add(new_node)
            return True
        else:
            return False


def DFS_DP_Search(Dgraph, RB=0):
    '''
    Generate multiple solutions with RB from MRB to infinite.
    Input: 
    dependency graph DG: DG[objID]=set(obj1, obj2, ...)
    RB: Running Buffer lower bound(=0)

    Output:
    RB and action sequence[(objID, 'g'/'b')]
    '''
    action_sequence = []
    # SCC Decomposition
    Partition = SCC_decomposition(Dgraph, Dgraph.keys())
    SCC_ordering = Topological_SCC_ordering(Dgraph, Partition)
    RB = 0 # start with 0 running buffers
    max_node = 0
    total_node = 0
    for SCC_index in SCC_ordering:
        SCC = Partition[SCC_index]
        SCC_list = list(SCC)
        new_Graph = Graph_Decomposition(Dgraph, SCC_list)
        solver = DFS_DP(new_Graph, RB)
        max_node = max(max_node, solver.num_node)
        total_node += solver.num_node
        while not solver.IsMonotone:
            RB += 1
            solver = DFS_DP(new_Graph, RB)
            max_node = max(max_node, solver.num_node)
            total_node += solver.num_node
        # transform the action sequence
        real_action_sequence = []
        for action in solver.action_sequence:
            real_action = (SCC_list[action[0]], action[1])
            real_action_sequence.append(real_action)
        action_sequence = action_sequence + real_action_sequence
    return RB, action_sequence, max_node, total_node

        
