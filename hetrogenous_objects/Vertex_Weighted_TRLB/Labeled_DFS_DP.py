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
    def __init__(self, DG, Weights, limit):
        self.limit = limit
        self.DG = deepcopy_graph(DG)
        self.weights = Weights
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
                        self.TB += self.obj_buffer_size(obs)
                        index = self.action_sequence.index((obj, 'g'))
                        self.action_sequence.insert(index, (obs, 'b'))
                        break

            # print "ordering", self.ordering
            # print "actions", self.action_sequence
    
    def next_object(self, old_node):
        obj_list = list(self.objects)
        shuffle(obj_list)
        # print("obj_list", obj_list)
        for i in obj_list:
            if i in old_node:
                pass
            else:
                yield i

    def valid_transition(self,old_node, next_object, new_node):
        old_buffer = self.buffer_objects[old_node]
        new_objects = set(self.DG[next_object]).difference(set(old_node))
        new_buffer = (old_buffer.union(new_objects)).difference(set([next_object]))

        old_buffer_size = self.set_buffer_size(old_buffer)
        new_buffer_size = self.set_buffer_size(new_buffer)

        if next_object in old_buffer:
            transition_cost = max(old_buffer_size, new_buffer_size+self.obj_buffer_size(next_object))
        else:
            transition_cost = max(old_buffer_size, new_buffer_size)

        if new_buffer_size > self.limit:
            self.explored.add(new_node) # dead_end
            return False

        if transition_cost <= self.limit:
            self.buffer_objects[new_node] = new_buffer
            self.explored.add(new_node)
            return True
        else:
            return False


    def set_buffer_size(self,obj_set:set):
        '''
        weighted buffer size
        '''
        return sum([self.obj_buffer_size(objID) for objID in obj_set])
        
    
    def obj_buffer_size(self,objID):
        return self.weights[objID]


def DFS_DP_Search(Dgraph, Weights=None, RB=0,mode='increase'):
    '''
    modes: increase, decrease
    '''
    if mode == 'increase':
        return increasing_search(Dgraph, Weights=Weights, RB=RB)
    elif mode == 'decrease':
        return decreasing_search(Dgraph, Weights=Weights, RB=RB)


def decreasing_search(Dgraph, Weights=None, RB=0):
    '''
    decrease from a upper bound
    '''
    if Weights == None:
        Weights = {k:1 for k in Dgraph.keys()}
    action_sequence = []
    # SCC Decomposition
    Partition = SCC_decomposition(Dgraph, Dgraph.keys())
    SCC_ordering = Topological_SCC_ordering(Dgraph, Partition)
    # print("Partition", Partition, "\n\nSCC_ordering", SCC_ordering)
    MRB = 0
    max_node = 0
    total_node = 0
    for SCC_index in SCC_ordering:
        # get a feasible plan and a upper bound of RB
        RB = 0 # start with 0 running buffers
        SCC = Partition[SCC_index]
        SCC_list = list(SCC)
        new_Graph, M = Graph_Decomposition(Dgraph, SCC_list, return_mapping=True)
        new_weights = {v:Weights[k] for k,v in M.items()}
        new_one_weights = {v:1 for k,v in M.items()}
        solver = DFS_DP(new_Graph, new_one_weights, RB)
        max_node = max(max_node, solver.num_node)
        total_node += solver.num_node
        while not solver.IsMonotone:
            RB += 1
            solver = DFS_DP(new_Graph, new_one_weights, RB)
            max_node = max(max_node, solver.num_node)
            total_node += solver.num_node
        # record temp solution
        temp_action_sequence = solver.action_sequence
        
        # decrease
        RB = get_real_RB(temp_action_sequence, new_weights)
        while 1:
            RB -= 1
            solver = DFS_DP(new_Graph, new_weights, RB)
            max_node = max(max_node, solver.num_node)
            total_node += solver.num_node
            if solver.IsMonotone:
                temp_action_sequence = solver.action_sequence
            else:
                RB += 1
                break
        
        
        MRB = max(RB, MRB)
        # transform the action sequence
        real_action_sequence = []
        for action in temp_action_sequence:
            real_action = (SCC_list[action[0]], action[1])
            real_action_sequence.append(real_action)
        action_sequence = action_sequence + real_action_sequence
    
    # print("RB", RB, "\n\naction_sequence", action_sequence, "\n\nmax_node", max_node, "\n\ntotal_node", total_node)
    return MRB, action_sequence, max_node, total_node


def get_real_RB(action_sequence, Weights):
    ''' get the weighted RB size from a sequence '''
    current_buffer_set = set()
    RB = 0
    current_RB = 0
    for objID, mode in action_sequence:
        if mode == 'b':
            current_buffer_set.add(objID)
            current_RB += Weights[objID]
            RB = max(RB, current_RB)
        elif objID in current_buffer_set:
            current_buffer_set.remove(objID)
            current_RB -= Weights[objID]
    return RB


def increasing_search(Dgraph, Weights=None, RB=0):
    '''
    Generate multiple solutions with RB from MRB to infinite.
    Input: 
    dependency graph DG: DG[objID]=set(obj1, obj2, ...)
    Weights: Weights[objID] = 2
    RB: Running Buffer lower bound(=0)

    Output:
    RB and action sequence[(objID, 'g'/'b')]
    '''
    if Weights == None:
        Weights = {k:1 for k in Dgraph.keys()}
    action_sequence = []
    # SCC Decomposition
    Partition = SCC_decomposition(Dgraph, Dgraph.keys())
    SCC_ordering = Topological_SCC_ordering(Dgraph, Partition)
    # print("Partition", Partition, "\n\nSCC_ordering", SCC_ordering)
    MRB = 0
    max_node = 0
    total_node = 0
    for SCC_index in SCC_ordering:
        RB = 0 # start with 0 running buffers
        SCC = Partition[SCC_index]
        SCC_list = list(SCC)
        new_Graph, M = Graph_Decomposition(Dgraph, SCC_list, return_mapping=True)
        new_weights = {v:Weights[k] for k,v in M.items()}
        solver = DFS_DP(new_Graph, new_weights, RB)
        max_node = max(max_node, solver.num_node)
        total_node += solver.num_node
        while not solver.IsMonotone:
            RB += 1
            solver = DFS_DP(new_Graph, new_weights, RB)
            max_node = max(max_node, solver.num_node)
            total_node += solver.num_node
        MRB = max(RB, MRB)
        # transform the action sequence
        real_action_sequence = []
        for action in solver.action_sequence:
            real_action = (SCC_list[action[0]], action[1])
            real_action_sequence.append(real_action)
        action_sequence = action_sequence + real_action_sequence
    
    # print("RB", RB, "\n\naction_sequence", action_sequence, "\n\nmax_node", max_node, "\n\ntotal_node", total_node)
    return MRB, action_sequence, max_node, total_node
        

if __name__ == "__main__":
    pass
