from math import log
from itertools import combinations
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from tools.SCC_decomposition import SCC_decomposition, Graph_Decomposition, Topological_SCC_ordering
from tools.util import deepcopy_graph, deepcopy_dict

class DP_with_TB(object):
    # DP, considering AB as the primary objective and TB is the secondary
    def __init__(self, graph, Weights):
        self.n = len(graph)
        self.graph = deepcopy_graph(graph)
        self.Weights = Weights
        self.action_list = [] # output
        self.total_buffer = self.dynamic_programming()
        # print "DP", self.active_buffer, self.total_buffer

    def dynamic_programming(self):
        old_table = {}
        new_table = {}
        parents = {}
        old_table[0] = [set(),0] #[buffer, active_buffer, total_buffer]
        for i in range(self.n):
            for obj_set in combinations(self.graph.keys(), i+1):
                index = 0
                for e in obj_set:
                    index += 1<<e

                # current buffer
                first_obj = obj_set[0] # current moving object
                old_index = index - (1<<first_obj)
                new_buffer = (old_table[old_index][0].union(set(self.graph[first_obj]))).difference(set(obj_set)) # new set of buffer objects

                # total buffer up to now
                total_buffer = float('inf')
                for obj in obj_set:
                    old_index = index - (1<<obj)
                    additional_buffer = new_buffer.difference(old_table[old_index][0])
                    current_total_buffer = sum([self.Weights[k] for k in additional_buffer]) + old_table[old_index][1]
                    if (current_total_buffer<total_buffer):
                        total_buffer = current_total_buffer
                        parents[index] = old_index
                
                old_index = parents[index]
                obj = int(log(index-old_index,2))
                if obj in old_table[old_index][0]:
                    new_buffer.add(obj)
                new_table[index] = [new_buffer, total_buffer]
            old_table = {k:[set(list(v[0])), v[1]] for k, v in new_table.items()}
            new_table = {}
        
        obj_ordering_list = []
        current_node = index
        while current_node in parents:
            old_node = parents[current_node]
            current_obj = int(log(current_node-old_node, 2))
            obj_ordering_list.append((current_obj))
            current_node = old_node
        obj_ordering_list = list(reversed(obj_ordering_list))
        
        buffer_set = []
        for i in range(self.n):
            for j in range(i+1, self.n):
                if obj_ordering_list[j] in self.graph[obj_ordering_list[i]]:
                    if obj_ordering_list[j] not in buffer_set:
                        self.action_list.append((obj_ordering_list[j],'b'))
                        buffer_set.append(obj_ordering_list[j])
            self.action_list.append((obj_ordering_list[i], 'g'))
        # print actions
        return old_table[index][1]

def FVS_Plan_Search(Dgraph, Weights=None,return_FVS=False):
    if Weights == None:
        Weights = {k:1 for k in Dgraph.keys()}
    action_sequence = []
    FVS = 0
    # SCC Decomposition
    Partition = SCC_decomposition(Dgraph, Dgraph.keys())
    SCC_ordering = Topological_SCC_ordering(Dgraph, Partition)
    for SCC_index in SCC_ordering:
        SCC = Partition[SCC_index]
        SCC_list = list(SCC)
        new_Graph,M = Graph_Decomposition(Dgraph, SCC_list,return_mapping=True)
        new_weights = {v:Weights[k] for k,v in M.items()}
        solver = DP_with_TB(new_Graph,new_weights)

        # transform the action sequence
        real_action_sequence = []
        for action in solver.action_list:
            real_action = (SCC_list[action[0]], action[1])
            real_action_sequence.append(real_action)
        action_sequence = action_sequence + real_action_sequence
        FVS += solver.total_buffer
    if return_FVS:
        return action_sequence, FVS
    else:
        return action_sequence


if __name__ == '__main__':
    Dgraph = {0:{1},1:{0}, 2:{3}, 3:{2}}
    Weights = {0:1000,1:1000,2:2000,3:3000}
    seq, fvs = FVS_Plan_Search(Dgraph, Weights,return_FVS=True)
    print('fvs',fvs)

