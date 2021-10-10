from random import shuffle
from tools.SCC_decomposition import SCC_decomposition, Graph_Decomposition, Topological_SCC_ordering


def random_order(graph):
    action_list = []
    obj_ordering_list = list(graph.keys())
    shuffle(obj_ordering_list)
    # print(obj_ordering_list)

    # # SCC Decomposition
    # Partition = SCC_decomposition(graph, graph.keys())
    # SCC_ordering = Topological_SCC_ordering(graph, Partition)
    # RB = 0 # start with 0 running buffers
    # for SCC_index in SCC_ordering:
    #     SCC = Partition[SCC_index]
    #     SCC_list = list(SCC)
    #     shuffle(SCC_list)
    #     obj_ordering_list.extend( SCC_list)
        
    

    n = len(obj_ordering_list)
        
    buffer_set = []
    for i in range(n):
        for j in range(i+1, n):
            if obj_ordering_list[j] in graph[obj_ordering_list[i]]:
                if obj_ordering_list[j] not in buffer_set:
                    action_list.append((obj_ordering_list[j],'b'))
                    buffer_set.append(obj_ordering_list[j])
        action_list.append((obj_ordering_list[i], 'g'))
    return action_list