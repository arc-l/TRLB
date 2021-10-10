from ujson import loads, dumps

class First_DFS(object):
    def __init__(self, Graph, vertex_set):
        self.Graph = Graph
        self.remaining_vertices = vertex_set
        self.Finish_queue = []
        self.DFS()

    def DFS(self):
        while len(self.remaining_vertices)>0:
            stack = [self.remaining_vertices.pop()]
            while len(stack)>0:
                old_vertex = stack.pop(-1)
                self.DFS_rec(old_vertex)
                self.Finish_queue.append(old_vertex)
        self.Finish_queue = list(reversed(self.Finish_queue)) # decreasing finishing

    def DFS_rec(self, old_vertex):
        for new_vertex in self.Graph[old_vertex]:
            if new_vertex in self.remaining_vertices:
                self.remaining_vertices.remove(new_vertex)
                self.DFS_rec(new_vertex)
                self.Finish_queue.append(new_vertex)

def Graph_Decomposition(G, V):
    M = {}
    for i in range(len(V)):
        M[V[i]] = i
    new_Graph = {}
    for key in V:
        new_Graph[M[key]] = []
        for value in V:
            if value in G[key]:
                new_Graph[M[key]].append(M[value])
    return new_Graph

def Transpose_Graph(G):
    GT = {}
    for i in G.keys():
        GT[i] = []
    for out_deg in G:
        for in_deg in G[out_deg]:
            GT[in_deg].append(out_deg)
    return GT

def Second_DFS(Graph, finish_queue):
    Partition = []
    remaining_vertices = loads(dumps(finish_queue))
    while len(remaining_vertices)>0:
        new_tree = set()
        root = remaining_vertices.pop(0)
        stack = [root]
        new_tree.add(root)
        while len(stack)>0:
            old_vertex = stack.pop(-1)
            for new_vertex in Graph[old_vertex]:
                if new_vertex not in remaining_vertices:
                    continue
                stack.append(new_vertex)
                new_tree.add(new_vertex)
                remaining_vertices.remove(new_vertex)
        Partition.append(new_tree)
    return Partition


def SCC_decomposition(Graph, V):
    F_DFS = First_DFS(Graph, set(V))
    Partition = Second_DFS(Transpose_Graph(Graph), F_DFS.Finish_queue)
    return Partition

def Topological_SCC_ordering(Graph, Partition):
    V2SCC = {}
    SCC_Graph = {}
    for index, SCC in enumerate(Partition):
        SCC_Graph[index] = set()
        for v in SCC:
            V2SCC[v] = index
            SCC_Graph[index].union(Graph[v])
    
    # Construct the component graph
    for index in range(len(Partition)):
        out_deg_v = SCC_Graph[index]
        SCC_Graph[index] = set()
        for another_index, SCC in enumerate(Partition):
            if index == another_index:
                continue
            if len(out_deg_v.intersection(SCC)):
                SCC_Graph[index].add(another_index)
    
    # topological order of the components
    SCC_ordering = [] # a reverse of the feasible ordering
    for SCC, out_deg in SCC_Graph.items():
        stopping_point = -1
        for i, c in enumerate(SCC_ordering):
            if c in out_deg:
                stopping_point = i
                break
        if stopping_point == -1:
            SCC_ordering.append(SCC)
        else:
            SCC_ordering.insert(stopping_point, SCC)
    SCC_ordering.reverse() # reverse it to the correct order
    return SCC_ordering
    