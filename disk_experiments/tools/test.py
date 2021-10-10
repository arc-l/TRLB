import copy

from Unlabeled_DFS_DP import DFS_DP_Search

Graph = {0: set([8]), 2: set([0, 3]), 3: set([2]), 7: set([0, 3]), 8: set([])}
limit, ordering = DFS_DP_Search(Graph)
print limit
print ordering