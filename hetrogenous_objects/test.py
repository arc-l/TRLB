import copy
import time
import ujson

from methods_with_timeout import timeout_MCTS_h
from tools.util import get_arrangements, limit_memory
import gc


def test_deepcopy_speed():
    '''
    test which deepcopy method is the most efficient
    '''
    # Environment Settings
    numObjs = 40
    Density = 0.35
    Height = 1000
    Width = 1000
    
    instanceID = 43

    instance_label = 'two_sized_discs'
    # instance_label = 'identical_discs'
    # instance_label = 'various_ellipses'

    # Load arrangement
    start_arr, goal_arr = get_arrangements(
        numObjs, instance_label, Density, 
        instances_choice=instanceID, descending_by_size=True
        )
    
    try:
        # planner = Bi_Directional_Search_Tree_Planner(start_arr, goal_arr, Height, Width, Density, primitive_plan="TBM")
        # planner = timeout_TBM(start_arr, goal_arr,Density)
        planner = timeout_MCTS_h(start_arr, goal_arr, Density, UCB_scalar=0.1, collision_check='grid',fineness=10)
        # print(planner.FVS)
    except TimeoutError:
        print("Timed Out")
        exit()
    cell_obj_table = planner.root.cell_obj_table
    obj_cell_table = planner.root.obj_cell_table
        
    # print('prev',obj_cell_table[0])
    # # a = deepcopy_grid(cell_obj_table)
    # # a = {k:set(list(v)) for k,v in cell_obj_table.items()}
    # b = {k:list(v) for k,v in obj_cell_table.items()}
    # b[0].append((-1,-1))
    # print('updated',b[0])
    # print('new', obj_cell_table[0])
    # return
    start = time.time()
    for _ in range(5000):
        a = copy.deepcopy(cell_obj_table)
        b = copy.deepcopy(obj_cell_table)
    
    print('deepcopy time', time.time()-start)
    
    start = time.time()
    for _ in range(5000):
        a = {k:set(list(v)) for k,v in cell_obj_table.items()}
        b = {k:list(v) for k,v in obj_cell_table.items()}
        # b = ujson.loads(ujson.dumps(obj_cell_table))
    
    print('operation time', time.time()-start)
    
    
def deepcopy_grid(cell_obj_table):
    new_table = {}
    for k,set_v in cell_obj_table.items():
        new_table[k] = set(ujson.loads(ujson.dumps(list(cell_obj_table[k]))))
    return new_table
    # a = ujson.loads(ujson.dumps(cell_obj_table))


def test_memory_error():
    '''
    test memory error
    '''
    limit_memory(1e4)
    a = []
    try:
        while 1:
            a.append(1)
    except MemoryError as e:
        print(e)
        print('excepted')
        # gc.collect()
        print(len(a))
    del a
    b = []
    try:
        while 1:
            b.append(1)
    except MemoryError as e:
        print(e)
        print('excepted')
        # gc.collect()
        print(len(b))
        # print(len(a))
    # b = []
    # b.append(1)
    print('finish')


if __name__ == '__main__':
    # test_deepcopy_speed()
    test_memory_error()