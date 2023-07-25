import os
import sys
import time
import matplotlib.pyplot as plt
import datetime
import pandas as pd
import numpy as np
import copy


from tools.generate_instances import create_instances, create_dense_instances, create_Tetris_instance,create_YCB_instance,\
    generate_identical_discs, generate_two_sized_discs, generate_various_ellipses,\
    generate_various_sticks, generate_thin_sticks,generate_Klotski_blocks, generate_random_ellipses_n_sticks,\
    generate_organized__Klotski_blocks_one_large_side_3, generate_organized__Klotski_blocks_two_large_side_3
from tools.util import get_arrangements, polysCollide, poly_disc, limit_memory
from tools.show_arrangement import show_hetrogenous_arrangement, show_grid
from TRLB.Bi_Directional_Search_Tree_Optimization_Planner import Bi_Directional_Search_Tree_Planner
from TRLB.primitive_plan import primitive_plan_external_buffer
from tools.plan_Animation import plan_animation, plan_pybullet_animation
from Python_MCTS.MCTS import Py_MCTS as Python_MCTS
from Python_MCTS.Node import test_collision_checker
from methods_with_timeout import timeout_TBM, timeout_RBM, timeout_MCTS, timeout_Vertex_Weighted_RBM, timeout_Reward_Weighted_MCTS
from experiment_recorders import TRLB_recorder, MCTS_recorder, Reward_Weighted_MCTS_recorder
from collision_probability.collision_probability_check import run_experiments as collision_experiment
from Vertex_Weighted_TRLB.Labeled_DFS_DP import DFS_DP_Search


sys.path.append(os.path.dirname(__file__))

def example():
    # Environment Settings
    numObjs = 20
    Density = 0.4
    Height = 1000
    Width = 1000
    
    instanceID = 0

    # different settings
    # instance_label = 'two_sized_discs'
    # instance_label = 'identical_discs'
    # instance_label = 'various_ellipses'
    # instance_label = 'various_sticks'
    # instance_label = 'thin_sticks'
    # instance_label = 'Klotski_blocks'
    # instance_label = 'random_general'
    # instance_label = 'Tetris0'
    # instance_label = 'various_ellipses_v2'
    instance_label = 'Klotski_blocks_2large'
    # instance_label = 'Organized_Klotski_blocks_one_large_side_3'
    # instance_label = 'Organized_Klotski_blocks_two_large_side_3'
    # instance_label = 'YCB0'
    

    # Load arrangement
    start_arr, goal_arr = get_arrangements(
        numObjs, instance_label, Density, 
        instances_choice=instanceID, descending_by_size=True,
        flag='coll'
    )
    
    deep_copied_start_arr = copy.deepcopy(start_arr)
    
    print([ start_arr[i][6] for i in range(len(start_arr))])
    
    show_hetrogenous_arrangement(numObjs, Density, start_arr, goal_arr,show_arrangements='start',show_text=False)
    exit()
    
    # Get a plan
    try:
        # planner, comp_time = timeout_Vertex_Weighted_RBM(start_arr, goal_arr,Density)
        # planner, comp_time = timeout_RBM(start_arr, goal_arr,Density)
        # planner, comp_time = timeout_MCTS(
        #     start_arr, goal_arr, Density, heuristic=0,
        #     collision_check='naive', max_depth=4,fineness=20, 
        #     buffer_strategy='naive',interval_size=4
        #     )
        planner, comp_time = timeout_Reward_Weighted_MCTS(
            start_arr, goal_arr, Density, heuristic=2,
            collision_check='naive', max_depth=4,fineness=20, 
            buffer_strategy='naive',interval_size=4
            )
    except TimeoutError:
        print("Timed Out")
        exit()
    
    try:
        print("computation time", comp_time)
        print( 'number of actions: ', len(planner.action_list))
        print('cost', sum([deep_copied_start_arr[a[0]][6] for a in planner.action_list]))
        print( 'plan - ', [t[0] for t in planner.action_list])
        print('maintn time', planner.time_coll_setup)
        print('colln check time', planner.time_coll_check)
        print('num colln', planner.num_collision)
        print('num iter', planner.num_iter)
    except Exception:
        print('TBM/RBM does not have some of these properties')

    # # # Print instance
    # show_hetrogenous_arrangement(numObjs, Density, start_arr, goal_arr)
    # # # show_separate_stick_arrangement(numObjs, Density, goal_arr, WL_ratio=WL_ratio)
    # # # show_separate_stick_arrangement(numObjs, Density, start_arr, WL_ratio=WL_ratio)

    # # Animation
    # plan_animation(Height, Width, start_arr, goal_arr, Density, planner.action_list)
    # plan_pybullet_animation(Height, Width, start_arr, goal_arr, Density, planner.action_list)


def run_tests(numObjs, experimentID, Density=0.3,num_instance=50,instance_label='test',weight_flag='coll'):
    """
    res = [
        {'n':, 'l':, 'aID':, 'density':, 'success':, 'b_s_ratio':, 'type':}
        ]
    """
    #If set to True then plan animation will be shown
    SHOW_OP = False
    
    # Environment Settings      
    Height = 1000
    Width = 1000

    # set memory limit
    limit_memory(1.3 * 2**(34)) # 16 G

    for arrangementID in range(num_instance):

        print(arrangementID)
        
        TRLB_recorder(
            'TBM_effort', experimentID, numObjs, Density, arrangementID, 
            instance_label, 'TBM', weight_flag=weight_flag
        )
        
        TRLB_recorder(
            'RBM_effort', experimentID, numObjs, Density, arrangementID, 
            instance_label, 'RBM', weight_flag=weight_flag
        )
        
        # TRLB_recorder(
        #     'V_RBM_increase_effort', experimentID, numObjs, Density, arrangementID, 
        #     instance_label, 'V_RBM', weight_flag=weight_flag
        # )
        
        MCTS_recorder(
            'MCTS_0.3_bv_effort', experimentID, numObjs, Density, arrangementID, 
            instance_label, UCB_scalar=0.3, use_heuristics=0,
            collision_checker='bv', max_depth=4,fineness=20, 
            buffer_strategy='naive',interval_size=1, weight_flag=weight_flag
        )
        
        # MCTS_recorder(
        #     'MCTS_h_1_bv', experimentID, numObjs, Density, arrangementID, 
        #     instance_label, UCB_scalar=1, use_heuristics=2,
        #     collision_checker='bv', max_depth=4,fineness=20, 
        #     buffer_strategy='naive',interval_size=1, weight_flag=weight_flag
        # )
        
        # TRLB_recorder(
        #     'V_TBM_effort', experimentID, numObjs, Density, arrangementID, 
        #     instance_label, 'V_TBM', weight_flag=weight_flag
        # )
        
        # Reward_Weighted_MCTS_recorder(
        #     'Rewards_MCTS_0.3_bv_effort', experimentID, numObjs, Density, arrangementID, 
        #     instance_label, UCB_scalar=0.3, use_heuristics=0,
        #     collision_checker='bv', max_depth=4,fineness=20, 
        #     buffer_strategy='naive',interval_size=1, weight_flag=weight_flag
        # )
        
        # Reward_Weighted_MCTS_recorder(
        #     'Rewards_MCTS_h1_0.1_bv_coll', experimentID, numObjs, Density, arrangementID, 
        #     instance_label, UCB_scalar=0.1, use_heuristics=1,
        #     collision_checker='bv', max_depth=4,fineness=20, 
        #     buffer_strategy='naive',interval_size=1, weight_flag=weight_flag
        # )
        
        # Reward_Weighted_MCTS_recorder(
        #     'Rewards_MCTS_h_0.3_bv_effort', experimentID, numObjs, Density, arrangementID, 
        #     instance_label, UCB_scalar=1, use_heuristics=2,
        #     collision_checker='bv', max_depth=4,fineness=20, 
        #     buffer_strategy='naive',interval_size=1, weight_flag=weight_flag
        # )
        
        
        

def batch_create_instances():
    '''
    create instances
    '''
    instance_dict = {
        # 'identical_discs': generate_identical_discs,
        # 'two_sized_discs': generate_two_sized_discs,
        # 'various_ellipses': generate_various_ellipses,
        # 'various_ellipses_v2': generate_various_ellipses,
        # 'various_sticks': generate_various_sticks,
        # 'thin_sticks': generate_thin_sticks,
        'Klotski_blocks_2large': generate_Klotski_blocks,
        # 'Organized_Klotski_blocks_one_large_side_3': generate_organized__Klotski_blocks_one_large_side_3,
        # 'Organized_Klotski_blocks_two_large_side_3': generate_organized__Klotski_blocks_two_large_side_3
        # 'Klotski_blocks': generate_Klotski_blocks,
        # 'random_general': generate_random_ellipses_n_sticks
    }
    # D=0.25, n=20, identical discs
    for Density in [0.35]:
        for n_obj in [10,30,50]:
            for label, object_generator in instance_dict.items():
                print(Density, n_obj, label)
                create_instances(
                    n_obj=n_obj, Density=Density, num_instance=50, 
                    label=label,
                    object_generator=object_generator
                )
                # create_dense_instances(
                #     n_obj=n_obj, Density=Density, num_instance=50, 
                #     label=label,
                #     object_generator=object_generator
                # )
                # create_Tetris_instance(0, n_obj=n_obj, Density=Density, 
                #                        num_instance=50,label=label
                #     )


def batch_create_instances_from_object_list():
    '''
    create instances
    '''
    instance_labels = {
        # 'Tetris':create_Tetris_instance,
        'YCB':create_YCB_instance,
        
    }
    # D=0.25, n=20, identical discs
    for label,method in instance_labels.items():
        print(label)
        method(1,num_instance=50,label=label)


def batch_run_tests():
    # generate an unique experiment ID
    timestamp = time.time()
    timestamp_value = datetime.datetime.fromtimestamp(timestamp)
    experimentID = timestamp_value.strftime("%Y-%m-%d-%H-%M-%S")
    
    for Density in [0.4]:
        for numObjs in [20,30,40]:
            for instance_label in [
                # 'two_sized_discs',
                # 'various_ellipses',
                # 'various_sticks',
                # 'thin_sticks',
                # 'Klotski_blocks',
                # 'various_ellipses_v2',
                # 'Klotski_blocks_2large',
                'random_general',
                # 'Organized_Klotski_blocks_two_large_side_3',
                # 'Organized_Klotski_blocks_one_large_side_3',
                ]:
                print(numObjs, Density, instance_label)
                run_tests(
                    numObjs, experimentID, Density,
                    num_instance=50, instance_label=instance_label,
                    weight_flag='effort'
                )


def buffer_allocation_experiment():
    # Environment Settings
    numObjs = 40
    Density = 0.45
    Height = 1000
    Width = 1000
    
    

    # instance_label = 'two_sized_discs'
    # instance_label = 'identical_discs'
    instance_label = 'various_ellipses'
    # instance_label = 'various_sticks'
    # instance_label = 'thin_sticks'

    quad_list = []
    naive_list = []

    for ii in range(50):
        print('instance', ii)
        instanceID = ii
        # Load arrangement
        start_arr, goal_arr = get_arrangements(
            numObjs, instance_label, Density, 
            instances_choice=instanceID, descending_by_size=True
        )

        planner, comp_time = timeout_MCTS(start_arr, goal_arr, Density, heuristic=0, UCB_scalar=1, collision_check='quad',fineness=10,max_depth=7)
        print(planner.avg_quad_buffer)
        print(planner.avg_naive_buffer)
        quad_list.append(planner.avg_quad_buffer)
        naive_list.append(planner.avg_naive_buffer)
    print('average')
    print('quad',np.average(quad_list))
    print('naive',np.average(naive_list))


def collision_probability_experiments():
    '''
    given two objects, check collision between sticks, 
    where the first object is with specified size and shape, and the other is 100*100 square
    '''
    collision_experiment()
            
        
    


if __name__=="__main__":
    
    # buffer_allocation_experiment()
    
    batch_run_tests()
    
    # batch_create_instances()
    # batch_create_instances_from_object_list()
    
    # example()
    
    # # Environment Settings
    # numObjs = 10
    # Density = 0.2
    # Height = 1000
    # Width = 1000
    
    # instanceID = 2

    # # different settings
    # instance_label = 'random_general'

    # # Load arrangement
    # start_arr, goal_arr = get_arrangements(
    #     numObjs, instance_label, Density, 
    #     instances_choice=instanceID, descending_by_size=True
    # )
    
    # show_hetrogenous_arrangement(
    #     numObjs, Density, start_arr, goal_arr,
    #     show_arrangements='start'
    #     )
    
    # collision_probability_experiments()
    
    
    # DG = {
    #     0:set([1]),
    #     1:set([0]),
    #     2:set([3]),
    #     3:set([2])
    # }
    
    # Weights = {
    #     0:3,
    #     1:4,
    #     2:1000000,
    #     3:2000000
    # }
    
    # print(DFS_DP_Search(DG, Weights, 0,'decrease'))