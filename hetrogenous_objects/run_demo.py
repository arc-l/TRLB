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

def demo():
    # Environment Settings
    numObjs = 12
    Density = 0.0
    Height = 1000
    Width = 1000
    
    instanceID = 0

    instance_label = 'YCB1'
    

    # Load arrangement
    start_arr, goal_arr = get_arrangements(
        numObjs, instance_label, Density, 
        instances_choice=instanceID, descending_by_size=True,
        flag='coll'
    )
    
    deep_copied_start_arr = copy.deepcopy(start_arr)
    
    # print([ start_arr[i][6] for i in range(len(start_arr))])
    
    show_hetrogenous_arrangement(numObjs, Density, start_arr, goal_arr,show_arrangements='start',show_text=True)
    # exit()
    
    # Get a plan
    try:
        # planner, comp_time = timeout_Vertex_Weighted_RBM(start_arr, goal_arr,Density)
        planner, comp_time = timeout_RBM(start_arr, goal_arr,Density)
        # planner, comp_time = timeout_MCTS(
        #     start_arr, goal_arr, Density, heuristic=0,
        #     collision_check='naive', max_depth=4,fineness=20, 
        #     buffer_strategy='naive',interval_size=4
        #     )
        # planner, comp_time = timeout_Reward_Weighted_MCTS(
        #     start_arr, goal_arr, Density, heuristic=2,
        #     collision_check='naive', max_depth=4,fineness=20, 
        #     buffer_strategy='naive',interval_size=4
        #     )
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

    # Animation
    plan_animation(Height, Width, start_arr, goal_arr, Density, planner.action_list)
    plan_pybullet_animation(Height, Width, start_arr, goal_arr, Density, planner.action_list)


        


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



if __name__=="__main__":
    

    demo()
    