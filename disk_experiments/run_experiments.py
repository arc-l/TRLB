from timeout_decorator.timeout_decorator import timeout
from fmrs.fmrs_planner import FMRS_Planner
from tools.show_arrangement import show_arrangement, show_separate_arrangement
from tools.util import generate_instance, write_a_plan, write_to_log
from tools.plan_Animation import plan_animation
from planner import TRLB_components_selector


from numpy.random import choice
from os.path import join, dirname, abspath
from math import sqrt, pi
from time import time


'''
Methods
High-Level: One-Shot(OS), Forward-Search-Tree(ST), Bi-Search-Tree(BST)

Buffer-Allocation: Sampling(SP), Optimization(OPT)

Primitive-Plan: RunningBuffer(RBM), TotalBuffer(TBM), Random(RO)

PreProcessing=: PP, None

'''


if __name__=="__main__":
    
    # Environment Settings
    numObjs = 10
    Density = 0.4
    Height = 1000
    Width = 1000
    arrangementIDs = [10,1]
    # arrangementIDs = choice(range(20),size=2, replace=False)
    radius = sqrt(Density*Height*Width/pi/numObjs)


    # Load arrangement
    start_arr, goal_arr = generate_instance(numObjs, Density, arrangementIDs)
    
    # Get a plan
    planner = TRLB_components_selector(start_arr, goal_arr, Height, Width, radius, 
    high_level='OS', 
    primitive_plan='TBM',
    buffer_generation='SP', 
    robust_sampling=False,
    PP=True)

    if len(planner.action_list) == 0:
        print("No feasible solution is found.")
        exit(0)
    else:
        for action in planner.action_list:
            print(action)

    # Print instance
    show_arrangement(numObjs, Density, start_arr, goal_arr, HEIGHT= Height, WIDTH= Width)

    # Animation
    plan_animation(Height, Width, start_arr, goal_arr, radius, planner.action_list)

    