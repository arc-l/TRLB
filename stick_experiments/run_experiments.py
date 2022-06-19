import random
import numpy as np
import os
import math
import sys

sys.path.append(os.path.dirname(__file__))

from fmrs.fmrs_planner import FMRS_Planner
from tools.show_arrangement import show_stick_arrangement, show_separate_stick_arrangement
from tools.util import generate_instance
from tools.plan_Animation import plan_animation
from TRLB.Bi_Directional_Search_Tree_Optimization_Planner import Bi_Directional_Search_Tree_Planner


def example():
    '''
    An example code for the stick scenario
    '''
    # Environment Settings
    numObjs = 10
    Density = 0.3
    Height = 1000
    Width = 1000
    # instance_num = 200
    # arrangementIDs = np.random.choice(range(instance_num),size=2, replace=False)
    
    arrangementIDs = [16,  2]

    # Load arrangement
    start_arr, goal_arr, WL_ratio = generate_instance(numObjs, Density, arrangementIDs)

    # Get a plan
    planner = Bi_Directional_Search_Tree_Planner(start_arr, goal_arr, Height, Width, Density, WL_ratio = WL_ratio)
    # planner = FMRS_Planner(start_arr, goal_arr, Height, Width, Density, WL_ratio = WL_ratio)

    print( 'number of actions: ', len(planner.action_list))

    # Print instance
    show_stick_arrangement(numObjs, Density, start_arr, goal_arr, WL_ratio=WL_ratio)
    # show_separate_stick_arrangement(numObjs, Density, goal_arr, WL_ratio=WL_ratio)
    # show_separate_stick_arrangement(numObjs, Density, start_arr, WL_ratio=WL_ratio)

    # Animation
    plan_animation(Height, Width, start_arr, goal_arr, Density, planner.action_list, WL_ratio=WL_ratio)

if __name__=="__main__":
    example()