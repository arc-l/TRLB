import wrapt_timeout_decorator as timeout_decorator
import time

from TRLB.Bi_Directional_Search_Tree_Optimization_Planner import Bi_Directional_Search_Tree_Planner
from Vertex_Weighted_TRLB.Bi_Directional_Search_Tree_Optimization_Planner import Bi_Directional_Search_Tree_Planner as Vertex_Weighted_TRLB
from Python_MCTS.MCTS import Py_MCTS as Python_MCTS
from Modified_Rewards_Python_MCTS.MCTS import Py_MCTS as Weighted_Reward_MCTS

Timeout=600
isUbuntu = False


@timeout_decorator.timeout(Timeout, use_signals=isUbuntu)
def timeout_TBM(start_arr, goal_arr,Density=0.3,Height=1000,Width=1000):
    start = time.time()
    planner = Bi_Directional_Search_Tree_Planner(start_arr, goal_arr, Height, Width, Density, primitive_plan="TBM")
    return planner, time.time()-start


@timeout_decorator.timeout(Timeout, use_signals=isUbuntu)
def timeout_RBM(start_arr, goal_arr,Density=0.3,Height=1000,Width=1000):
    start = time.time()
    planner = Bi_Directional_Search_Tree_Planner(start_arr, goal_arr, Height, Width, Density, primitive_plan="RBM")
    return planner, time.time()-start
    


@timeout_decorator.timeout(Timeout, use_signals=isUbuntu)
def timeout_Vertex_Weighted_RBM(start_arr, goal_arr,Density=0.3,Height=1000,Width=1000):
    start = time.time()
    planner = Vertex_Weighted_TRLB(start_arr, goal_arr, Height, Width, Density, primitive_plan="RBM")
    return planner, time.time()-start


@timeout_decorator.timeout(Timeout, use_signals=isUbuntu)
def timeout_Vertex_Weighted_TBM(start_arr, goal_arr,Density=0.3,Height=1000,Width=1000):
    start = time.time()
    planner = Vertex_Weighted_TRLB(start_arr, goal_arr, Height, Width, Density, primitive_plan="TBM")
    return planner, time.time()-start


@timeout_decorator.timeout(Timeout, use_signals=isUbuntu)
def timeout_MCTS(
    start_arr, goal_arr,Density=0.3,Height=1000,Width=1000, 
    heuristic=2,UCB_scalar=1.0,collision_check='naive',
    fineness=10,max_depth=4,buffer_strategy = 'naive',
    interval_size = 3,
    **kwargs):
    start = time.time()
    planner = Python_MCTS(start_arr, goal_arr, 0, Width, Height, heuristic=heuristic, UCB_scalar=UCB_scalar,
            collision_check=collision_check, fineness=fineness, max_depth=max_depth,
            buffer_strategy=buffer_strategy,interval_size = interval_size, **kwargs)
    return planner, time.time()-start


@timeout_decorator.timeout(Timeout, use_signals=isUbuntu)
def timeout_Reward_Weighted_MCTS(
    start_arr, goal_arr,Density=0.3,Height=1000,Width=1000, 
    heuristic=2,UCB_scalar=1.0,collision_check='naive',
    fineness=10,max_depth=4,buffer_strategy = 'naive',
    interval_size = 3,
    **kwargs):
    start = time.time()
    planner = Weighted_Reward_MCTS(start_arr, goal_arr, 0, Width, Height, heuristic=heuristic, UCB_scalar=UCB_scalar,
            collision_check=collision_check, fineness=fineness, max_depth=max_depth,
            buffer_strategy=buffer_strategy,interval_size = interval_size, **kwargs)
    return planner, time.time()-start