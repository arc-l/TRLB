import time
import pandas as pd
import os
import gc
import traceback
import copy

from tools.util import get_arrangements, limit_memory
from methods_with_timeout import timeout_MCTS, timeout_TBM, timeout_RBM,timeout_Vertex_Weighted_RBM, timeout_Vertex_Weighted_TBM, timeout_Reward_Weighted_MCTS



def TRLB_recorder(
    method_name, experimentID,
    numObjs, Density=0.3, arrangementID=0,
    instance_label='test', primitive_planner='TBM', 
    weight_flag='coll'
    ):
    ''' record TRLB results '''
    trial_res = {}
    trial_res['n'] = numObjs
    trial_res['label'] = instance_label
    trial_res['aID'] = arrangementID
    trial_res['density'] = Density
    trial_res['type'] = 'TRLB'
    trial_res['primitive'] = primitive_planner
    trial_res['name'] = method_name
    

    start_arr, goal_arr = get_arrangements(
    numObjs, instance_label, Density, 
    instances_choice=arrangementID, descending_by_size=True,
    flag=weight_flag
    )
    
    deep_copied_start_arr = copy.deepcopy(start_arr)
    
    if primitive_planner == 'TBM':
        solver = timeout_TBM
    elif primitive_planner == 'RBM':
        solver = timeout_RBM
    elif primitive_planner == 'V_RBM':
        solver = timeout_Vertex_Weighted_RBM
    elif primitive_planner == 'V_TBM':
        solver = timeout_Vertex_Weighted_TBM
    
    Success = True
    try:
        # collision_check_strategy = method[7:]
        planner, comp_time = solver(
            start_arr, goal_arr,Density=Density,
            )
        print(
            "\t ", method_name, "..."*5, str(round(comp_time, 2)), 
            round(len(planner.action_list)/numObjs,2), 
            round(sum([deep_copied_start_arr[a[0]][6] for a in planner.action_list]),2)
            )
        
        if len(planner.action_list) != 0:
            Success = True
        else:
            Success = False
    
    except TimeoutError as e:
        Success = False
        print(e)
        print("\t ", method_name, "..."*5, "TIMED OUT")
    except MemoryError as e:
        Success = False
        print(e)
        print("\t ", method_name, "..."*5, "Run out of Memory")
    except Exception as e:
        Success = False
        print(traceback.format_exc())
        print(e)
    
    if Success:
        trial_res['soln_len'] = len(planner.action_list)/numObjs
        trial_res['cost'] = sum([deep_copied_start_arr[a[0]][6] for a in planner.action_list])
        trial_res['time'] = comp_time
        if primitive_planner == 'TBM':
            trial_res['FVS'] = planner.FVS
    else:
        trial_res['soln_len'] = -1
        trial_res['cost'] = -1
        trial_res['time'] = -1
        if primitive_planner == 'TBM':
            trial_res['FVS'] = -1
    
    
    
    save_file = os.path.join(
        os.path.dirname(__file__),
        'results', instance_label, method_name,
        experimentID+'.csv'
        )
    
    folder = os.path.dirname(save_file)
    if not os.path.exists(folder):
        os.makedirs(folder)
    
    try:
        del planner
    except Exception:
        pass
    gc.collect()
    
    try:
        # Saving the results
        pd.DataFrame.from_dict([trial_res]).to_csv(save_file, index=False, header=(arrangementID==0),mode='a')    
    except MemoryError:
        gc.collect()

def MCTS_recorder(
    method_name,  experimentID,
    numObjs, Density=0.3, arrangementID=0,
    instance_label='test',
    UCB_scalar = 0.1,use_heuristics=0,
    collision_checker='bv', fineness=10, weight_flag='coll', **kwargs
    ):
    ''' record MCTS results '''
    trial_res = {}
    trial_res['n'] = numObjs
    trial_res['label'] = instance_label
    trial_res['aID'] = arrangementID
    trial_res['density'] = Density
    trial_res['type'] = 'MCTS'
    trial_res['use_heuristics'] = use_heuristics
    trial_res['name'] = method_name
    trial_res['collision_checker'] = collision_checker
    trial_res['UCB_scalar'] = UCB_scalar
    trial_res['fineness'] = fineness
    

    start_arr, goal_arr = get_arrangements(
    numObjs, instance_label, Density, 
    instances_choice=arrangementID, descending_by_size=True,
    flag=weight_flag
    )
    deep_copied_start_arr = copy.deepcopy(start_arr)
    
    solver = timeout_MCTS
    
    Success = True
    try:
        # collision_check_strategy = method[7:]
        planner, comp_time = solver(
            start_arr, goal_arr,Density=Density,
            collision_check=collision_checker,
            UCB_scalar=UCB_scalar,fineness=fineness,
            heuristic=use_heuristics,
            **kwargs
            )
        print(
            "\t ", method_name, "..."*5, str(round(comp_time, 2)), 
            round(len(planner.action_list)/numObjs,2), 
            round(sum([deep_copied_start_arr[a[0]][6] for a in planner.action_list]),2)
            )
        
        if len(planner.action_list) != 0:
            Success = True
        else:
            Success = False
    
    except TimeoutError as e:
        Success = False
        print(e)
        print("\t ", method_name, "..."*5, "TIMED OUT")
    except MemoryError as e:
        Success = False
        print(e)
        print("\t ", method_name, "..."*5, "Run out of Memory")
    except Exception as e:
        Success = False
        print(traceback.format_exc())
        print(e)
    
    if Success:
        trial_res['soln_len'] = len(planner.action_list)/numObjs
        trial_res['cost'] = sum([deep_copied_start_arr[a[0]][6] for a in planner.action_list])
        trial_res['time'] = comp_time
        trial_res['time_coll_setup'] = planner.time_coll_setup
        trial_res['time_coll_check'] = planner.time_coll_check
        trial_res['num_collision'] = planner.num_collision
        trial_res['num_iteration'] = planner.num_iter
    else:
        trial_res['soln_len'] = -1
        trial_res['cost'] = -1
        trial_res['time'] = -1
        trial_res['time_coll_setup'] = -1
        trial_res['time_coll_check'] = -1
        trial_res['num_collision'] = -1
        trial_res['num_iteration'] = -1
    
    save_file = os.path.join(
        os.path.dirname(__file__),
        'results', instance_label, method_name,
        experimentID+'.csv'
        )
    
    folder = os.path.dirname(save_file)
    if not os.path.exists(folder):
        os.makedirs(folder)
    
    try:
        del planner
    except Exception:
        pass
    gc.collect()
    
    try:
        # Saving the results
        pd.DataFrame.from_dict([trial_res]).to_csv(save_file, index=False, header=(arrangementID==0),mode='a')    
    except MemoryError:
        gc.collect()
    
    
def Reward_Weighted_MCTS_recorder(
    method_name,  experimentID,
    numObjs, Density=0.3, arrangementID=0,
    instance_label='test',
    UCB_scalar = 0.1,use_heuristics=0,
    collision_checker='bv', fineness=10, weight_flag='coll', **kwargs
    ):
    ''' record MCTS results '''
    trial_res = {}
    trial_res['n'] = numObjs
    trial_res['label'] = instance_label
    trial_res['aID'] = arrangementID
    trial_res['density'] = Density
    trial_res['type'] = 'MCTS'
    trial_res['use_heuristics'] = use_heuristics
    trial_res['name'] = method_name
    trial_res['collision_checker'] = collision_checker
    trial_res['UCB_scalar'] = UCB_scalar
    trial_res['fineness'] = fineness
    

    start_arr, goal_arr = get_arrangements(
    numObjs, instance_label, Density, 
    instances_choice=arrangementID, descending_by_size=True,
    flag=weight_flag
    )
    deep_copied_start_arr = copy.deepcopy(start_arr)
    
    solver = timeout_Reward_Weighted_MCTS
    
    Success = True
    try:
        # collision_check_strategy = method[7:]
        planner, comp_time = solver(
            start_arr, goal_arr,Density=Density,
            collision_check=collision_checker,
            UCB_scalar=UCB_scalar,fineness=fineness,
            heuristic=use_heuristics,
            **kwargs
            )
        print(
            "\t ", method_name, "..."*5, str(round(comp_time, 2)), 
            round(len(planner.action_list)/numObjs,2), 
            round(sum([deep_copied_start_arr[a[0]][6] for a in planner.action_list]),2)
            )
        
        if len(planner.action_list) != 0:
            Success = True
        else:
            Success = False
    
    except TimeoutError as e:
        Success = False
        print(e)
        print("\t ", method_name, "..."*5, "TIMED OUT")
    except MemoryError as e:
        Success = False
        print(e)
        print("\t ", method_name, "..."*5, "Run out of Memory")
    except Exception as e:
        Success = False
        print(traceback.format_exc())
        print(e)
    
    if Success:
        trial_res['soln_len'] = len(planner.action_list)/numObjs
        trial_res['cost'] = sum([deep_copied_start_arr[a[0]][6] for a in planner.action_list])
        trial_res['time'] = comp_time
        trial_res['time_coll_setup'] = planner.time_coll_setup
        trial_res['time_coll_check'] = planner.time_coll_check
        trial_res['num_collision'] = planner.num_collision
        trial_res['num_iteration'] = planner.num_iter
    else:
        trial_res['soln_len'] = -1
        trial_res['cost'] = -1
        trial_res['time'] = -1
        trial_res['time_coll_setup'] = -1
        trial_res['time_coll_check'] = -1
        trial_res['num_collision'] = -1
        trial_res['num_iteration'] = -1
    
    save_file = os.path.join(
        os.path.dirname(__file__),
        'results', instance_label, method_name,
        experimentID+'.csv'
        )
    
    folder = os.path.dirname(save_file)
    if not os.path.exists(folder):
        os.makedirs(folder)
    
    try:
        del planner
    except Exception:
        pass
    gc.collect()
    
    try:
        # Saving the results
        pd.DataFrame.from_dict([trial_res]).to_csv(save_file, index=False, header=(arrangementID==0),mode='a')    
    except MemoryError:
        gc.collect()
        
    
    