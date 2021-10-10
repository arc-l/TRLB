from local_planner.Labeled_Optimizer import One_Shot_Planner as Quadratic_planner
from local_planner.Labeled_Optimizer_PP import Labeled_Optimizer_PP as Quadratic_planner_PP
from high_level_planners.Bi_Directional_Search_Tree_Planner import Bi_Directional_Search_Tree_Planner
from high_level_planners.Forward_Search_Planner import Forward_Search_Planner

def TRLB_components_selector(start_arr, goal_arr, Height, Width, radius, high_level='BST', primitive_plan='RB', buffer_generation='SP', robust_sampling = False, PP=True):
    if primitive_plan == 'RBM':
        primitive_plan = 'RB'
    elif primitive_plan == 'TBM':
        primitive_plan = 'FVS'
    elif primitive_plan == 'RO':
        primitive_plan == 'rand'
    else:
        print('Invalid primitive planner')
        exit(0)

    if (buffer_generation != 'SP') and (buffer_generation != 'OPT'):
        print('Invalid buffer generation option')
        exit(0)

    if high_level == 'BST': # Bi Directional Search Tree
        planner = Bi_Directional_Search_Tree_Planner(start_arr, goal_arr, Height, Width, radius, sampling=(buffer_generation=='SP'), robust_sampling=robust_sampling, PP=PP)
    elif high_level == 'ST': # Forward Search Tree
        planner = Forward_Search_Planner(start_arr, goal_arr, Height, Width, radius, sampling=(buffer_generation=='SP'), robust_sampling=robust_sampling, PP=PP)
    elif high_level == 'OS': # One Shot Planner
        if PP == True:
            planner = Quadratic_planner_PP(start_arr, goal_arr, Height, Width, radius, sampling=(buffer_generation=='SP'), robust_sampling=robust_sampling)
        else:
            planner = Quadratic_planner(start_arr, goal_arr, Height, Width, radius, sampling=(buffer_generation=='SP'), robust_sampling=robust_sampling)
    else:
        print('Invalid high-level planner.')
        exit(0)
    return planner