from cmath import pi
from tools.util import deepcopy_dict, poly_stick, poly_disc, polysCollide
from tools.general_objects import get_poly
from numpy import array
from Vertex_Weighted_TRLB.Labeled_DFS_DP import DFS_DP_Search
from Vertex_Weighted_TRLB.FVS_DP_Solver import FVS_Plan_Search

class primitive_plan_external_buffer():
    """
    Used to make a plan for the external buffer setting, i.e. just the primitive plan
    Input:
    start_arr   Initial arrangements
    goal_arr    Final arrangements
    type        TBM/RBM

    Output:
    self.plan   sequence of actions required to go from initial to final arrangement

    """
    def __init__(self, start_arr, goal_arr, Height, Width, Density, type="RBM"):
        self.start_arr = deepcopy_dict(start_arr)
        self.goal_arr = deepcopy_dict(goal_arr)
        self.Height = Height
        self.Width = Width
        self.Density = Density
        self.type = type

        # Solution
        self.DG = {}
        self.object_size_map = {}
        self.plan = []
        self.b_s_ratio = 0

        # Making the dependancy graph
        self.DG = self.construct_DG()

        # Generating the plan
        if self.type == "RBM":
            self.plan = DFS_DP_Search(self.DG, 0)[1]
        else:
            self.plan = FVS_Plan_Search(self.DG)

        # Analyzing the plan
        self.object_size_map = self.map_objects_by_size()
        self.calc_b_s()
        # self.calc_buffer_b_s()

    def construct_DG(self):
        DG = {}
        for goal_obj, goal_pose in self.goal_arr.items():
            DG[goal_obj] = set()
            if goal_pose[3] == 'cuboid':
                goal_poly = array(
                    poly_stick([goal_pose[0],goal_pose[1]], goal_pose[2], 
                    goal_pose[4], goal_pose[5])
                    )
            elif goal_pose[3] == 'disc':
                goal_poly = array(
                    poly_disc([goal_pose[0],goal_pose[1]], goal_pose[2], 
                    goal_pose[4], goal_pose[5])
                    )
            else:
                goal_poly = array(
                    get_poly(goal_pose)
                )
            for start_obj, start_pose in self.start_arr.items():
                if start_pose[3] == 'cuboid':
                    start_poly = array(
                        poly_stick([start_pose[0],start_pose[1]], start_pose[2], 
                        start_pose[4], start_pose[5])
                    )
                elif start_pose[3] == 'disc':
                    start_poly = array(
                        poly_disc([start_pose[0],start_pose[1]], start_pose[2], 
                        start_pose[4], start_pose[5])
                    )
                else:
                    start_poly = array(
                        get_poly(start_pose)
                    )
                if polysCollide(start_poly, goal_poly):
                    DG[goal_obj].add(start_obj)
        return DG
    
    def map_objects_by_size(self):
        big_small_map = { 'big':[], 'small':[] }
        areas = {}
        for obj_idx, obj in self.start_arr.items():
            if obj[3] == 'cuboid':
                area = obj[4]*obj[5]
            else:
                area = obj[4]*obj[5]*pi/4
            
            if area not in areas:
                areas[area] = True
        
        if len(areas) != 2:
            raise Exception("More than two distinct areas exists")
        
        area_small, area_big = sorted(areas.keys())

        for obj_idx, obj in self.start_arr.items():
            if obj[3] == 'cuboid':
                area = obj[4]*obj[5]
            else:
                area = obj[4]*obj[5]*pi/4
            
            if area == area_big:
                big_small_map['big'].append(obj_idx)
            else:
                big_small_map['small'].append(obj_idx)

        return big_small_map
    
    def calc_b_s(self):
        n_b, n_s = 0, 0
        for move in self.plan:
            if move[0] in self.object_size_map['big']:
                n_b +=1
            elif move[0] in self.object_size_map['small']:
                n_s +=1
            else:
                raise Exception("Object not in object size map")
        
        self.n_b = n_b
        self.n_s = n_s
    
    # def calc_buffer_b_s(self):
    #     n_b, n_s = 0, 0
    #     for move in self.plan:
    #         if move[0] in self.object_size_map['big'] and move[1] == 'b':
    #             n_b +=1
    #         elif move[0] in self.object_size_map['small'] and move[1] == 'b':
    #             n_s +=1
        
    #     if n_b == 0 and n_s == 0:
    #         self.buffer_b_s_ratio = None
    #     elif n_s == 0:
    #         self.buffer_b_s_ratio = n_b
    #     else:
    #         self.buffer_b_s_ratio = n_b/n_s
