import math
from math import *
from textwrap import indent

import numpy as np
from shapely.geometry import Polygon
from matplotlib.patches import Ellipse
import random
import os
from os.path import join
from ujson import loads, dumps
from json import load, dumps

import matplotlib.pyplot as plt

from tools.general_objects import get_area, get_AABB

def find_area(obj):
    area = obj[4] * obj[5]
    if obj[3] == 'disc':
        area = area * math.pi / 4
    return area

def get_arrangements(numObjs, instance_label='test', Density=0.3, instances_choice=0, descending_by_size=True,flag='coll'):
    '''
    get arrangements from json files
    flag for weights:
    coll: collision possibility
    effort: area
    '''
    folder_name = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'instances',
        instance_label, 
        'Density='+"{:.2f}".format(Density),
        'numObjs='+str(numObjs)
        )
    file_name = str(instances_choice)+"_"+str(numObjs)+"_"+"{:.2f}".format(Density)+'.json'
    
    #Arrage objects in order of size
    start_list = []
    goal_list = []

    ### start arr and goal arr ###
    with open( 
        join(
            folder_name,file_name
            )) as f:

        data_dict = load(f)
        object_list = data_dict['object_list']
        for obj_idx in range(len(object_list)):
            obj = object_list[obj_idx]
            if obj['Object_Shape'] == 'cuboid':
                area = obj['Object_Length'] * obj['Object_Width']
            elif obj['Object_Shape'] == 'disc':
                area = obj['Object_Length'] * obj['Object_Width']*4/math.pi
            else:
                pose = (0,0,0,obj['Object_Shape'], obj['Object_Length'], obj['Object_Width'],0)
                area = int(get_area(pose))
            start_list.append([
                area,
                obj['initial_pose'][0],
                obj['initial_pose'][1],
                obj['initial_pose'][2],
                obj['Object_Shape'],
                obj['Object_Length'],
                obj['Object_Width']
                ])
            goal_list.append([
                area,
                obj['goal_pose'][0],
                obj['goal_pose'][1],
                obj['goal_pose'][2],
                obj['Object_Shape'],
                obj['Object_Length'],
                obj['Object_Width']
                ])

    
    indx_ranking = sorted(list(range(len(start_list))),key=lambda e:start_list[e][0],reverse=descending_by_size)
    start_list = [start_list[i] for i in indx_ranking]
    goal_list = [goal_list[i] for i in indx_ranking]


    #Correcting the format
    start_arr = {}
    goal_arr = {}

    for i in range(len(start_list)):
        start_arr[i] = (start_list[i][1:])
        goal_arr[i] = (goal_list[i][1:])
    
    # Calculating z-score h_1
    # areas = []
    # for idx in start_arr:
    #     areas.append( find_area( start_arr[idx]) )
    
    # mean = sum(areas) / len(areas)
    # std_dev = ( sum( [(x - mean)**2 for x in areas] ) / len(areas) ) ** 0.5

    # for idx in start_arr:
    #     z = ( areas[idx] - mean ) / std_dev
    #     start_arr[idx].append(z)
    #     goal_arr[idx].append(z)
        
    # Uncomment to see z-score for each object
    #     print(areas[idx], "\t", z)
    
    # Add weight to the state
    return weighting_for_objects(start_arr, goal_arr,flag=flag)
    
    # areas = []
    # for idx in start_arr:
    #     areas.append( find_area( start_arr[idx]) )
    
    # min_ = min(areas)
    # max_ = max(areas)
    # range_ = max_ - min_

    # for idx in start_arr:
    #     if range_ == 0:
    #         new_c = 0
    #     else:
    #         new_c = (areas[idx] - min_) / range_
    #     start_arr[idx].append(new_c)
    #     goal_arr[idx].append(new_c)
    # #     # print(areas[idx], "\t", new_c)

    # return start_arr, goal_arr


def weighting_for_objects(start_arr, goal_arr,flag='coll'):
    '''
    weighting for objects
    
    flag:
    coll: collision possibility
    effort: area
    
    return: start and goal arrs with weights scaled to [1,+oo)
    '''
    if flag == 'coll':
        return weighting_with_collision_possibility(start_arr, goal_arr)
    elif flag == 'effort':
        return weighting_with_efforts(start_arr, goal_arr)


    
def weighting_with_collision_possibility(start_arr, goal_arr):
    '''
    weighting with collision possibility
    '''
    # Add weight to the state
    areas = {}
    cs = {} # circumference
    for objID,pose in start_arr.items():
        if pose[3] == 'cuboid':
            areas[objID] = pose[4]*pose[5]
            cs[objID] = 2*(pose[4]+pose[5])
        elif pose[3] == 'disc':
            areas[objID] = pi/4.0*pose[4]*pose[5]
            a,b = pose[4]/2,pose[5]/2
            h = ((a-b)**2)/((a+b)**2)
            cs[objID] = pi*(a+b)*(1+3*h/(10+sqrt(4-3*h)))
        else:
            areas[objID] = get_area(pose)
            AABB = get_AABB(pose)
            # cs[objID] = 2*(abs(AABB[0][0]-AABB[0][1])+abs(AABB[0][0]-AABB[1][0]))
            cs[objID] = 2*(abs(AABB[1][0]-AABB[0][0])+abs(AABB[1][1]-AABB[0][1]))
    avg_area = np.average(list(areas.values()))
    avg_r = sqrt(avg_area/pi)
    abs_p = {}
    for objID, pose in start_arr.items():
        abs_p[objID] = (areas[objID]+avg_area+avg_r*cs[objID])/\
            ((1000-2*sqrt(avg_r))**2)
    print(abs_p)
    # normalize
    min_p = min(list(abs_p.values()))

    for idx in start_arr:
        new_c = abs_p[idx]/min_p
        start_arr[idx].append(new_c)
        goal_arr[idx].append(new_c)
    #     # print(areas[idx], "\t", new_c)
    return start_arr, goal_arr
    

def weighting_with_efforts(start_arr, goal_arr):
    '''
    evaluate the total effort
    '''
    areas = {}
    for objID,pose in start_arr.items():
        if pose[3] == 'cuboid':
            areas[objID] = pose[4]*pose[5]
        elif pose[3] == 'disc':
            areas[objID] = pi/4.0*pose[4]*pose[5]
        else:
            areas[objID] = get_area(pose)
    min_area = min(list(areas.values()))
    for idx in start_arr:
        new_c = areas[idx]/min_area
        start_arr[idx].append(new_c)
        goal_arr[idx].append(new_c)
    #     # print(areas[idx], "\t", new_c)
    return start_arr, goal_arr
    

def poly_stick(center, direction, length, width):
    '''
    Input: 
    center: (x,y)
    direction: radian
    length
    width

    Output:
    poly: point list
    '''
    poly = []
    diameter = math.sqrt(length**2+width**2)

    dtheta = math.asin(float(width)/diameter)

    direction_list = [
        direction-dtheta,
        direction+dtheta,
        math.pi + direction-dtheta,
        math.pi + direction+dtheta
    ]

    for d in direction_list:
        x = center[0] + diameter/2.0*math.cos(d)
        y = center[1] + diameter/2.0*math.sin(d)
        poly.append([x,y])
    return poly

def poly_disc(center, direction, length, width):
    ellipse = Ellipse(center, length, width, direction*180/pi)
    vertices = ellipse.get_verts()

    vertices = vertices[::3]
    
    return vertices


# To understand how poly_disc works ^

# ellipse_points = poly_disc((0,0), pi/2, 10, 2)
# print(ellipse_points)

# X = [pt[0] for pt in ellipse_points]
# Y = [pt[1] for pt in ellipse_points]

# print(X, Y)
# plt.scatter(X, Y)
# plt.show()


### check if two polygons overlap ###
def polysCollide(poly1, poly2):
    cpoly1 = Polygon(poly1)
    cpoly2 = Polygon(poly2)
    return cpoly1.overlaps(cpoly2) or cpoly1.intersects(cpoly2)

def polysWithin(poly1, poly2):
    ''' not used anymore '''
    cpoly1 = Polygon(poly1)
    cpoly2 = Polygon(poly2)
    return cpoly1.overlaps(cpoly2)


# Verification of polysCollide
# s1 = poly_stick((654.944976, 676.4770821), 1.148582618, 239.9715892, 117.9974675)
# g1 = poly_stick((802.7683371, 697.090734), 1.42051809, 239.9715892, 117.9974675)

# s2 = poly_stick((823.1652278, 104.06247829), 3.31595237, 239.9715892, 117.9974675)
# print("s1 - g1 ", polysCollide(s1, g1))
# print("s1 - s2 ", polysCollide(s1, s2))

# s4 = poly_disc((185.2573183, 359.46518145), 1.6690455826, 696.8393902, 199.4625567)
# g4 = poly_disc((155.1952224, 359.472837), 1.84795887, 696.8393902, 199.4625567)

# print("s4 - g4 ", polysCollide(s4, g4))        
                

def deepcopy_dict(d):
    return { k: loads(dumps(d[k])) for k in d.keys()}

def set2tuple(s):
    return tuple(sorted(list(s)))

def deepcopy_graph(G):
    new_G = {}
    for k in G.keys():
        new_G[k] = deepcopy_set(G[k])
    return new_G

def deepcopy_set(s):
    return set(list(s))


def limit_memory(maxsize): 
    import resource
    soft, hard = resource.getrlimit(resource.RLIMIT_AS) 
    resource.setrlimit(resource.RLIMIT_AS, (maxsize, hard))
    
################# plotting tools #################
def plot_figures(fig_list, fig_names=[]):
    n = len(fig_list)
    if len(fig_names)!=n:
        fig_names = [str(x) for x in range(1, n+1)]
    if n==1:
        plt.imshow(fig_list[0])
        plt.title(fig_names[0])
    else:
        fig, axes = plt.subplots(1,n)
        for i in range(n):
            axes[i].imshow(fig_list[i])
            axes[i].set_title(fig_names[i])
    plt.show()
    
    