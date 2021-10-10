from math import asin, pi, sqrt, log, cos, sin, radians
from collections import deque
import numpy
from ujson import loads, dumps
from json import load
from numpy import power, subtract, add, array, Inf, savetxt, random, linspace
from shapely.geometry import Polygon
from random import choice, shuffle, uniform
from os import makedirs
from os.path import join, exists, abspath, dirname
import sys
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.colors as colors
import matplotlib.cm as cmx

sys.path.append(abspath(dirname(dirname(__file__))))


def norm(u):
    return sqrt(sum(power(u, 2)))


def dist(a, b):
    return norm(subtract(a, b))


### find the bounding box for a polygon ###
def bbox(poly):
    xmin = Inf
    xmax = -Inf
    ymin = Inf
    ymax = -Inf
    for p in poly:
        xmin = min(xmin, p[0])
        xmax = max(xmax, p[0])
        ymin = min(ymin, p[1])
        ymax = max(ymax, p[1])
    return xmin, xmax, ymin, ymax


### build the shape of the object based on resolution ###
def poly_disk(center, radius, resolution):
    poly = []
    for i in range(360, 0, -resolution):
        rad = radians(i)
        x = center[0] + radius * cos(rad)
        y = center[1] + radius * sin(rad)
        poly.append([x, y])
    return poly


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
    diameter = sqrt(length**2+width**2)

    dtheta = asin(diameter/width)

    direction_list = [
        direction-dtheta,
        direction+dtheta,
        -direction-dtheta,
        -direction+dtheta
    ]

    for d in direction_list:
        x = center[0] + diameter/2.0*cos(d)
        y = center[1] + diameter/2.0*sin(d)
        poly.append([x,y])
    return poly


### check if two polygons overlap ###
def polysCollide(poly1, poly2):
    cpoly1 = Polygon(poly1)
    cpoly2 = Polygon(poly2)
    return cpoly1.intersects(cpoly2)


### collision checker2 ###
def collisionCheck(objects):
    for i in range(len(objects) - 1):
        for poly in objects[i + 1:]:
            if polysCollide(poly, objects[i]):
                return False
    return True


### collision checker ###
def isCollisionFree(object_shape, center, obstacles):
    object_location = add(center, object_shape)
    
    ### Now check if the new object collides with all other objects (as obstacles)
    for poly in obstacles:
        if polysCollide(poly, object_location):
            return False

    return True


### collision checker ###
def isCollisionFree_with_counter(object_shape, center, obstacles):
    num_check = 0
    object_location = add(center, object_shape)
    
    ### Now check if the new object collides with all other objects (as obstacles)
    for poly in obstacles:
        num_check += 1
        if polysCollide(poly, object_location):
            return False, num_check

    return True, num_check


### collision checker with buffers ###
def countNumOverlap(object_shape, center, obstacles, buffer_obs, numOverlapAllowed):
    buffer_polygon = add(center, object_shape)
    ### Now check if the new buffer collides with all other objects (as obstacles)
    numOverlap = 0
    for poly in obstacles:
        if polysCollide(poly, buffer_polygon):
            numOverlap += 1
        if (numOverlap > numOverlapAllowed):
            return numOverlap
    for poly in buffer_obs:
        if polysCollide(poly, buffer_polygon):
            numOverlap += 3
        if (numOverlap > numOverlapAllowed):
            return numOverlap
    ### reach here since numOverlap <= numOverlapAllowed
    return numOverlap


### BFS search on the connectivity graph ###
def bfs(tree, start, goal):
    path = []
    backtrace = {start: start}
    explore = deque([start])
    while goal not in backtrace:
        try:
            p = explore.pop()
        except Exception:
            return []
        for c in tree[p]:
            if c not in backtrace:
                backtrace[c] = p
                explore.append(c)

    path.append(goal)
    while backtrace[path[-1]] != path[-1]:
        path.append(backtrace[path[-1]])
    path.reverse()

    return path


def BFS(graphAdj, start, goal, condition=lambda x: True):
    path = []
    backtrace = {start: start}
    explore = deque([start])
    while goal not in backtrace:
        if len(explore) > 0:
            u = explore.pop()
        else:
            return []
        for v in filter(condition, graphAdj[u]):
            if v not in backtrace:
                backtrace[v] = u
                explore.appendleft(v)

    path.append(goal)
    while backtrace[path[-1]] != path[-1]:
        path.append(backtrace[path[-1]])
    path.reverse()

    return path


def checkBitStatusAtPos(n, k):
    #This code is contributed by Gitanjali (GeeksforGeeks)
    new_num = n >> k
    return (new_num & 1)


def generate_instance(numObjs, Density, IDs):
    # instance_num = 200
    # instances_choice = choice(range(instance_num),size=2, replace=False)
    instances_choice = IDs
    print( "arrangement choices: ", instances_choice)
    my_path = abspath(dirname(dirname(__file__))) + "/arrangements"
    init_list = range(numObjs)
    final_list = range(numObjs)
    # shuffle(init_list)
    # shuffle(final_list)

    start_arr = {}
    goal_arr = {}

    ### start arr ###
    with open( 
        join(
            my_path, 
            "D="+str(round(Density,1)), 
            "n="+str(numObjs),
            str(instances_choice[0])+"_"+str(numObjs)+"_"+str(round(Density,1))+".json"
            )) as f:
        data_dict = load(f)
        point_list = data_dict['point_list']
        start_arr = {init_list[i]:tuple(p) for i, p in enumerate(point_list)}

    ### goal arr ###
    with open( 
        join(
            my_path, 
            "D="+str(round(Density,1)), 
            "n="+str(numObjs),
            str(instances_choice[1])+"_"+str(numObjs)+"_"+str(round(Density,1))+".json"
            )) as f:
        data_dict = load(f)
        point_list = data_dict['point_list']
        goal_arr = {final_list[i]:tuple(p) for i, p in enumerate(point_list)}
    
    return start_arr, goal_arr


def write_a_plan(experimentID, numObjs, Density, trial, plan, labeltxt):
    folder = join(
        abspath(dirname(dirname(__file__))),
        "plans",
        experimentID,
        labeltxt
    )
    if not exists(folder):
        makedirs(folder)
    
    plan_log = join(folder, str(Density) + '_' + str(numObjs) + '_' + str(trial) + '.txt')

    plan = [ [a[0]] + list(a[1]) + list(a[2]) for a in plan]

    savetxt(plan_log, array(plan))


def write_to_log(experimentID, numObjs, Density, trial, Time, operation_radio, arrangementIDs, labeltxt, num_collision, max_node=0, total_node=0):
    folder = join(
        abspath(dirname(dirname(__file__))),
        "logs",
        labeltxt
    )
    if not exists(folder):
        makedirs(folder)
    
    log = join(folder, experimentID + '.txt')

    with open(log, 'a') as f:
        f.write(str(Density) + " " + str(numObjs) + " " + str(trial) + " " 
        + str(Time) + " " + str(operation_radio) + " " + str(arrangementIDs) + " "
        + str(num_collision) + " " + str(max_node) + " " + str(max_node) + "\n")


def set2tuple(s):
    return tuple(sorted(list(s)))


def deepcopy_dict(d):
    return { k: loads(dumps(d[k])) for k in d.keys()}


def deepcopy_graph(G):
    new_G = {}
    for k in G.keys():
        new_G[k] = deepcopy_set(G[k])
    return new_G


def deepcopy_set(s):
    return set(list(s))


def getColorMap(obj_list):
    # set colormap
    gist_ncar = plt.get_cmap('gist_ncar')
    cNorm = colors.Normalize(vmin=0, vmax=1)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)

    color_values = linspace(0.1, 0.9, len(obj_list))
    color_pool = {} 
    for ii, obj in enumerate(obj_list):
        color_pool[obj] = scalarMap.to_rgba(color_values[ii])

    return color_pool


if __name__ == '__main__':
    pass
