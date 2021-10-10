import sys
import os

sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from tools.util import *
import math

import numpy as np
import matplotlib.pyplot as plt
import random
import json
import copy

from shapely.geometry import Polygon

import matplotlib.colors as colors
import matplotlib.cm as cmx

def getColorMap(numObjs):
    # set colormap
    gist_ncar = plt.get_cmap('gist_ncar')
    cNorm = colors.Normalize(vmin=0, vmax=1)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)

    color_values = np.linspace(0, 0.9, numObjs)
    color_pool = [scalarMap.to_rgba(color_values[ii]) for ii in xrange(numObjs)]

    return color_pool

def isCollisionFree(robot, point, obstacles):
    robotAt = np.add(point, robot)
    for poly in obstacles:
        if polysCollide(poly, robotAt):
            return False

    return True


def generate_arrangement():
    WIDTH = 1000
    HEIGHT = WIDTH
    Density = 0.6
    for numObjs in range(10, 11, 5):
        RAD = math.sqrt( Density * WIDTH * HEIGHT/numObjs/math.pi)

        # color_pool = getColorMap(numObjs)
        # wall_pts = pn.Polygon([(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT)])
        # wall_mink = pn.Polygon([(RAD, RAD), (WIDTH - RAD, RAD), (WIDTH - RAD, HEIGHT - RAD), (RAD, HEIGHT - RAD)])

        for iter_ in range(20):
            # print numObjs, iter_
            ### Objs ###
            polygon = np.array(poly_disk([0, 0], RAD, 30))
            Success = False
            while not Success:
                Success = True
                points = []
                objects = []
                for i in range(numObjs):
                    isfree = False
                    timeout = 10
                    while not isfree and timeout > 0:
                        timeout -= 1
                        point = (
                            random.uniform(0 - min(polygon[:, 0]),
                                    WIDTH - max(polygon[:, 0])), random.uniform(0 - min(polygon[:, 1]), HEIGHT - max(polygon[:, 1]))
                        )
                        isfree = isCollisionFree(polygon, point, objects)

                    if timeout <= 0:
                        # print("Failed to generate!", numObjs, Density)
                        Success = False

                    points.append(point)
                    objects.append(Polygon(polygon + point))

            my_path = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))+"/arrangements"
            try:
                os.mkdir(my_path)
            except Exception:
                pass
            my_path = my_path+"/D="+str(Density)
            try:
                os.mkdir(my_path)
            except Exception:
                pass
            my_path = my_path+"/n="+str(numObjs)
            try:
                os.mkdir(my_path)
            except Exception:
                pass
            file_name = str(iter_) + "_" + str(numObjs) + "_" + str(Density) + ".txt"

            ### clear txt ###
            f = open(os.path.join(my_path, file_name), 'w')
        
            f.write( "objects" + '\n')

            for p in points:
                f.write( str(p[0]) + " " + str(p[1]) + '\n')

            
        f.close()
    # return points, Obs_points

def import_arrangements(Height, Width, numObjs, Density):
    instance_num = 200
    instances_choice = np.random.choice(range(instance_num),size=2, replace=False)
    # print instances_choice
    my_path = os.path.abspath(os.path.dirname(os.path.dirname(__file__))) + "/arrangements"
    init_list = range(numObjs)
    final_list = range(numObjs)
    random.shuffle(init_list)
    random.shuffle(final_list)

    points = [[0,0]]*(2*numObjs)

    ### init arr ###
    f = open( my_path+"/D="+str(round(Density,1))+"/n="+str(numObjs)
    +"/"+str(instances_choice[0])+"_"+str(numObjs)+"_"+str(round(Density,1))
    +".txt", 'rb')
    object_index = 0
    for line in f.readlines():
        if line == "objects\n":
            continue
        else:
            pose = line.split()
            points[2*init_list[object_index]] = [float(pose[0]), float(pose[1])]
            object_index += 1
            # print pose
    f.close()

    ### final arr ###
    f = open( my_path+"/D="+str(round(Density,1))+"/n="+str(numObjs)
    +"/"+str(instances_choice[1])+"_"+str(numObjs)+"_"+str(round(Density,1))
    +".txt", 'rb')
    object_index = 0
    for line in f.readlines():
        if line == "objects\n":
            continue
        else:
            pose = line.split()
            points[2*init_list[object_index]+1] = [float(pose[0]), float(pose[1])]
            object_index += 1
            # print pose
    f.close()
    
    return points

def generate_instance(instance_index, Height, Width, numObjs, Density):
    points = import_arrangements(Height, Width, numObjs, Density)
    
    ch1 = {}

    ch1["n"] = numObjs
    ch1["radius"] = math.sqrt( Density * Width * Height/numObjs/math.pi)
    ch1["height"] = Height
    ch1["width"] = Width
    starts = []
    goals = []
    for i in range(numObjs):
        starts.append(list(points[2*i]))
        goals.append(list(points[2*i+1]))
    ch1["starts"] = starts
    ch1["goals"] = goals
    ch1["obstacles"] = []

    ch2 = copy.deepcopy(ch1)

    generate_ch1_instance(ch1, instance_index, Density, numObjs)
    generate_ch2_instance(ch2, instance_index, Density, numObjs)


def batch_generate_instances():
    Height = 1000
    Width = 1000
    num_trials = 500
    for D in [0.1, 0.2, 0.3]:
        for numObjs in range(5, 41, 5):
            for trial in range(num_trials):
                # print D, numObjs, trial
                generate_instance(trial, Height, Width, numObjs, D)


if __name__ == "__main__":
    generate_arrangement()
    # batch_generate_instances()