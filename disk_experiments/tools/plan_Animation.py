import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from tools.util import poly_disk

import copy
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
from matplotlib.path import Path
import matplotlib.patches as patches
from shapely.geometry import Polygon


def setupPlot(HEIGHT, WIDTH):
    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()
    return fig, ax

def createPolygonPatch(polygon, color, zorder=1):
    verts = []
    codes = []
    for v in range(0, len(polygon)):
        verts.append(polygon[v])
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)
    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    patch = patches.PathPatch(path, facecolor=color, lw=0.5, zorder=zorder)
    return patch


def createPolygonPatch_distinct(polygon, color, isGoal, zorder=1, alpha=1):
    verts = []
    codes = []
    for v in range(0, len(polygon)):
        verts.append(polygon[v])
        if v == 0:
            codes.append(Path.MOVETO)
        else:
            codes.append(Path.LINETO)

    verts.append(verts[0])
    codes.append(Path.CLOSEPOLY)
    path = Path(verts, codes)
    if isGoal:
        patch = patches.PathPatch(path, linestyle='--', edgecolor=color, facecolor="white", lw=2, zorder=zorder, alpha = alpha)
    else:
        patch = patches.PathPatch(path, facecolor=color, lw=2, zorder=zorder, alpha=alpha)

    return patch

def getColorMap(obj_list):
    # set colormap
    gist_ncar = plt.get_cmap('gist_ncar')
    cNorm = colors.Normalize(vmin=0, vmax=1)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)

    color_values = np.linspace(0.1, 0.9, len(obj_list))
    color_pool = {} 
    for ii, obj in enumerate(obj_list):
        color_pool[obj] = scalarMap.to_rgba(color_values[ii])

    return color_pool


def plan_animation(HEIGHT, WIDTH, start_arr, goal_arr, radius, action_list):
    ### polygon shape
    objectShape = np.array(poly_disk([0, 0], radius, 3))

    obj_list = start_arr.keys()
    current_pts = copy.deepcopy(start_arr)
    goal_pts = goal_arr
    isGoalReached = {}
    for i in obj_list:
        isGoalReached[i] = False
    
    ### get some colors from colormap
    color_pool = getColorMap(obj_list)

    ### set the canvas
    fig, ax = setupPlot(HEIGHT, WIDTH)
    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    ax.plot(wallx, wally, 'blue')
    plt.show(block=False)

    ### give a prompt to start animation
    input("Press <ENTER> to start animation")
    # input("Press <ENTER> to start animation")

    istransfer = False
    gripper_position = (WIDTH/2.0, HEIGHT/2.0)

    for action in action_list:
        # transit path
        pt1 = gripper_position
        pt2 = action[1]

        step1 = int(abs(pt1[0] - pt2[0]) / 50.0)
        step2 = int(abs(pt1[1] - pt2[1]) / 50.0)
        if step1 == 0 and step2 == 0:
            n_steps = 1
        else:
            n_steps = max(step1, step2)
        for step in range(n_steps + 1):
            ### update current_pts
            gripper_position = (
                pt1[0] + (pt2[0] - pt1[0]) / n_steps * step, pt1[1] + (pt2[1] - pt1[1]) / n_steps * step
            )

            ax.cla()
            ax.plot(wallx, wally, 'blue')

            ### plot the gripper
            polygon = Polygon(objectShape + gripper_position)
            patch = createPolygonPatch_distinct(list(polygon.exterior.coords), 'darkgray', alpha = 0.7, isGoal=False, zorder=30)
            ax.add_patch(patch)

            ### plot the objects
            for obj in current_pts.keys():
                ### current
                polygon = Polygon(objectShape + current_pts[obj])
                patch = createPolygonPatch_distinct(list(polygon.exterior.coords), color_pool[obj], isGoal=False, zorder=3)
                ax.add_patch(patch)
                ax.text(current_pts[obj][0], current_pts[obj][1], str(obj), fontweight='bold', fontsize=16, zorder=3)
                ### goal
                if not isGoalReached[obj]:
                    polygon = Polygon(objectShape + goal_pts[obj])
                    patch = createPolygonPatch_distinct(
                        list(polygon.exterior.coords), color_pool[obj], isGoal=True, zorder=1
                    )
                    ax.add_patch(patch)
                    ax.text(goal_pts[obj][0], goal_pts[obj][1], str(obj), fontweight='bold', fontsize=16, zorder=1)

            plt.pause(0.00005)
        
        
        
        # transfer path
        current_obj = action[0]
        pt1 = action[1]
        pt2 = action[2]

        step1 = int(abs(pt1[0] - pt2[0]) / 50.0)
        step2 = int(abs(pt1[1] - pt2[1]) / 50.0)
        if step1 == 0 and step2 == 0:
            n_steps = 1
        else:
            n_steps = max(step1, step2)
        for step in range(n_steps + 1):
            ### update current_pts
            current_pts[current_obj] = (
                pt1[0] + (pt2[0] - pt1[0]) / n_steps * step, pt1[1] + (pt2[1] - pt1[1]) / n_steps * step
            )
            gripper_position = current_pts[current_obj]
            ### check if the current object reaches the goal
            if current_pts[current_obj] == goal_pts[current_obj]:
                isGoalReached[current_obj] = True

            ax.cla()
            ax.plot(wallx, wally, 'blue')

            ### plot the gripper
            polygon = Polygon(objectShape + gripper_position)
            patch = createPolygonPatch_distinct(list(polygon.exterior.coords), 'darkgray', isGoal=False, zorder=30, alpha = 0.7)
            ax.add_patch(patch)

            ### plot the objects
            for obj in current_pts.keys():
                if obj == current_obj:
                    current_bonus = 10
                else:
                    current_bonus = 0
                ### current
                polygon = Polygon(objectShape + current_pts[obj])
                patch = createPolygonPatch_distinct(list(polygon.exterior.coords), color_pool[obj], isGoal=False, zorder=3+current_bonus)
                ax.add_patch(patch)
                ax.text(current_pts[obj][0], current_pts[obj][1], str(obj), fontweight='bold', fontsize=16, zorder=3+current_bonus)
                ### goal
                if not isGoalReached[obj]:
                    polygon = Polygon(objectShape + goal_pts[obj])
                    patch = createPolygonPatch_distinct(
                        list(polygon.exterior.coords), color_pool[obj], isGoal=True, zorder=1
                    )
                    ax.add_patch(patch)
                    ax.text(goal_pts[obj][0], goal_pts[obj][1], str(obj), fontweight='bold', fontsize=16, zorder=1)

            plt.pause(0.00005)

    plt.show()
    return

def demo_animation():
    HEIGHT = 10
    WIDTH = 7
    start_arr = {
        0:(5.0,1.1),
        1:(3.9,4.8),
        2:(5.8,7.2),
        3:(1.2,2.3),
        4:(1.0,5.6),
        5:(3.1,7.2)
    } 
    goal_arr = {
        0:(2.0,2.0),
        1:(2.0,5.0),
        2:(2.0,8.0),
        3:(5.0,2.0),
        4:(5.0,5.0),
        5:(5.0,8.0)
    }
    radius = 1.0
    action_list = [
        [0,(5.0,1.1), (9.0,5.0)],
        [3,(1.2,2.3), (5.0,2.0)],
        [0,(9.0,5.0), (2.0,2.0)],
        [1,(3.9,4.8), (9.0,5.0)],
        [4,(1.0,5.6), (5.0,5.0)],
        [1,(9.0,5.0), (2.0,5.0)],
        [2,(5.8,7.2), (9.0,5.0)],
        [5,(3.1,7.2), (5.0,8.0)],
        [2,(9.0,5.0), (2.0,8.0)]
    ]
    
    ### polygon shape
    objectShape = np.array(poly_disk([0, 0], radius, 3))

    obj_list = start_arr.keys()
    current_pts = copy.deepcopy(start_arr)
    goal_pts = goal_arr
    isGoalReached = {}
    for i in obj_list:
        isGoalReached[i] = False
    
    ### get some colors from colormap
    color_pool = getColorMap(obj_list)

    ### set the canvas
    fig, ax = setupPlot(10, 10)
    wallx = [0, WIDTH, WIDTH, 0, 0]
    wally = [0, 0, HEIGHT, HEIGHT, 0]
    ax.plot(wallx, wally, 'blue')
    bufferx = [8,10,10,8,8]
    buffery = [4,4,6,6,4]
    ax.plot(bufferx, buffery, 'blue', linestyle='--')
    plt.show(block=False)

    ### give a prompt to start animation
    raw_input("Press <ENTER> to start animation")

    istransfer = False
    gripper_position = (WIDTH/2.0, HEIGHT/2.0)

    for action in action_list:
        # transit path
        pt1 = gripper_position
        pt2 = action[1]

        step1 = int(abs(pt1[0] - pt2[0]) )
        step2 = int(abs(pt1[1] - pt2[1]) )
        if step1 == 0 and step2 == 0:
            n_steps = 1
        else:
            n_steps = max(step1, step2)
        for step in range(n_steps + 1):
            ### update current_pts
            gripper_position = (
                pt1[0] + (pt2[0] - pt1[0]) / n_steps * step, pt1[1] + (pt2[1] - pt1[1]) / n_steps * step
            )

            ax.cla()
            ax.plot(wallx, wally, 'blue')
            ax.plot(bufferx, buffery, 'blue', linestyle='--')

            ### plot the gripper
            polygon = Polygon(objectShape + gripper_position)
            patch = createPolygonPatch_distinct(list(polygon.exterior.coords), 'darkgray', alpha = 0.7, isGoal=False, zorder=30)
            ax.add_patch(patch)

            ### plot the objects
            for obj in current_pts.keys():
                ### current
                polygon = Polygon(objectShape + current_pts[obj])
                patch = createPolygonPatch_distinct(list(polygon.exterior.coords), color_pool[obj], isGoal=False, zorder=3)
                ax.add_patch(patch)
                ax.text(current_pts[obj][0], current_pts[obj][1], str(obj), fontweight='bold', fontsize=16, zorder=3)
                ### goal
                if not isGoalReached[obj]:
                    polygon = Polygon(objectShape + goal_pts[obj])
                    patch = createPolygonPatch_distinct(
                        list(polygon.exterior.coords), color_pool[obj], isGoal=True, zorder=1
                    )
                    ax.add_patch(patch)
                    ax.text(goal_pts[obj][0], goal_pts[obj][1], str(obj), fontweight='bold', fontsize=16, zorder=1)

            plt.pause(0.00005)
        
        
        
        # transfer path
        current_obj = action[0]
        pt1 = action[1]
        pt2 = action[2]

        step1 = int(abs(pt1[0] - pt2[0]))
        step2 = int(abs(pt1[1] - pt2[1]) )
        if step1 == 0 and step2 == 0:
            n_steps = 1
        else:
            n_steps = max(step1, step2)
        for step in range(n_steps + 1):
            ### update current_pts
            current_pts[current_obj] = (
                pt1[0] + (pt2[0] - pt1[0]) / n_steps * step, pt1[1] + (pt2[1] - pt1[1]) / n_steps * step
            )
            gripper_position = current_pts[current_obj]
            ### check if the current object reaches the goal
            if current_pts[current_obj] == goal_pts[current_obj]:
                isGoalReached[current_obj] = True

            ax.cla()
            ax.plot(wallx, wally, 'blue')
            ax.plot(bufferx, buffery, 'blue', linestyle='--')

            ### plot the gripper
            polygon = Polygon(objectShape + gripper_position)
            patch = createPolygonPatch_distinct(list(polygon.exterior.coords), 'darkgray', isGoal=False, zorder=30, alpha = 0.7)
            ax.add_patch(patch)

            ### plot the objects
            for obj in current_pts.keys():
                if obj == current_obj:
                    current_bonus = 10
                else:
                    current_bonus = 0
                ### current
                polygon = Polygon(objectShape + current_pts[obj])
                patch = createPolygonPatch_distinct(list(polygon.exterior.coords), color_pool[obj], isGoal=False, zorder=3+current_bonus)
                ax.add_patch(patch)
                ax.text(current_pts[obj][0], current_pts[obj][1], str(obj), fontweight='bold', fontsize=16, zorder=3+current_bonus)
                ### goal
                if not isGoalReached[obj]:
                    polygon = Polygon(objectShape + goal_pts[obj])
                    patch = createPolygonPatch_distinct(
                        list(polygon.exterior.coords), color_pool[obj], isGoal=True, zorder=1
                    )
                    ax.add_patch(patch)
                    ax.text(goal_pts[obj][0], goal_pts[obj][1], str(obj), fontweight='bold', fontsize=16, zorder=1)

            plt.pause(0.00005)

    plt.show()
    return

if __name__ == '__main__':
    demo_animation()
  