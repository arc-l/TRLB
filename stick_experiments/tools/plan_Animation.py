import copy
import numpy as np
import math
from matplotlib import pyplot as plt
import matplotlib.colors as colors
import matplotlib.cm as cmx
from matplotlib.path import Path
import matplotlib.patches as patches
from shapely.geometry import Polygon
import os
import sys
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from tools.util import poly_disk, poly_stick


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


def createPolygonPatch_distinct(polygon, color, isGoal, zorder=1):
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
        patch = patches.PathPatch(path, linestyle='--', edgecolor=color, facecolor="white", lw=1, zorder=zorder)
    else:
        patch = patches.PathPatch(path, facecolor=color, lw=2, zorder=zorder)

    return patch

def getColorMap(obj_list):
    # set colormap
    gist_ncar = plt.get_cmap('gist_ncar')
    cNorm = colors.Normalize(vmin=0, vmax=1)
    scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=gist_ncar)

    color_values = np.linspace(0, 0.9, len(obj_list))
    color_pool = {} 
    for ii, obj in enumerate(obj_list):
        color_pool[obj] = scalarMap.to_rgba(color_values[ii])

    return color_pool


def plan_animation(HEIGHT, WIDTH, start_arr, goal_arr, Density, action_list, WL_ratio = 0.3):
    ratio = WL_ratio # Width/Height=0.3
    Length = math.sqrt( Density * WIDTH * HEIGHT/len(start_arr)/ratio)

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
    # raw_input("Press <ENTER> to start animation")
    input("Press <ENTER> to start animation")

    for action in action_list:
        if isinstance(action, str):
            continue
        current_obj = action[0]
        pt1 = action[1]
        pt2 = action[2]


        step1 = int(abs(pt1[0] - pt2[0]) / 50.0)
        step2 = int(abs(pt1[1] - pt2[1]) / 50.0)
        step3 = int(abs(pt1[2] - pt2[2]) / 50.0)
        if step1 == 0 and step2 == 0 and step3 == 0:
            n_steps = 1
        else:
            n_steps = max(step1, step2, step3)
        for step in range(n_steps + 1):
            ### update current_pts
            current_pts[current_obj] = (
                pt1[0] + (pt2[0] - pt1[0]) / n_steps * step, 
                pt1[1] + (pt2[1] - pt1[1]) / n_steps * step,
                pt1[2] + (pt2[2] - pt1[2]) / n_steps * step
            )
            ### check if the current object reaches the goal
            if current_pts[current_obj] == goal_pts[current_obj]:
                isGoalReached[current_obj] = True

            ax.cla()
            ax.plot(wallx, wally, 'blue')

            ### plot the objects
            for obj in current_pts.keys():
                if obj == current_obj:
                    bonus = 5
                else:
                    bonus = 0
                ### current
                polygon = Polygon( np.array(poly_stick(current_pts[obj][:2], current_pts[obj][2], Length, Length*ratio)))
                patch = createPolygonPatch_distinct(list(polygon.exterior.coords), color_pool[obj], isGoal=False, zorder=3+bonus)
                ax.add_patch(patch)
                ax.text(current_pts[obj][0], current_pts[obj][1], str(obj), fontweight='bold', fontsize=10, zorder=3+bonus)
                ### goal
                if not isGoalReached[obj]:
                    polygon = Polygon( np.array(poly_stick(goal_pts[obj][:2], goal_pts[obj][2], Length, Length*ratio)))
                    patch = createPolygonPatch_distinct(
                        list(polygon.exterior.coords), color_pool[obj], isGoal=True, zorder=1
                    )
                    ax.add_patch(patch)
                    ax.text(goal_pts[obj][0], goal_pts[obj][1], str(obj), fontweight='bold', fontsize=10, zorder=1)

            plt.pause(0.00005)

    plt.show()
    return