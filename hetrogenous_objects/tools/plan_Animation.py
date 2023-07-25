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
import time
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from pybullet_object_models import ycb_objects
import pybullet as p
import pybullet_data


from .util import poly_stick, poly_disc
from tools.general_objects import get_poly


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
    current_pts = { k:l[:3] for k,l in start_arr.items()}
    goal_pts = { k:l[:3] for k,l in goal_arr.items()}
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

    obj_shape_map = {}
    for obj in start_arr:
        obj_shape_map[obj] = tuple(start_arr[obj][3:])

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
                if obj_shape_map[obj][0] == 'cuboid':
                    polygon = Polygon( np.array(poly_stick(current_pts[obj][:2], current_pts[obj][2], obj_shape_map[obj][1], obj_shape_map[obj][2])))
                elif obj_shape_map[obj][0] == 'disc':
                    polygon = Polygon( np.array(poly_disc(current_pts[obj][:2], current_pts[obj][2], obj_shape_map[obj][1], obj_shape_map[obj][2])))
                else:
                    print(list(current_pts[obj])+list(obj_shape_map[obj]))
                    polygon = Polygon( np.array(get_poly(list(current_pts[obj])+list(obj_shape_map[obj]))))

                patch = createPolygonPatch_distinct(list(polygon.exterior.coords), color_pool[obj], isGoal=False, zorder=3+bonus)
                ax.add_patch(patch)
                ax.text(current_pts[obj][0], current_pts[obj][1], str(obj), fontweight='bold', fontsize=10, zorder=3+bonus)
                ### goal
                if not isGoalReached[obj]:
                    if obj_shape_map[obj][0] == 'cuboid':
                        polygon = Polygon( np.array(poly_stick(goal_pts[obj][:2], goal_pts[obj][2], obj_shape_map[obj][1], obj_shape_map[obj][2])))
                    elif obj_shape_map[obj][0] == 'disc':
                        polygon = Polygon( np.array(poly_disc(goal_pts[obj][:2], goal_pts[obj][2], obj_shape_map[obj][1], obj_shape_map[obj][2])))
                    else:
                        polygon = Polygon( np.array(get_poly(list(goal_pts[obj])+list(obj_shape_map[obj]))))
                    
                    patch = createPolygonPatch_distinct(
                        list(polygon.exterior.coords), color_pool[obj], isGoal=True, zorder=1
                    )
                    ax.add_patch(patch)
                    ax.text(goal_pts[obj][0], goal_pts[obj][1], str(obj), fontweight='bold', fontsize=10, zorder=1)

            plt.pause(0.00005)

    plt.show()
    return

def plan_pybullet_animation(HEIGHT, WIDTH, start_arr, goal_arr, Density, action_list):

    obj_list = start_arr.keys()
    scale = max(start_arr[0][4:6])
    real_size = HEIGHT/scale
    print('real size', real_size)
    current_pts = { k:l[:3] for k,l in start_arr.items()}
    goal_pts = { k:l[:3] for k,l in goal_arr.items()}
    isGoalReached = {}
    for i in obj_list:
        isGoalReached[i] = False

    ### set the canvas
    # Open GUI and set pybullet_data in the path
    p.connect(p.GUI)
    # p.connect(p.DIRECT)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])

    p.setTimeStep(1. / 240)

    # Load plane contained in pybullet_data
    planeId = p.loadURDF(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "workspace.urdf"),
        [real_size/2,real_size/2,0], globalScaling=(HEIGHT/scale)/30
        )

    # Set gravity for simulation
    p.setGravity(0, 0, 0)

    objID_mapping, max_height = load_objects(start_arr,1/scale)

    # while 1:
    #     p.stepSimulation()
    #     time.sleep(1./240)
    
    ### give a prompt to start animation
    # raw_input("Press <ENTER> to start animation")
    input("Press <ENTER> to start animation")

    obj_shape_map = {}
    for obj in start_arr:
        obj_shape_map[obj] = tuple(start_arr[obj][3:])

    for action in action_list:
        if isinstance(action, str):
            continue
        
        
        current_obj = action[0]
        pt1 = action[1]
        pt2 = action[2]

        # raise the object
        vertical_move(objID_mapping[current_obj], max_height+0.05)

        pos,_ = p.getBasePositionAndOrientation(objID_mapping[current_obj])
        curr_height = pos[2]

        step1 = int(abs(pt1[0] - pt2[0]) / 50.0)
        step2 = int(abs(pt1[1] - pt2[1]) / 50.0)
        step3 = int(abs(pt1[2] - pt2[2]) * 180 / 3.14 / 50.0)
        if step1 == 0 and step2 == 0 and step3 == 0:
            n_steps = 1
        else:
            n_steps = max(step1, step2, step3)
        for step in range(n_steps + 1):
            ### update current_pts
            new_pos = (
                (pt1[0] + (pt2[0] - pt1[0]) / n_steps * step) * (1/scale), 
                (pt1[1] + (pt2[1] - pt1[1]) / n_steps * step) * (1/scale),
                curr_height
            )
            
            new_qua = p.getQuaternionFromEuler(
                [0,0,pt1[2] + (pt2[2] - pt1[2]) / n_steps * step]
            )
            p.resetBasePositionAndOrientation(
                objID_mapping[current_obj],
                new_pos,
                new_qua
            )
            time.sleep(1/n_steps)
            # ### check if the current object reaches the goal
            # if current_pts[current_obj] == goal_pts[current_obj]:
            #     isGoalReached[current_obj] = True


        # drop the object
        vertical_move(objID_mapping[current_obj], -(max_height+0.05))


    input("Finish!")
    return

def load_objects(start_arr,real2sim_scale,scale=1):
    '''
    load the objects so that the bottom of the object is on the ground
    '''
    objID_mapping = {}
    max_height = -1
    for obj, obj_pose in start_arr.items():
        x,y,theta,object_name = obj_pose[:4]
        obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(), object_name, "model.urdf"), [0., 0.0, 1])
        AABB_min,AABB_max = p.getAABB(obj_id)
        height = AABB_max[2] - AABB_min[2]
        if height >= max_height:
            max_height = height
        grounded_height = 1-AABB_min[2]
        p.resetBasePositionAndOrientation(
            obj_id,
            [real2sim_scale*scale*x,real2sim_scale*scale*y,grounded_height],
            p.getQuaternionFromEuler([0,0,theta])
        )
        objID_mapping[obj] = obj_id
        print(object_name, height)
    
    return objID_mapping, max_height
    


def vertical_move(objID, dist):
    '''
    raise objID by dist
    '''
    raise_time = 1
    timestep = 50
    pos,qua = p.getBasePositionAndOrientation(objID)
    pos = list(pos)
    for _ in range(timestep):
        pos[2] += dist/timestep
        p.resetBasePositionAndOrientation( objID, pos, qua)
        time.sleep(raise_time/timestep)
    

def horizontal_move(objID, pose1, pose2):
    '''
    move objID horizontally
    '''
    
