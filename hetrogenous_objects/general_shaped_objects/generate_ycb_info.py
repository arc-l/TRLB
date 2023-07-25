import os
import time
import numpy as np
import pybullet_object_models
from pybullet_object_models import ycb_objects
from pybullet_object_models import graspa_layouts
from pybullet_object_models import superquadric_objects
import matplotlib.pyplot as plt
import cv2
import ujson

import pybullet as p
import pybullet_data
import sys
sys.path.insert(
    0, os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
)
from tools.util import plot_figures

curr_dir = os.path.dirname(os.path.abspath(__file__))
model_dir = curr_dir

def get_json_model(object_name:str):
    # Open GUI and set pybullet_data in the path
    # p.connect(p.GUI)
    p.connect(p.DIRECT)
    # p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    p.setTimeStep(1. / 240)

    # Load plane contained in pybullet_data
    # planeId = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "plane.urdf"))

    # Set gravity for simulation
    p.setGravity(0, 0, 0)


    flags = p.URDF_USE_INERTIA_FROM_FILE
    obj_id = p.loadURDF(os.path.join(ycb_objects.getDataPath(), object_name, "model.urdf"), [0., 0.0 , 0.6], flags=flags)
    # obj_id = p.loadSoftBody(os.path.join(ycb_objects.getDataPath(), object_name, "textured.obj"), [0., 0.0 , 0.8])
    AABB_min, AABB_max = p.getAABB(obj_id)
    print('AABB center', (np.array(AABB_max) + np.array(AABB_min))/2)
    possible_visual_objs = [
        "textured.obj","textured_simple_reoriented.obj","textured_reoriented.obj"
    ]
    for f in possible_visual_objs:
        if os.path.isfile(os.path.join(ycb_objects.getDataPath(), object_name, f)):
            file_name = f
            break
    else:
        raise ValueError("FileNotFound")
    with open(os.path.join(ycb_objects.getDataPath(), object_name, file_name),'r') as f:
        pts = []
        for line in f.readlines():
            if line[:2] == 'v ':
                words = line.split(sep=' ')
                x,y,z = float(words[1]),float(words[2]),float(words[3])
                pts.append((x,y))
        pts = np.array(pts)
    # D = p.getMeshData(obj_id)
    # pts = np.array(D[1])
    # print(pts.shape)
    # plt.scatter(
    #     pts[:,0], pts[:,1]
    # )
    # plt.axis('equal')
    # plt.show()
    # exit()
    # print(p.getAABB(obj_id))
    img_size = 200
    bound = 0.2
    scale = 2*bound/img_size
    cell_offset = img_size//2
    nx, ny = (img_size,img_size)
    
    # x = np.linspace(-bound,bound,nx)
    # y = np.linspace(-bound,bound,ny)
    # xv,yv = np.meshgrid(x,y)
    # xv = xv.flatten().reshape((-1,1))
    # yv = yv.flatten().reshape((-1,1))
    # pts = np.concatenate([xv,yv],axis=1)
    
    hit = np.zeros((ny,nx)).astype(np.uint8)
    for pt in pts:
        x,y = pt
        x_cell = int(x//scale+cell_offset)
        y_cell = int(y//scale+cell_offset)
        hit[y_cell,x_cell] = 1
        
    
    
    # hit = []
    # for pt in pts:
    #     ret = p.rayTest(
    #         (pt[0],pt[1],1),
    #         (pt[0],pt[1],0),
    #     )
    #     # if (pt[0] > 0.2/100*17) and (pt[0] < 0.2/100*18):
    #     #     print(ret)
    #     if len(ret[0]) == 5:
    #         hitID = ret[0][0]
    #     else:
    #         continue
    #     if hitID == obj_id:
    #         hit.append(1)
    #     else:
    #         hit.append(0)
    
    
    
    
    # hit = np.array(hit).reshape((ny,nx))
    hit = cv2.morphologyEx(hit, cv2.MORPH_CLOSE, (3,3))
    plt.imshow(hit)
    plt.show()
    
    cnts,_ = cv2.findContours(hit.astype(np.uint8),1,2)
    max_area = -1
    max_cnt = None
    for cnt in cnts:
        area = cv2.contourArea(cnt)
        if area >= max_area:
            max_area = area
            max_cnt = cnt
    cnt = max_cnt
    # cnt = cnts[0][0]
    poly = cv2.approxPolyDP(cnt, 1, closed=True)
    poly = poly.reshape((-1,2))
    poly = [list(e) for e in poly]
    print(object_name)
    print(len(poly))

    p.disconnect()
    # while 1:
    #     p.stepSimulation()
    #     time.sleep(1./240)
    
    poly_arr = np.array(poly).astype(np.float32)
    poly_arr = poly_arr - cell_offset
    poly_arr = (scale*poly_arr).astype(np.float32)
    # plot_model(tetris_Z)
    center, radius = cv2.minEnclosingCircle(poly_arr)
    center_arr = np.tile(np.array(center), (len(poly),1))
    new_poly_arr = poly_arr-center_arr
    print(center,radius)
    print('adjustment', "<origin rpy=\"0 0 0\" xyz=\"" + str(round(-center[0],3)) + " " + str(round(-center[1],3)) + " 0.0\"/>")
    new_poly_list = [list(e) for e in new_poly_arr ]
    for i in range(-1, len(new_poly_arr)-1):
        x1,y1 = new_poly_arr[i]
        x2,y2 = new_poly_arr[i+1]
        plt.plot([x1,x2],[y1,y2],color='r')
    plt.axis('equal')
    plt.show()
    # print(new_poly_list)
    data_dict = {
        'center' : [0,0],
        'radius' : radius,
        'points' : new_poly_list
    }
    save_file_name = os.path.join(
        curr_dir, object_name+'.json'
    )
    with open(save_file_name, 'w') as f:
        ujson.dump(data_dict,f)


if __name__ == '__main__':
    object_list = [
        # 'YcbBanana',
        # 'YcbStrawberry',
        # 'YcbPear',
        # 'YcbApple',
        # 'YcbPeach',
        # 'YcbOrange',
        # 'YcbPlum',
        # 'YcbPitcher',
        # 'YcbBleach',
        # 'YcbWindex',
        # 'YcbBowl',
        # 'YcbMug',
        # 'YcbSponge',
        # 'YcbSkillet',
        # 'YcbLid',
        # 'YcbPlate',
        # 'YcbFork',
        # 'YcbSpoon',
        # 'YcbKnife',
        # 'YcbSpatula',
    ]
    for obj in object_list:
        get_json_model(obj)