'''
Input: pose((x,y,theta, obj_name, Width, Height))
Output objects with general shapes
'''
import os
import ujson
import numpy as np
import math
import cv2


model_dir = os.path.join( 
                        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'general_shaped_objects'
                        )

supported_model_list = []

for filename in os.listdir(model_dir):
    if filename.endswith('.json'):
        supported_model_list.append(filename[:-5])


model_info = {}

for model in supported_model_list:
    file_name = os.path.join(
        model_dir,model+'.json'
    )
    with open(file_name, 'r') as f:
        model_dict = ujson.load(f)
    model_info[model] = {}
    model_info[model]['poly'] = model_dict['points']
    model_info[model]['radius'] = model_dict['radius']



def get_poly(pose):
    ''' get polygon of general shapes '''
    if len(pose) == 7:
        x,y,theta,obj_name,Width,Height,_ = pose
    elif len(pose) == 6:
        x,y,theta,obj_name,Width,Height = pose
    scale = max(Width, Height)
    original_poly = np.array(model_info[obj_name]['poly'])
    ones = np.ones((len(original_poly),1))
    Pts = np.concatenate([original_poly, ones], axis=1)
    T = np.array(
        [
            [math.cos(theta)*scale, -math.sin(theta)*scale, x],
            [math.sin(theta)*scale, math.cos(theta)*scale, y]
        ]
    )
    new_Pts = np.transpose(np.dot(T,np.transpose(Pts)))
    
    
    return [list(r) for r in new_Pts]

def get_radius(pose):
    ''' get radius of general shapes '''
    _,_,_,obj_name,Width,Height,_ = pose
    scale = max(Width, Height)
    return model_info[obj_name]['radius']*scale


def get_area(pose):
    ''' get area '''
    poly = get_poly(pose)
    poly.reverse()
    area = cv2.contourArea(np.array(poly).astype(np.float32))
    return area
    

def get_AABB(pose):
    poly = get_poly(pose)
    print(poly)
    min_x = min([p[0] for p in poly])
    min_y = min([p[1] for p in poly])
    max_x = max([p[0] for p in poly])
    max_y = max([p[1] for p in poly])
    return ((min_x,min_y),(max_x, max_y))

if __name__ == '__main__':
    poly = get_poly(
        (100,200,0,'monitor', 2,1,0)
    )
    radius = get_radius(
        (100,200,0,'monitor', 2,1,0)
    )
    area = get_area(
        (100,200,0,'monitor', 200,200,0)
    )
    AABB = get_AABB(
        (100,200,0,'monitor', 200,200,0)
    )
    print(AABB)
    