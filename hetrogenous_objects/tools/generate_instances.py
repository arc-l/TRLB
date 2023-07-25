# Generate all kinds of arrangements
# Maintainers: Tanay, Kai

from json import dump, dumps
from logging import raiseExceptions
from math import *
from shapely.geometry import Polygon
import numpy as np
import os
import sys

if __name__ == '__main__':
    sys.path.insert(0,os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from tools.util import poly_disc, poly_stick, polysCollide
from tools.general_objects import get_poly

from tools.push_for_dense_cases import generate_dense_arrangement

curr_dir = os.path.dirname(os.path.abspath(__file__))

MAX_ITERATIONS = 10000

def generate_cuboid(area,lw_ratio=1):
    w = sqrt(area / lw_ratio)
    l = sqrt(area*lw_ratio)
    return [l, w]

def generate_disc(area,lw_ratio=1):
    return generate_oval(area,lw_ratio=lw_ratio)

def generate_oval(area, lw_ratio=1):
    w = sqrt( (4*area) / (pi * lw_ratio) )
    l = lw_ratio * w
    return [l, w]


def generate_two_sized_discs(density, HEIGHT, WIDTH, n_objects, n_small=None, size_proportion=0.25):
    '''
    generate two sized discs 
    '''
    if n_small == None:
        n_small = n_objects//2
    
    total_area = density * HEIGHT * WIDTH
    small_disc_area = total_area/(n_small+(n_objects-n_small)/size_proportion)
    large_disc_area = small_disc_area/size_proportion
    areas = [small_disc_area]*n_small + [large_disc_area]*(n_objects-n_small)
    
    objects = {}
    for i in range(n_objects):
        ## Make Disc ##
        objects[i] = tuple( [None, None, None] + ["disc"] + generate_disc(areas[i]))
    
    return objects
    

def generate_identical_discs(density, HEIGHT, WIDTH, n_objects):
    '''
    generate identical discs
    '''
    return generate_two_sized_discs(density, HEIGHT, WIDTH, n_objects, n_small=0)


def generate_various_ellipses(density, HEIGHT, WIDTH, n_objects, size_proportion=0.0625):
    '''
    generate two sized ellipses with various width/height ratio
    '''
    areas = np.random.uniform(low=size_proportion,high=1,size=n_objects)
    total_area = density * HEIGHT * WIDTH
    areas = areas/np.sum(areas)*total_area
    
    
    objects = {}
    for i in range(n_objects):
        ## Make Disc ##
        objects[i] = tuple( 
                        [None, None, None] + ["disc"] + 
                        generate_disc(areas[i],lw_ratio=np.random.uniform(low=0.66,high=1))
                        )
    
    return objects


def generate_various_sticks(density, HEIGHT, WIDTH, n_objects, size_proportion=0.25):
    '''
    generate two sized ellipses with various width/height ratio
    '''
    areas = np.random.uniform(low=size_proportion,high=1,size=n_objects)
    total_area = density * HEIGHT * WIDTH
    areas = areas/np.sum(areas)*total_area
    
    
    objects = {}
    for i in range(n_objects):
        ## Make Disc ##
        objects[i] = tuple( 
                        [None, None, None] + ["cuboid"] + 
                        generate_cuboid(areas[i],lw_ratio=np.random.uniform(low=0.66,high=1))
                        )
    
    return objects


def generate_thin_sticks(density, HEIGHT, WIDTH, n_objects, size_proportion=0.25):
    '''
    generate two sized ellipses with various width/height ratio
    '''
    areas = np.random.uniform(low=size_proportion,high=1,size=n_objects)
    total_area = density * HEIGHT * WIDTH
    areas = areas/np.sum(areas)*total_area
    
    
    objects = {}
    for i in range(n_objects):
        ## Make Disc ##
        objects[i] = tuple( 
                        [None, None, None] + ["cuboid"] + 
                        generate_cuboid(areas[i],lw_ratio=np.random.uniform(low=0.2,high=0.5))
                        )
    
    return objects


def generate_Klotski_blocks(density=0.3,HEIGHT=1000, WIDTH=1000, n_objects=20, size_proportion=0.11,num_large=2):
    ''' generate Klotski demos with a unique large object '''
    total_area = density * HEIGHT * WIDTH
    small_area = total_area/((n_objects-num_large)+num_large/size_proportion)
    large_area = small_area/size_proportion
    areas = [large_area]*num_large + [small_area]*(n_objects-num_large)
    
    objects = {}
    for i in range(n_objects):
        ## Make Disc ##
        objects[i] = tuple( 
                        [None, None, None] + ["cuboid"] + 
                        generate_cuboid(areas[i],lw_ratio=1)
                        )
    
    return objects
    

def generate_organized__Klotski_blocks_one_large_side_3(density=0.3,HEIGHT=1000, WIDTH=1000, n_objects=20):
    ''' generate Klotski demos with a unique large object '''
    size_proportion=0.11
    num_large=1
    
    total_area = density * HEIGHT * WIDTH
    small_area = total_area/((n_objects-num_large)+num_large/size_proportion)
    large_area = small_area/size_proportion
    areas = [large_area]*num_large + [small_area]*(n_objects-num_large)
    
    long_side = sqrt(large_area)
    short_side = sqrt(small_area)
    clearance = 10
    
    objects = {}
    # first two layers
    objects[0] = tuple(
        [long_side/2+clearance, long_side/2+clearance, 0] + 
        ["cuboid"] + [long_side]*2
    )
    
    curr_objID = 1
    
    remaining_space = WIDTH - long_side - clearance
    num_fit_in = int(remaining_space//(short_side+clearance))
    for i in range(num_fit_in):
        objects[curr_objID] = tuple(
            [long_side+clearance+(i+1)*(short_side+clearance)-0.5*short_side, short_side/2+clearance, 0] + 
            ["cuboid"] + [short_side]*2
        )
        curr_objID += 1
        if curr_objID >= n_objects:
            return objects
    
    for i in range(num_fit_in):
        objects[curr_objID] = tuple(
            [long_side+clearance+(i+1)*(short_side+clearance)-0.5*short_side, long_side-short_side/2+clearance, 0] + 
            ["cuboid"] + [short_side]*2
        )
        curr_objID += 1
        if curr_objID >= n_objects:
            return objects
    
    # regular layers
    num_fit_in = int(WIDTH//(short_side+clearance))
    num_small_layer = 0
    while 1:
        for i in range(num_fit_in):
            objects[curr_objID] = tuple(
                [(i+1)*(short_side+clearance)-0.5*short_side, long_side+clearance+(num_small_layer+1)*(short_side+clearance)-short_side/2, 0] + 
                ["cuboid"] + [short_side]*2
            )
            curr_objID += 1
            if curr_objID >= n_objects:
                return objects
        num_small_layer += 1
    
    return objects
    
    
def generate_organized__Klotski_blocks_two_large_side_3(density=0.3,HEIGHT=1000, WIDTH=1000, n_objects=20):
    ''' generate Klotski demos with a unique large object '''
    size_proportion=0.11
    num_large=2
    
    total_area = density * HEIGHT * WIDTH
    small_area = total_area/((n_objects-num_large)+num_large/size_proportion)
    large_area = small_area/size_proportion
    areas = [large_area]*num_large + [small_area]*(n_objects-num_large)
    
    long_side = sqrt(large_area)
    short_side = sqrt(small_area)
    clearance = 10
    
    objects = {}
    # first two layers
    objects[0] = tuple(
        [long_side/2+clearance, long_side/2+clearance, 0] + 
        ["cuboid"] + [long_side]*2
    )
    
    objects[1] = tuple(
        [1.5*long_side+2*clearance, long_side/2+clearance, 0] + 
        ["cuboid"] + [long_side]*2
    )
    
    curr_objID = 2
    
    remaining_space = WIDTH - 2*(long_side - clearance)
    num_fit_in = int(remaining_space//(short_side+clearance))
    for i in range(num_fit_in):
        objects[curr_objID] = tuple(
            [2*(long_side+clearance)+(i+1)*(short_side+clearance)-0.5*short_side, short_side/2+clearance, 0] + 
            ["cuboid"] + [short_side]*2
        )
        curr_objID += 1
        if curr_objID >= n_objects:
            return objects
    
    for i in range(num_fit_in):
        objects[curr_objID] = tuple(
            [2*(long_side+clearance)+(i+1)*(short_side+clearance)-0.5*short_side, long_side-short_side/2+clearance, 0] + 
            ["cuboid"] + [short_side]*2
        )
        curr_objID += 1
        if curr_objID >= n_objects:
            return objects
    
    # regular layers
    num_fit_in = int(WIDTH//(short_side+clearance))
    num_small_layer = 0
    while 1:
        for i in range(num_fit_in):
            objects[curr_objID] = tuple(
                [(i+1)*(short_side+clearance)-0.5*short_side, long_side+clearance+(num_small_layer+1)*(short_side+clearance)-short_side/2, 0] + 
                ["cuboid"] + [short_side]*2
            )
            curr_objID += 1
            if curr_objID >= n_objects:
                return objects
        num_small_layer += 1
    
    return objects
    

def generate_random_ellipses_n_sticks(density, HEIGHT, WIDTH, n_objects, size_proportion=0.05, ratio=0.33):
    '''
    generate two sized ellipses with various width/height ratio
    '''
    areas = np.random.uniform(low=size_proportion,high=1,size=n_objects)
    total_area = density * HEIGHT * WIDTH
    areas = areas/np.sum(areas)*total_area
    
    objects = {}
    for i in range(n_objects):
        ## Make general objects ##
        shape = np.random.choice(['disc','cuboid'])
        if shape == 'disc':
            objects[i] = tuple( 
                            [None, None, None] + [shape] + 
                            generate_disc(areas[i],lw_ratio=np.random.uniform(low=ratio,high=1))
                            )
        elif shape == 'cuboid':
            objects[i] = tuple( 
                            [None, None, None] + [shape] + 
                            generate_cuboid(areas[i],lw_ratio=np.random.uniform(low=ratio,high=1))
                            )
        else:
            raise ValueError('Wrong type')
    
    return objects


    
def create_Tetris_instance(exampleID,num_instance=50,label='test'):
    if exampleID == 0:
        scale = 700/7
        objects = {
            0:[None,None,None,'tetris_T',scale,scale],
            1:[None,None,None,'tetris_S',scale,scale],
            2:[None,None,None,'tetris_S',scale,scale],
            3:[None,None,None,'tetris_T',scale,scale],
            4:[None,None,None,'tetris_Z',scale,scale],
            5:[None,None,None,'tetris_T',scale,scale],
            6:[None,None,None,'tetris_Z',scale,scale],
            7:[None,None,None,'tetris_I',scale,scale]
        }
    create_instances_from_object_list(objects,len(objects),0,num_instance,label+str(exampleID))


def create_YCB_instance(exampleID,num_instance=50,label='test'):
    if exampleID == 0: # fruits
        scale = 1000/0.5
        object_list = [
            'YcbBanana',
            'YcbStrawberry',
            'YcbPear',
            'YcbApple',
            'YcbPeach',
            'YcbOrange',
            'YcbPlum'
        ]*2
        objects = {
            i:[None,None,None,name,scale,scale] for i, name in enumerate(object_list)}
    elif exampleID == 1: # kitchen items
        scale = 1000/0.7
        object_list = [
        'YcbPitcher',
        'YcbBleach',
        'YcbWindex',
        'YcbBowl',
        'YcbMug',
        'YcbSponge',
        # 'YcbSkillet',
        'YcbLid',
        'YcbPlate',
        'YcbFork',
        'YcbSpoon',
        'YcbKnife',
        ]
        objects = {
            i:[None,None,None,name,scale,scale] for i, name in enumerate(object_list)}
        
    create_instances_from_object_list(objects,len(objects),0,num_instance,label+str(exampleID))



def generate_objects(density, HEIGHT, WIDTH, n_objects, n_cuboids):
    average_area = ( density * HEIGHT * WIDTH ) / n_objects
    areas = list( np.random.normal(average_area, average_area*0.3, n_objects-1) )
    areas.append( (density * HEIGHT * WIDTH) - sum(areas) )
 
    objects = {}
    cuboid_counter = 0
    for obj_idx in range(n_objects):
        if cuboid_counter < n_cuboids:
            ## Make Cubiod ##
            objects[obj_idx] = tuple( [None, None, None] + ['cuboid'] + generate_cuboid(areas[obj_idx]) )
            cuboid_counter += 1
        else:
            ## Make Disc ##
            objects[obj_idx] = tuple( [None, None, None] + ["disc"] + generate_disc(areas[obj_idx]) )
            
    return objects


def create_instances_from_object_list(objects,n_obj,Density=0.3,num_instance=50,label='test', **kwargs):
    '''
    input a object list
    objects[obj_ID] = [None,None,None,'object_name',W,L]
    '''
    i = 0
    while i < num_instance:
        try:
            initial_poses = repeated_sampling(1000, 1000, objects.copy())
            final_poses = repeated_sampling(1000, 1000, objects.copy())
            folder_name = os.path.join(
                os.path.dirname(curr_dir),
                'instances',
                label,
                'Density='+"{:.2f}".format(Density),
                'numObjs='+str(n_obj)
                )
            if not os.path.exists(folder_name):
                os.makedirs(folder_name)
            file_name = os.path.join(
                folder_name,
                str(i)+"_"+str(n_obj)+"_"+"{:.2f}".format(Density)+'.json'
            )
            format_and_save(Density, 1000, 1000, initial_poses, final_poses, file_name)
            print(i, "Success")
        except ValueError:
        # except TimeoutError:
            print(i, "failed, trying again")
            i -= 1
        i+=1


def valid(obj, workspace, placed_objects):
    #Check if obj overlaps with any other placed object
    for obs in placed_objects:
        if polysCollide(obj, obs):
            return False
    
    #Check if obj lies within the workspace or not
    workspace_polygon = Polygon(workspace)
    obj_polygon = Polygon(obj)
    if not workspace_polygon.contains(obj_polygon):
        return False

    return True



def format_and_save(density, HEIGHT, WIDTH, initial_poses, final_poses, op_filename):
    if len(initial_poses) != len(final_poses):
        raise Exception("Number of objects incorrect")

    arr_json = {}

    arr_json['Workspace_Density'] = density
    arr_json['Workspace_Height'] = HEIGHT
    arr_json['Workspace_Width'] = WIDTH
    arr_json['number_of_objects'] = len(initial_poses)
    arr_json['object_list'] = []

    for obj_idx in initial_poses:
        obj_i = initial_poses[obj_idx]
        obj_g = final_poses[obj_idx]

        temp_obj_dict = {}
        temp_obj_dict['Object_Shape'] = obj_i[3]
        temp_obj_dict['Object_Length'] = obj_i[4]
        temp_obj_dict['Object_Width'] = obj_i[5]

        temp_obj_dict['initial_pose'] = obj_i[:3]
        temp_obj_dict['goal_pose'] = obj_g[:3]

        arr_json['object_list'].append(temp_obj_dict)
    
    with open(op_filename, "w") as w_file:
        dump(arr_json, w_file, indent=2)
    return

def get_points(obj):
    """
    Given an object tuple (x, y, theta, type, l, h) it returns the set of points used to 
    approximate the shape
    """
    if obj[3] == 'cuboid':
        return poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
    elif obj[3] == 'disc':
        return poly_disc((obj[0], obj[1]), obj[2], obj[4], obj[5])
    else:
        return get_poly(obj+[0])

def sample(obj, obstacles, L, H):
    """
    Find a safe pose for obj given some obstacles - repeated sampling
    """
    workspace = poly_stick((L/2, H/2), 0, L, H)
    for i in range(MAX_ITERATIONS):
        x = np.random.uniform(0, L, 1)[0]
        y  = np.random.uniform(0, H, 1)[0]
        theta = np.random.uniform(0, 6.28, 1)[0]
        new_obj = [x, y, theta, obj[3], obj[4], obj[5]]
        obj_points = get_points(new_obj)

        if valid(obj_points, workspace, obstacles):
            return new_obj

    return -1

def repeated_sampling(L, H, objects):
    """
    Generate poses for all objects by repeatedly sampling till valid poses are found
    """
    for curr_idx in objects:
        obstacles = [get_points(objects[t]) for t in range(0, curr_idx)]
        obj_with_valid_pose = sample(objects[curr_idx], obstacles, L, H)
        
        if obj_with_valid_pose == -1:
            raise ValueError("Repeated Sampling failed")

        objects[curr_idx] = obj_with_valid_pose
    
    return objects
        

def create_instances(n_obj,Density=0.3,num_instance=50,label='test', object_generator=generate_identical_discs, **kwargs):
    '''
    create instances
    '''
    i = 0
    while i < num_instance:
        try:
            objects = object_generator(Density, 1000, 1000, n_obj, **kwargs)
            initial_poses = repeated_sampling(1000, 1000, objects.copy())
            final_poses = repeated_sampling(1000, 1000, objects.copy())
            # final_poses = objects.copy()
            folder_name = os.path.join(
                os.path.dirname(curr_dir),
                'instances',
                label,
                'Density='+"{:.2f}".format(Density),
                'numObjs='+str(n_obj)
                )
            if not os.path.exists(folder_name):
                os.makedirs(folder_name)
            file_name = os.path.join(
                folder_name,
                str(i)+"_"+str(n_obj)+"_"+"{:.2f}".format(Density)+'.json'
            )
            format_and_save(Density, 1000, 1000, initial_poses, final_poses, file_name)
            print(i, "Success")
        except ValueError:
            print(i, "failed, trying again")
            i -= 1
        i+=1


def create_dense_instances(n_obj,Density=0.3,num_instance=50,label='test', object_generator=generate_identical_discs, **kwargs):
    '''
    create instances
    '''
    i = 0
    while i < num_instance:
        try:
            objects = object_generator(Density, 1000, 1000, n_obj, **kwargs)
            initial_poses = generate_dense_arrangement(objects.copy())
            final_poses = generate_dense_arrangement(objects.copy())
            folder_name = os.path.join(
                os.path.dirname(curr_dir),
                'instances',
                label,
                'Density='+"{:.2f}".format(Density),
                'numObjs='+str(n_obj)
                )
            if not os.path.exists(folder_name):
                os.makedirs(folder_name)
            file_name = os.path.join(
                folder_name,
                str(i)+"_"+str(n_obj)+"_"+"{:.2f}".format(Density)+'.json'
            )
            format_and_save(Density, 1000, 1000, initial_poses, final_poses, file_name)
            print(i, "Success")
        except Warning as e:
            print(e)
            print(i, "failed, trying again")
            i -= 1
        i+=1


def modify_OBJ(file_dir, new_file_dir):
    with open(file_dir, 'r') as f:
        lines = []
        for line in f.readlines():
            if (line[:2] == 'v ') or (line[:2] == 'vn '):
                new_line = line[:-1]
                words = new_line.split()
                new_line = words[0] + ' ' \
                    + words[3] + ' ' \
                    + words[1] + ' ' \
                    + words[2] + '\n'
                lines.append(new_line)
            else:
                lines.append(line)
    with open(new_file_dir, 'w') as new_f:
        new_f.writelines(lines)
    




if __name__ == '__main__':
    # curr_dir = os.path.dirname(os.path.abspath(__file__))
    # modify_OBJ(
    #     os.path.join(curr_dir, 'cylinder.obj'),
    #     os.path.join(curr_dir, 'modified_cylinder.obj')
    # )
    create_dense_instances(20, 0.5,num_instance=1, object_generator=generate_random_ellipses_n_sticks)
    