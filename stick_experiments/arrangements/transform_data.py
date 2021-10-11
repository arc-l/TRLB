import os
import json
import math
import numpy as np
# import matplotlib
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.colors as colors
import matplotlib.cm as cmx
from shapely.geometry import Polygon

def main():        
    dir_name = "/home/kai/Documents/Kai_Gao/git/TRLB/stick_experiments/arrangements_old"

    # g = os.walk(dir_name)
    # print(g)
    # choose densities
    # for path,dir_list,file_list in g:
    for density_name in os.listdir(dir_name):
        if density_name[:2] != 'D=':
            continue
        density_dir = os.path.join(dir_name, density_name)
        if not os.path.isdir(density_dir):
            continue
        Density = float(density_name[2:])
        for num_name in os.listdir(density_dir):
            if num_name[:2] != 'n=':
                continue
            num_dir = os.path.join(density_dir,num_name)
            if not os.path.isdir(num_dir):
                continue
            numobjs = int(num_name[2:])
            print(Density, numobjs)
            id = 0
            for ins_file in os.listdir(num_dir):
                if id > 19:
                    break
                ins_dir = os.path.join(num_dir, ins_file)
                obj_list = get_object_list(ins_dir)
                transform_data(Density, numobjs, id, 1000, 1000, obj_list)
                id += 1


def transform_data(Density, numobjs, id, Width, Height, point_list):
    data = {}
    data['Object_Shape'] = 'cuboid'
    data['Workspace_Density'] = Density
    data['number_of_objects'] = numobjs
    WL_ratio = 0.3
    data['Object_Length'] = math.sqrt( Density * Width * Height/numobjs/WL_ratio)
    data['Object_Width'] = WL_ratio * data['Object_Length']
    data['Workspace_Width'] = Width
    data['Workspace_Height'] = Height
    data['point_list'] = point_list
    my_path = os.path.dirname(__file__)
    file_name = str(id)+'_'+str(numobjs)+'_'+str(round(Density,1))
    save_path = os.path.join(
        my_path, 'D='+str(round(Density,1)), 'n=' + str(numobjs)
    )
    if not os.path.exists(save_path):
        os.makedirs(save_path)

    # save data
    with open(os.path.join(save_path, file_name+'.json'), 'w') as f:
        json.dump(data, f, sort_keys=True, indent=1)
    # # print figure
    # save_figure(save_path, file_name+'.png', data)


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


def save_figure(save_path, file_name, data):
    Radius = data['Radius']
    WIDTH = data['Width']
    HEIGHT = data['Height']

    color_pool = getColorMap(range(len(data['point_list'])))

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='b')
    plt.ion
    ax = fig.subplots()

    wall = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0, 0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1)]:
        # walls = pu.pointList(cont)
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')

    for key, point in enumerate(data['point_list']):
        circle = plt.Circle(point,Radius, facecolor=color_pool[key], lw=2, edgecolor='black',zorder=3)
        ax.add_artist(circle)
        plt.text(point[0], point[1],str(key),zorder=10)
    plt.ioff()
    plt.savefig(os.path.join(save_path,file_name))



def get_object_list(file_name):
    object_list = []
    with open(file_name, 'r') as f:
        for line in f.readlines():
            # line = line.decode('utf-8')
            if line == 'objects\n':
                continue
            else:
                pose = line.split()
                object_list.append((float(pose[0]), float(pose[1]), float(pose[2])))
    return object_list

if __name__ == '__main__':
    main()