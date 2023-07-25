import matplotlib.pyplot as plt
import numpy as np
import os
import math

import matplotlib.colors as colors
import matplotlib.cm as cmx
from matplotlib.patches import Ellipse, Circle

from shapely.geometry import polygon
import time

from tools.util import poly_stick
from tools.general_objects import get_poly
from Python_MCTS.QuadTree import QuadTree, Item

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


def show_hetrogenous_arrangement(numObjs, Density, start_arr, goal_arr,WIDTH = 1000,HEIGHT = 1000, show_arrangements='all', show_text=True, save_file=''):
    '''
    show_arrangements='all', 'start', 'goal', 
    show_text=True/False, 
    save_file='' or 'path/to/file'
    '''
    
    lw=4
    
    HEIGHT = 1000

    color_pool = getColorMap(start_arr.keys())

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0,0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0,0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1) ]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue',lw=lw)
    
    ## Goal Arrangements ##
    if show_arrangements in ['all', 'goal']:
        for key, obj in goal_arr.items():
            if obj[3] == 'cuboid':
                poly = poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
                poly_patch = plt.Polygon(poly, linestyle='--', edgecolor=color_pool[key], facecolor="white", lw=lw, zorder=2)
                ax.add_artist(poly_patch)
            elif obj[3] =='disc':
                ellipse = Ellipse((obj[0], obj[1]), obj[4], obj[5], obj[2]*180/math.pi, 
                    linestyle="--", edgecolor=color_pool[key], fc="None", lw=lw, zorder=2)
                ax.add_patch(ellipse)
            else:
                poly = get_poly(obj)
                poly_patch = plt.Polygon(poly, linestyle='--', edgecolor=color_pool[key], facecolor="white", lw=lw, zorder=2)
                ax.add_artist(poly_patch)
            if show_text:
                plt.text(obj[0], obj[1], "G" + str(key))

    ## Start Arrangements ##
    if show_arrangements in ['all', 'start']:
        for key, obj in start_arr.items():
            if obj[3] == 'cuboid':
                poly = poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
                poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=lw, zorder=3)
                ax.add_artist(poly_patch)
            elif obj[3] =='disc':
                ellipse = Ellipse((obj[0], obj[1]), obj[4], obj[5], obj[2]*180/math.pi, 
                    edgecolor='black', facecolor=color_pool[key], lw=lw, zorder=3)
                ax.add_patch(ellipse)
            else:
                poly = get_poly(obj)
                poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=lw, zorder=3)
                ax.add_artist(poly_patch)
            if show_text:    
                plt.text(obj[0], obj[1], "S" + str(key), zorder=4)


    if save_file != '':
        folder = os.path.dirname(save_file)
        if not os.path.exists(folder):
            os.makedirs(folder)
        plt.savefig(save_file)
    plt.axis('equal')
    plt.show()
    
    
def show_grid(start_arr,cell_obj_table, resolution=100, WIDTH=1000,HEIGHT=1000,show_text=True, save_file=''):
    '''
    show_arrangements='all', 'start', 'goal', 
    show_text=True/False, 
    save_file='' or 'path/to/file'
    '''

    color_pool = getColorMap(start_arr.keys())

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0,0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0,0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1) ]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')
    

    ## Start Arrangements ##
    for key, obj in start_arr.items():
        if obj[3] == 'cuboid':
            poly = poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
            poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=3)
            ax.add_artist(poly_patch)
        elif obj[3] =='disc':
            ellipse = Ellipse((obj[0], obj[1]), obj[4], obj[5], obj[2]*180/math.pi, 
                edgecolor='black', facecolor=color_pool[key], lw=2)
            ax.add_patch(ellipse)
        else:
            poly = get_poly(obj)
            poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=2)
            ax.add_artist(poly_patch)
        if show_text:    
            plt.text(obj[0], obj[1], "S" + str(key), zorder=4)

    max_x = int(WIDTH//resolution)
    max_y = int(HEIGHT//resolution)
    if HEIGHT%resolution == 0:
        max_y -= 1
    if WIDTH%resolution == 0:
        max_x -= 1
    for y in range(max_y,-1,-1):
        print_list = []
        for x in range(max_x+1):
            print_list.append(cell_obj_table[(x,y)])
        print(print_list)
    for cell_x in range(1,max_x+1):
        plt.plot([cell_x*resolution,cell_x*resolution], [0,HEIGHT])
    for cell_y in range(1,max_y+1):
        plt.plot([0,WIDTH], [cell_y*resolution,cell_y*resolution])

    if save_file != '':
        folder = os.path.dirname(save_file)
        if not os.path.exists(folder):
            os.makedirs(folder)
        plt.savefig(save_file)
    plt.show()
    

def show_bounding_volume(start_arr, obj_bv_dict, WIDTH=1000,HEIGHT=1000,show_text=True, save_file=''):
    '''
    show_arrangements='all', 'start', 'goal', 
    show_text=True/False, 
    save_file='' or 'path/to/file'
    '''

    color_pool = getColorMap(start_arr.keys())

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0,0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0,0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1) ]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')
    

    ## Start Arrangements ##
    for key, obj in start_arr.items():
        if obj[3] == 'cuboid':
            poly = poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
            poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=3)
            ax.add_artist(poly_patch)
        elif obj[3] =='disc':
            ellipse = Ellipse((obj[0], obj[1]), obj[4], obj[5], obj[2]*180/math.pi, 
                edgecolor='black', facecolor=color_pool[key], lw=2)
            ax.add_patch(ellipse)
        else:
            poly = get_poly(obj)
            poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=2)
            ax.add_artist(poly_patch)
        # print bounding dict
        x,y,r = obj_bv_dict[key]
        disc = Circle((x,y),r,edgecolor='black', fc="None", lw=2, zorder=3)
        ax.add_patch(disc)
        if show_text:    
            plt.text(obj[0], obj[1], "S" + str(key), zorder=4)


    if save_file != '':
        folder = os.path.dirname(save_file)
        if not os.path.exists(folder):
            os.makedirs(folder)
        plt.savefig(save_file)
    plt.show()
    
    
def show_poly_dict(start_arr, obj_poly_dict, WIDTH=1000,HEIGHT=1000,show_text=True, save_file=''):
    '''
    show_arrangements='all', 'start', 'goal', 
    show_text=True/False, 
    save_file='' or 'path/to/file'
    '''

    color_pool = getColorMap(start_arr.keys())

    fig = plt.figure(num=None, figsize=(int(10 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    axes = fig.subplots(1,2)

    wall = [(0,0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0,0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1) ]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        axes[0].plot(wallx, wally, 'blue')
        axes[1].plot(wallx, wally, 'blue')
    

    ## Start Arrangements ##
    for key, obj in start_arr.items():
        if obj[3] == 'cuboid':
            poly = poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
            poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=3)
            axes[0].add_artist(poly_patch)
        elif obj[3] =='disc':
            ellipse = Ellipse((obj[0], obj[1]), obj[4], obj[5], obj[2]*180/math.pi, 
                edgecolor='black', facecolor=color_pool[key], lw=2)
            axes[0].add_patch(ellipse)
        else:
            poly = get_poly(obj)
            poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=2)
            axes[0].add_artist(poly_patch)
        # print bounding dict
        poly = obj_poly_dict[key]
        poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=3)
        axes[1].add_artist(poly_patch)
        if show_text:    
            axes[0].text(obj[0], obj[1], "S" + str(key), zorder=4)
            axes[1].text(obj[0], obj[1], "S" + str(key), zorder=4)


    if save_file != '':
        folder = os.path.dirname(save_file)
        if not os.path.exists(folder):
            os.makedirs(folder)
        plt.savefig(save_file)
    plt.show()
    

def show_quadtree(start_arr,quadtree:QuadTree, WIDTH=1000,HEIGHT=1000,show_text=True, save_file=''):
    '''
    show_arrangements='all', 'start', 'goal', 
    show_text=True/False, 
    save_file='' or 'path/to/file'
    '''

    color_pool = getColorMap(start_arr.keys())

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0,0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0,0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1) ]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')
    

    ## Start Arrangements ##
    for key, obj in start_arr.items():
        if obj[3] == 'cuboid':
            poly = poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
            poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=3)
            ax.add_artist(poly_patch)
        elif obj[3] =='disc':
            ellipse = Ellipse((obj[0], obj[1]), obj[4], obj[5], obj[2]*180/math.pi, 
                edgecolor='black', facecolor=color_pool[key], lw=2)
            ax.add_patch(ellipse)
        else:
            poly = get_poly(obj)
            poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=2)
            ax.add_artist(poly_patch)
        if show_text:    
            plt.text(obj[0], obj[1], "S" + str(key), zorder=4)

    grid_zorder=10

    # draw quadtree
    def draw_node(ax,tree:QuadTree):
        if tree != None:
            try:
                ax.plot([tree.l,tree.r],[tree.t,tree.t],zorder=grid_zorder)
            except:
                print(tree)
            try:
                ax.plot([tree.r,tree.r],[tree.t,tree.b],zorder=grid_zorder)
            except:
                print(tree)
            try:
                ax.plot([tree.r,tree.l],[tree.b,tree.b],zorder=grid_zorder)
            except:
                print(tree)
            try:
                ax.plot([tree.l,tree.l],[tree.b,tree.t],zorder=grid_zorder)
            except:
                print(tree)
            draw_node(ax,tree.nw)
            draw_node(ax,tree.ne)
            draw_node(ax,tree.se)
            draw_node(ax,tree.sw)
    
    draw_node(ax,quadtree)

    if save_file != '':
        folder = os.path.dirname(save_file)
        if not os.path.exists(folder):
            os.makedirs(folder)
        plt.savefig(save_file)
    plt.show()


    
def test_buffer_allocation(start_arr, objID, new_pose, quadtree, WIDTH=1000,HEIGHT=1000,show_text=True, draw_quadtree=True, save_file=''):
    '''
    show_arrangements='all', 'start', 'goal', 
    show_text=True/False, 
    save_file='' or 'path/to/file'
    '''

    color_pool = getColorMap(start_arr.keys())

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0,0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0,0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1) ]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')
    

    ## Start Arrangements ##
    for key, obj in start_arr.items():
        if key == objID:
            obj = new_pose
            if obj[3] == 'cuboid':
                poly = poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
                poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=3)
                ax.add_artist(poly_patch)
            elif obj[3] =='disc':
                ellipse = Ellipse((obj[0], obj[1]), obj[4], obj[5], obj[2]*180/math.pi, 
                    edgecolor='black', facecolor=color_pool[key], lw=2)
                ax.add_patch(ellipse)
            else:
                poly = get_poly(obj)
                poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=2)
                ax.add_artist(poly_patch)
            if show_text:    
                plt.text(obj[0], obj[1], "B" + str(key), zorder=4)
        else:
            if obj[3] == 'cuboid':
                poly = poly_stick((obj[0], obj[1]), obj[2], obj[4], obj[5])
                poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=3)
                ax.add_artist(poly_patch)
            elif obj[3] =='disc':
                ellipse = Ellipse((obj[0], obj[1]), obj[4], obj[5], obj[2]*180/math.pi, 
                    edgecolor='black', facecolor=color_pool[key], lw=2)
                ax.add_patch(ellipse)
            else:
                poly = get_poly(obj)
                poly_patch = plt.Polygon(poly, edgecolor='black', facecolor=color_pool[key], lw=2, zorder=2)
                ax.add_artist(poly_patch)
            if show_text:    
                plt.text(obj[0], obj[1], "S" + str(key), zorder=4)

    
    grid_zorder=10

    # draw quadtree
    def draw_node(ax,tree:QuadTree):
        if tree != None:
            try:
                ax.plot([tree.l,tree.r],[tree.t,tree.t],zorder=grid_zorder)
            except:
                print(tree)
            try:
                ax.plot([tree.r,tree.r],[tree.t,tree.b],zorder=grid_zorder)
            except:
                print(tree)
            try:
                ax.plot([tree.r,tree.l],[tree.b,tree.b],zorder=grid_zorder)
            except:
                print(tree)
            try:
                ax.plot([tree.l,tree.l],[tree.b,tree.t],zorder=grid_zorder)
            except:
                print(tree)
            draw_node(ax,tree.nw)
            draw_node(ax,tree.ne)
            draw_node(ax,tree.se)
            draw_node(ax,tree.sw)
    
    if draw_quadtree:
        draw_node(ax,quadtree)


    if save_file != '':
        folder = os.path.dirname(save_file)
        if not os.path.exists(folder):
            os.makedirs(folder)
        plt.savefig(save_file)
    plt.show()

