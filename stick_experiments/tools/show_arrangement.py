import math
import numpy as np
# matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.ticker import LinearLocator, FormatStrFormatter
import matplotlib.colors as colors
import matplotlib.cm as cmx
import os
import sys

sys.path.append(os.path.dirname(os.path.dirname(__file__)))


from shapely.geometry import polygon

from tools.util import poly_stick

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


def show_arrangement(numObjs, Density, start_arr, goal_arr):
    WIDTH = 1000
    HEIGHT = 1000

    Radius = math.sqrt(Density*HEIGHT*WIDTH/numObjs/math.pi)

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0, 0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1)]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')

    for key, point in goal_arr.items():
        circle = plt.Circle(point,Radius, color="blue")
        ax.add_artist(circle)
        plt.text(point[0], point[1], "G" + str(key))

    for key, point in start_arr.items():
        circle = plt.Circle(point,Radius, color="red")
        ax.add_artist(circle)
        plt.text(point[0], point[1], "S" + str(key))
    plt.show()

def show_stick_arrangement(numObjs, Density, start_arr, goal_arr, WL_ratio=0.3):
    WIDTH = 1000
    HEIGHT = 1000
    
    Length = math.sqrt( Density * WIDTH * HEIGHT/numObjs/WL_ratio)

    color_pool = getColorMap(start_arr.keys())

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0, 0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1)]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')

    for key, point in goal_arr.items():
        poly = poly_stick((point[0], point[1]), point[2], Length, WL_ratio*Length)
        poly_patch = plt.Polygon(poly, linestyle='--', edgecolor=color_pool[key], facecolor="white", lw=2,zorder=2)
        ax.add_artist(poly_patch)
        plt.text(point[0], point[1], "G" + str(key))

    for key, point in start_arr.items():
        poly = poly_stick((point[0], point[1]), point[2], Length, WL_ratio*Length)
        poly_patch = plt.Polygon(poly, facecolor=color_pool[key], lw=2, edgecolor='black',zorder=3)
        ax.add_artist(poly_patch)
        plt.text(point[0], point[1], "S" + str(key), zorder=4)
    plt.show()
    
def show_pose(real_poly, Length, WL_ratio, start_arr, goal_arr, real_buffer):
    WIDTH = 1000
    HEIGHT = 1000
    
    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0, 0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1)]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')
    
    for key, point in goal_arr.items():
        poly = poly_stick((point[0], point[1]), point[2], Length, WL_ratio*Length)
        poly_patch = plt.Polygon(poly, color="blue")
        ax.add_artist(poly_patch)
        plt.text(point[0], point[1], "G" + str(key))

    for key, point in start_arr.items():
        poly = poly_stick((point[0], point[1]), point[2], Length, WL_ratio*Length)
        poly_patch = plt.Polygon(poly, color="red")
        ax.add_artist(poly_patch)
        plt.text(point[0], point[1], "S" + str(key))

    poly_patch = plt.Polygon(real_poly, color="green")
    ax.add_artist(poly_patch)
    # poly_patch = plt.Polygon(real_buffer, color="yellow")
    # ax.add_artist(poly_patch)

    plt.show()


def show_buffer(buffer, Length, WL_ratio, obstacle_list):
    WIDTH = 1000
    HEIGHT = 1000
    
    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0, 0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1)]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')
    
    for obs in obstacle_list:
        poly_patch = plt.Polygon(poly_stick(obs[:2], obs[2], Length, Length*WL_ratio), color="blue")
        ax.add_artist(poly_patch)

    poly_patch = plt.Polygon(poly_stick(buffer[:2], buffer[2], Length, Length*WL_ratio), color="red")
    ax.add_artist(poly_patch)

    plt.show()


def show_separate_stick_arrangement(numObjs, Density, start_arr, WL_ratio = 0.3):
    WIDTH = 1000
    HEIGHT = 1000
    text_offset_x = 30
    text_offset_y = text_offset_x
    
    Length = math.sqrt( Density * WIDTH * HEIGHT/numObjs/WL_ratio)

    color_pool = getColorMap(start_arr.keys())

    fig = plt.figure(num=None, figsize=(int(5 * WIDTH / HEIGHT), 5), dpi=120, facecolor='w', edgecolor='k')
    ax = fig.subplots()

    wall = [(0, 0), (WIDTH, 0), (WIDTH, HEIGHT), (0, HEIGHT), (0, 0)]
    for walls in [ [wall[ii], wall[ii+1]] for ii in range(len(wall)-1)]:
        wallx = [p[0] for p in walls]
        wally = [p[1] for p in walls]
        plt.plot(wallx, wally, 'blue')


    for key, point in start_arr.items():
        poly = poly_stick((point[0], point[1]), point[2], Length, WL_ratio*Length)
        poly_patch = plt.Polygon(poly, facecolor=color_pool[key], lw=2, edgecolor='black',zorder=3)
        ax.add_artist(poly_patch)
        plt.text(point[0]-text_offset_x, point[1]-text_offset_y, str(key), zorder=5, fontsize = 'x-large')
    plt.show()

