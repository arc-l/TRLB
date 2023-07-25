import os
import math
from shapely.geometry import Polygon
import numpy as np
import matplotlib.pyplot as plt

from tools.util import poly_stick, polysCollide


curr_dir = os.path.dirname(os.path.abspath(__file__))


workspace_poly =poly_stick([500,500],0,1000,1000)

def run_experiments():
    # evaluate_result()
    # exit()
    num_checks = 100000
    # num_checks = 1
    areas = [1e4*(2**e) for e in range(-6, 4, 1)]
    ab_ratios = [e/10 for e in range(1,11)]
    
    for ab_ratio in ab_ratios:
        for area in areas:
            counter = 0
            num_collisions = 0
            length = math.sqrt(area/ab_ratio)
            width = length * ab_ratio
            for _ in range(num_checks):
                obj_poly = random_rectangle(
                    (length, width)
                )
                obs_poly = random_rectangle(
                    (100,100)
                )
                if polysCollide(obj_poly, obs_poly):
                    num_collisions += 1
            result = [area,ab_ratio,num_collisions/num_checks]
            # save_result(result)
            print(result)
            plot_polys([workspace_poly, obj_poly, obs_poly])
            
def save_result(result_list):
    '''
    save
    '''
    with open(
        os.path.join(curr_dir, 'results.txt'), 'a'
    ) as f:
        line = ''
        for e in result_list:
            line += str(e) + ' '
        line += '\n'
        f.write(line)
        

def evaluate_result():
    '''
    load and evaluate results
    '''    
    data = {}
    with open(
        os.path.join(curr_dir, 'results.txt'), 'r'
    ) as f:
        for line in f.readlines():
            words = line.split()
            area, ratio, prob = float(words[0]), float(words[1]), float(words[2])
            data[(area, ratio)] = prob
    
    # area evaluation    
    areas = [156.25, 312.5, 625.0, 1250.0, 2500.0, 5000.0, 10000.0, 20000.0, 40000.0, 80000.0]
    # my_x = [math.log(area, 2) for area in areas]
    my_x = areas
    # ratios = [e/10.0 for e in range(1,11)]
    ratios = [0.1,0.2,1.0]
    for ratio in ratios:
        probs = [data[(area, ratio)] for area in areas]
        plt.plot(my_x, probs,label='ratio='+str(ratio))
        # for i in range(len(areas)-1):
        #     plt.plot([areas[i], areas[i+1]], [probs[i], probs[i]*2], label='ideal')
    # probs = [(area+1e4+math.sqrt(area/math.pi)*400)/(1000-2*math.sqrt(area/math.pi))**2 for area in areas]
    probs = [(area+1e4+math.sqrt(10000/math.pi)*math.sqrt(area/0.1)*2*(1+0.1))/(1000-2*math.sqrt(10000/math.pi))**2 for area in areas]
    # probs = [(area+1e4+math.sqrt(area/math.pi)*400)/(1000)**2 for area in areas]
    plt.plot(my_x, probs,label='ideal')
    # probs = [(area+1e4+math.sqrt(10000/math.pi)*math.sqrt(area)*4)/(1000)**2 for area in areas]
    # plt.plot(my_x, probs,label='combine2')
    # probs = [(area)/(1000)**2 for area in areas]
    # plt.plot(my_x, probs,label='linear')
    # probs = [(math.sqrt(area/math.pi)*400)/(1000)**2 for area in areas]
    # plt.plot(my_x, probs,label='sqrt')
    # plt.xticks(my_x,labels=areas)
    # probs = [math.log(area,1.414)/100 for area in areas]
    # plt.plot(areas, probs,label='ideal')
    # plt.xscale('log')
    plt.legend()
    plt.show()
    
    # # ratio evaluation
    # areas = [156.25, 312.5, 625.0, 1250.0, 2500.0, 5000.0, 10000.0, 20000.0, 40000.0, 80000.0]
    # ratios = [e/10.0 for e in range(1,11)]
    # for area in areas:
    #     plt.plot(ratios, [data[(area,ratio)] for ratio in ratios], label='area='+str(area))
    # # plt.yscale('log')
    # plt.legend()
    # plt.show()
        

    
def random_rectangle(obj_size:tuple):
    '''
    randomly generate rectangle polygon in the workspace
    '''
    x,y = np.random.uniform(low=0,high=1000, size=2)
    theta = np.random.uniform(low=0,high=math.pi)
    poly = np.array(poly_stick([x,y], theta, obj_size[0],obj_size[1]))
    while not Polygon(workspace_poly).contains(Polygon(poly)):
        x,y = np.random.uniform(low=0,high=1000, size=2)
        theta = np.random.uniform(low=0,high=math.pi)
        poly = np.array(poly_stick([x,y], theta, obj_size[0],obj_size[1]))
    return poly
    
def plot_polys(polys):
    '''
    plot polys
    '''
    for poly in polys:
        poly = np.array(poly)
        X = list(poly[:,0])+[poly[0,0]]
        Y = list(poly[:,1])+[poly[0,1]]
        plt.plot(X,Y)
    plt.axis('equal')
    plt.show()