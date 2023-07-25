'''
generate json files for tetris blocks
'''

import cv2
import matplotlib.pyplot as plt
import numpy as np
import os
import ujson

curr_dir = os.path.dirname(os.path.abspath(__file__))

tetris_I = [
    [0,0],[0,1],[4,1],[4,0]
]

tetris_J = [
    [0,0],[0,2],[1,2],[1,1],[3,1],[3,0],
]

tetris_L = [
    [0,0],[0,1],[2,1],[2,2],[3,2],[3,0]
]

tetris_O = [
    [0,0],[0,2],[2,2],[2,0]
]

tetris_S = [
    [0,0],[0,1],[1,1],[1,2],[3,2],[3,1],[2,1],[2,0]
]

tetris_T = [
    [0,0],[0,1],[1,1],[1,2],[2,2],[2,1],[3,1],[3,0]
]

tetris_Z = [
    [0,0],[0,1],[-1,1],[-1,2],[1,2],[1,1],[2,1],[2,0]
]

tetris_blocks = {
    'tetris_I':tetris_I, 
    'tetris_J':tetris_J, 
    'tetris_L':tetris_L, 
    'tetris_O':tetris_O, 
    'tetris_S':tetris_S, 
    'tetris_T':tetris_T, 
    'tetris_Z':tetris_Z
}

def plot_model(poly):
    for i in range(-1,len(poly)-1):
        x1,y1 = poly[i]
        x2,y2 = poly[i+1]
        plt.plot([x1,x2],[y1,y2],color='r')
    plt.axis('equal')
    plt.show()


def main():
    for poly_name, poly in tetris_blocks.items():
        poly_arr = np.array(poly)
        # plot_model(tetris_Z)
        center, radius = cv2.minEnclosingCircle(poly_arr)
        center_arr = np.tile(np.array(center), (len(poly),1))
        new_poly_arr = poly_arr-center_arr
        print(center,radius)
        new_poly_list = [list(e) for e in new_poly_arr ]
        print(new_poly_list)
        data_dict = {
            'center' : [0,0],
            'radius' : radius,
            'points' : new_poly_list
        }
        save_file_name = os.path.join(
            curr_dir, poly_name+'.json'
        )
        with open(save_file_name, 'w') as f:
            ujson.dump(data_dict,f)
        


if __name__ == '__main__':
    main()


