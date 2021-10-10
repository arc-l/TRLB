import datetime
import os
import sys
import timeout_decorator
import math
import numpy as np
import time

sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from primitive_plan.Labeled_DFS_DP import DFS_DP_Search
from primitive_plan.FVS_DP_Solver import FVS_Plan_Search
from primitive_plan.random_order import random_order
from util import generate_instance

TimeLimit = 300

def batch_experiments():
    Height = 1000
    Width = 1000
    num_trials = 30

    # generate an unique experiment ID
    timestamp = time.time()
    timestamp_value = datetime.datetime.fromtimestamp(timestamp)
    experimentID = timestamp_value.strftime("%Y-%m-%d-%H-%M-%S")

    # # restore arrangementIDs
    # arrangementID_bank = restore_arrangementIDs()

    for Density in [0.5]:
        for numObjs in [10,20]:
            for trial in range(num_trials):
            # for trial in [9]:
                print "D, n, trl: ", Density, numObjs, trial
                
                instance_num = 20
                arrangementIDs = np.random.choice(range(instance_num),size=2, replace=False)

                # arrangementIDs = arrangementID_bank[Density][numObjs][trial]
                
                radius = math.sqrt(Density*Height*Width/math.pi/numObjs)
    
                # Load arrangement
                start_arr, goal_arr = generate_instance(numObjs, Density, arrangementIDs)

                DG = construct_DG(start_arr, goal_arr, radius)

                try:
                    MRB_RB, MRB_TB = MRB_analysis(DG)
                except Exception:
                    MRB_RB = -1 
                    MRB_TB = -1
                try:
                    FVS_RB, FVS_TB = FVS_analysis(DG)
                except Exception:
                    FVS_RB = -1
                    FVS_TB = -1
                try:
                    rand_RB, rand_TB = rand_analysis(DG)
                except Exception:
                    rand_RB = -1
                    rand_TB = -1

                print MRB_RB, FVS_RB, rand_RB
                print MRB_TB, FVS_TB, rand_TB

                labeltxt = 'RB_TB_Comparison'

                folder = os.path.join(
                    os.path.abspath(os.path.dirname(__file__)),
                    "logs",
                    labeltxt
                )
                if not os.path.exists(folder):
                    os.makedirs(folder)
                
                log = os.path.join(folder, experimentID + '.txt')

                with open(log, 'ab') as f:
                    f.write(str(Density) + " " + str(numObjs) + " " + str(trial) + " " 
                    + str(MRB_RB) + " " + str(FVS_RB) + " " + str(rand_RB) + " " 
                    + str(MRB_TB) + " " + str(FVS_TB) + " " + str(rand_TB) + " "
                    + str(arrangementIDs) + "\n")

def test():
    # Environment Settings
    numObjs = 10
    Density = 0.5
    Height = 1000
    Width = 1000
    # arrangementIDs = [0, 72] # two stage n=5, D=0.3
    # arrangementIDs = [3, 1]
    arrangementIDs = np.random.choice(range(20),size=2, replace=False)
    radius = math.sqrt(Density*Height*Width/math.pi/numObjs)
    
    # Load arrangement
    start_arr, goal_arr = generate_instance(numObjs, Density, arrangementIDs)

    DG = construct_DG(start_arr, goal_arr, radius)

    RB, TB = MRB_analysis(DG)
    print RB,TB

    RB, TB = FVS_analysis(DG)
    print RB,TB

    RB, TB = rand_analysis(DG)
    print RB,TB

@timeout_decorator.timeout(TimeLimit)
def MRB_analysis(DG):
    RB, action_list = DFS_DP_Search(DG)
    TB = 0
    for action in action_list:
        if action[1] == 'b':
            TB+=1
    return RB, TB

@timeout_decorator.timeout(TimeLimit)
def FVS_analysis(DG):
    action_list = FVS_Plan_Search(DG)
    RB = 0
    TB = 0
    current_buffers = set()
    for action in action_list:
        obj = action[0]
        if action[1] == 'b':
            TB+=1
            current_buffers.add(obj)
        elif obj in current_buffers:
            current_buffers.remove(obj)
        RB = max(RB, len(current_buffers))
    return RB, TB

@timeout_decorator.timeout(TimeLimit)
def rand_analysis(DG):
    action_list = random_order(DG)
    RB = 0
    TB = 0
    current_buffers = set()
    for action in action_list:
        obj = action[0]
        if action[1] == 'b':
            TB+=1
            current_buffers.add(obj)
        elif obj in current_buffers:
            current_buffers.remove(obj)
        RB = max(RB, len(current_buffers))
    return RB, TB


def construct_DG( start_arr, goal_arr, radius):
    DG = {}
    for goal_obj, goal_center in goal_arr.items():
        DG[goal_obj] = set()
        for start_obj, start_center in start_arr.items():
            if start_obj == goal_obj:
                continue
            if (math.sqrt((start_center[0]-goal_center[0])**2+
            (start_center[1]-goal_center[1])**2) <= 2*radius):
                DG[goal_obj].add(start_obj)
    return DG


def plot_results():
    File = os.path.join(os.path.abspath(os.path.dirname(__file__)),
        "logs",
        "RB_TB_Comparison",
        "2021-05-20-01-22-19.txt"
    )

    Results = {}

    with open(File, 'rb') as f:
        for line in f.readlines():
            words = line.split()
            Density = float(words[0])
            numObjs = int(words[1])
            MRB_RB = int(words[3])
            FVS_RB = int(words[4])
            rand_RB = int(words[5])
            MRB_TB = int(words[6])
            FVS_TB = int(words[7])
            rand_TB = int(words[8])
            if Density not in Results:
                Results[Density] = {}
            if numObjs not in Results[Density]:
                Results[Density][numObjs] = {}
            if "MRB" not in Results[Density][numObjs]:
                Results[Density][numObjs]["MRB"] = {
                    'RB':[],'TB':[]
                }
            if "FVS" not in Results[Density][numObjs]:
                Results[Density][numObjs]["FVS"] = {
                    'RB':[],'TB':[]
                }
            if "rand" not in Results[Density][numObjs]:
                Results[Density][numObjs]["rand"] = {
                    'RB':[],'TB':[]
                }
            if (
                (MRB_TB>-0.5) and
                (FVS_TB>-0.5) and
                (rand_TB>-0.5)
                ):
                Results[Density][numObjs]["MRB"]['RB'].append(MRB_RB)
                Results[Density][numObjs]["MRB"]['TB'].append(MRB_TB)
                Results[Density][numObjs]["FVS"]['RB'].append(FVS_RB)
                Results[Density][numObjs]["FVS"]['TB'].append(FVS_TB)
                Results[Density][numObjs]["rand"]['RB'].append(rand_RB)
                Results[Density][numObjs]["rand"]['TB'].append(rand_TB)
    
    for Density in Results:
        for numObjs in Results[Density]:
            for method in Results[Density][numObjs].keys():
                for key in Results[Density][numObjs][method]:
                    Results[Density][numObjs][method][key] = np.average(Results[Density][numObjs][method][key])

    # print Results

    for Density in Results.keys():
        print "D=", Density
        for numObjs in sorted(Results[Density].keys()):
            print numObjs,\
                Results[Density][numObjs]["MRB"]['RB'],\
                Results[Density][numObjs]["FVS"]['RB'],\
                Results[Density][numObjs]["rand"]['RB'],\
                Results[Density][numObjs]["MRB"]['TB'],\
                Results[Density][numObjs]["FVS"]['TB'],\
                Results[Density][numObjs]["rand"]['TB']

if __name__ == "__main__":
    plot_results()
    # batch_experiments()
    # test()