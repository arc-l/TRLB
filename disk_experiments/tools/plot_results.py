import matplotlib
from matplotlib import pyplot as plt 
import os
import copy
import numpy as np


# File = os.path.join(os.path.abspath(os.path.dirname(__file__)),
#     "logs",
#     "fmRS",
#     "2021-04-20-00-58-23.txt"
# )


Compared_Methods = [
    # "BST_stick",
    # "BST_SP_RB", 
    # "BST_SP_RB_PP",
    # "BST_SP_RB_MPP", 
    # "ST_SP_RB", 
    # "BST_SP_RB_random_buffer_initiation", 
    # "BST_SP_RB_07182134_NoRepeat",
    # "BST_SP_RB_07181515",
    # "BST_SP_RB_Rb", 
    # "MCTS_external"
    "MCTS"
    ]


Results = {method:{} for method in Compared_Methods}

for method in Compared_Methods:
    File = os.path.join(os.path.abspath(os.path.dirname(os.path.dirname(__file__))),
        "logs",
        method,
        # "fmRS",
        # "MCTS (M=1M)",
        # "MCTS(M=100k)",
        # "MCTS",
        # "OS_SP_RB",
        # "OS_SP_RB_Rb",
        # "ST_SP_RB",
        # "BST_SP_RB",
        # "BST_SP_Rd",
        # "BST_OPT_RB",
        # "ST_SP_RB_Rb",
        # "BST_SP_RB_Rb",
        "collection.txt"
        # "2021-06-16-21-14-08.txt"
    )
    with open(File, 'rb') as f:
        for line in f.readlines():
            words = line.split()
            Density = float(words[0])
            numObjs = int(words[1])
            Time = float(words[3])
            ratio = float(words[4])
            if ratio >0.1:
                rate = 1.0
            else:
                rate = 0.0
            num_collision = 0
            
            if Density not in Results[method]:
                Results[method][Density] = {}
            if numObjs not in Results[method][Density]:
                Results[method][Density][numObjs] = {}
            if "Rate" not in Results[method][Density][numObjs]:
                Results[method][Density][numObjs]["Rate"] = []
            if "Time" not in Results[method][Density][numObjs]:
                Results[method][Density][numObjs]["Time"] = []
            if "Ratio" not in Results[method][Density][numObjs]:
                Results[method][Density][numObjs]["Ratio"] = []
            if "Coll" not in Results[method][Density][numObjs]:
                Results[method][Density][numObjs]["Coll"] = []
            
            Results[method][Density][numObjs]["Rate"].append(rate)
            Results[method][Density][numObjs]["Time"].append(Time)
            Results[method][Density][numObjs]["Ratio"].append(ratio)
            Results[method][Density][numObjs]["Coll"].append(num_collision)


# Selected_Densities = Results[Compared_Methods[0]].keys()
Selected_Densities = [0.5]
Selected_NumObjs = [5, 40]
# Selected_NumObjs = range(10, 41, 10)


Joint_Results = {method:{} for method in Compared_Methods}
for Density in Selected_Densities:
    # for numObjs in Results[Compared_Methods[0]][Density].keys():
    for numObjs in Selected_NumObjs:
        Joint = True
        for method in Compared_Methods:
            if (Density not in Results[method]) or (numObjs not in Results[method][Density]):
                Joint = False
                break
        if not Joint:
            continue
        # joint_instances = set(range(30))
        joint_instances = set([ i for i, r in enumerate(Results[Compared_Methods[0]][Density][numObjs]['Rate']) if r > 0.5])
        for method in Compared_Methods:
            joint_instances = joint_instances.intersection(set([ i for i, r in enumerate(Results[method][Density][numObjs]['Rate']) if r > 0.5]))
        for method in Compared_Methods:
            if Density not in Joint_Results[method]:
                Joint_Results[method][Density] = {}
            if numObjs not in Joint_Results[method][Density]:
                Joint_Results[method][Density][numObjs] = {}
            if len(joint_instances) >0:
                Joint_Results[method][Density][numObjs]['Rate'] = np.average(Results[method][Density][numObjs]['Rate'])
                Joint_Results[method][Density][numObjs]['Time'] = np.average([ Results[method][Density][numObjs]['Time'][i] for i in joint_instances])
                Joint_Results[method][Density][numObjs]['Ratio'] = np.average([ Results[method][Density][numObjs]['Ratio'][i] for i in joint_instances])
                Joint_Results[method][Density][numObjs]['Coll'] = np.average([ Results[method][Density][numObjs]['Coll'][i] for i in joint_instances])
            else:
                Joint_Results[method][Density][numObjs]['Rate'] = np.average(Results[method][Density][numObjs]['Rate'])
                Joint_Results[method][Density][numObjs]['Time'] = -1
                Joint_Results[method][Density][numObjs]['Ratio'] = -1
                Joint_Results[method][Density][numObjs]['Coll'] = -1

for method in Joint_Results.keys():
    print("#################")
    print(method)
    for Density in Joint_Results[method].keys():
        print( "D=", Density)
        for numObjs in sorted(Joint_Results[method][Density].keys()):
            print( numObjs,  '\t',
            format(Joint_Results[method][Density][numObjs]['Time'], '.2e'), '\t',
            format(Joint_Results[method][Density][numObjs]['Ratio'], '.2e'),  '\t',
            format(float(Joint_Results[method][Density][numObjs]['Rate']), '.2e'), '\t',
            format(Joint_Results[method][Density][numObjs]['Coll'], '.2e'), '\t'
            )


# plotting
Plotting_Index = 'Time'

