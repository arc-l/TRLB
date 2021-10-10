# Import deepcopy for copying data structures
from copy import deepcopy
import numpy as np
import matplotlib.pyplot as plt
import os

def fig_buffer_05():
    Compared_Methods = [
        "OS_SP_RB", 
        # "OS_SP_RB_Rb", 
        "OS_OPT_RB", 
        # "MCTS"
        ]

    Selected_Densities = [0.5]
    Selected_NumObjs = [10, 20, 30, 40]

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
                index = 7
                try:
                    num_collision = float(words[index])
                except Exception:
                    NotNum = True
                else:
                    NotNum = False
                while NotNum:
                    index += 1
                    try:
                        num_collision = float(words[index])
                    except Exception:
                        NotNum = True
                    else:
                        NotNum = False
                
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

    Avg_Results = deepcopy(Results)
    for method in Compared_Methods:
        for Density in Results[method].keys():
            for numObjs in Results[method][Density].keys():
                Avg_Results[method][Density][numObjs]["Rate"] = np.average(Avg_Results[method][Density][numObjs]["Rate"])
                Avg_Results[method][Density][numObjs]["Time"] = np.average([Avg_Results[method][Density][numObjs]["Time"][i] for i in range(30) if Results[method][Density][numObjs]["Rate"][i]>0.5])
                Avg_Results[method][Density][numObjs]["Ratio"] = np.average([Avg_Results[method][Density][numObjs]["Ratio"][i] for i in range(30) if Results[method][Density][numObjs]["Rate"][i]>0.5])
                Avg_Results[method][Density][numObjs]["Coll"] = np.average([Avg_Results[method][Density][numObjs]["Coll"][i] for i in range(30) if Results[method][Density][numObjs]["Rate"][i]>0.5])
                if Avg_Results[method][Density][numObjs]["Rate"] == np.average(Avg_Results[method][Density][numObjs]["Rate"]) <0.01:
                    Avg_Results[method][Density][numObjs]["Time"] = 1e-6
                    Avg_Results[method][Density][numObjs]["Ratio"] = 0
                    Avg_Results[method][Density][numObjs]["Coll"] = 0

    Joint_Results = {method:{} for method in Compared_Methods}
    for Density in Selected_Densities:
        for numObjs in Results[Compared_Methods[0]][Density].keys():
            Joint = True
            for method in Compared_Methods:
                if (Density not in Results[method]) or (numObjs not in Results[method][Density]):
                    Joint = False
                    break
            if not Joint:
                continue
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

    my_path = os.path.abspath(os.path.dirname(__file__))
    '''
    Style elements
        Here we can setup a drawing style for each algorithm.
        This helps to make the drawing style of an algorithm consistent.
    '''
    defaultStyle = {
        'label' : 'default',    # The name of the algorithm
        'ls' : '-',             # Line style, '-' means a solid line
        'linewidth' : 2,        # Line width
        "edgecolor" : 'black',
        'color' : 'k',          # Line color, 'k' means color
        'zorder' : 10,         # The 'height' of the plot. 
                                # Affects whether items are in the front of / behind each other.
        # You can add more style items here, e.g., markers.
    }
    # Here, we setup the style for an algorithm. Let's call it Alg.1.
    alg1Style = deepcopy(defaultStyle)          # First copy all default styles.
    alg1Style['label'] = r'\textsc{'+ Compared_Methods[0] +'}'    # Setup algorithm name. 
                                                # We use \textsc here which is a latex command. 
                                                # This is fine since we use latex to generate text.
    alg1Style['color'] = 'tab:green'           # Customized line color
                                                # https://matplotlib.org/3.1.0/gallery/color/named_colors.html
    
    if len(Compared_Methods)>=2:
        # Another algorithm
        alg2Style = deepcopy(defaultStyle)
        alg2Style['label'] = r'\textsc{'+ Compared_Methods[1] +'}'
        alg2Style['color'] = 'tab:orange'

    if len(Compared_Methods)>=3:
        # Another algorithm
        alg3Style = deepcopy(defaultStyle)
        alg3Style['label'] = r'\textsc{'+ Compared_Methods[2] +'}'
        alg3Style['color'] = 'tab:orange'

    if len(Compared_Methods)>=4:
        # Another algorithm
        alg4Style = deepcopy(defaultStyle)
        alg4Style['label'] = r'\textsc{'+ Compared_Methods[3] +'}'
        alg4Style['color'] = 'tab:green'


    ''' Some global variables '''
    FIGURE_SIZE = (20, 4)      # Figure width and height. 
                                    # This is a good value for 2-column paper.
    FONT_SIZE = 36                   # Size of text
    LEGEND_FONT_SIZE = 24            # We might need different font sizes for different text

    ''' 
    Parameters for legend to better utilize space.
    '''
    plt.rcParams["legend.labelspacing"] = 0.2
    plt.rcParams["legend.handlelength"] = 1.75
    plt.rcParams["legend.handletextpad"] = 0.5
    plt.rcParams["legend.columnspacing"] = 0.75

    # plt.rcParams.update({'figure.autolayout': True})

    '''
    Use latex to generate text
    Note that these params usually make the code slow. If you want to preview the figure without generating latex text, feel free to comment these. 
    '''
    plt.rcParams['ps.useafm'] = True
    plt.rcParams['pdf.use14corefonts'] = True
    plt.rcParams['text.usetex'] = True
    plt.rcParams['font.family'] = "serif"
    plt.rcParams['font.serif'] = "Times"

    ''' The real drawing part starts here. '''
    # Put your data over here.



    bar_width = 4
    # Start to create the figure
    fig = plt.figure(figsize = FIGURE_SIZE)

    ###### 1 #####
    ax = fig.add_subplot(1,3,1)

    ax.bar(np.array(Selected_NumObjs)-0.5*bar_width, [Avg_Results[Compared_Methods[0]][Selected_Densities[0]][numObjs]['Time'] for numObjs in Selected_NumObjs], bar_width, **alg1Style)
    ax.bar(np.array(Selected_NumObjs)+0.5*bar_width, [Avg_Results[Compared_Methods[1]][Selected_Densities[0]][numObjs]['Time'] for numObjs in Selected_NumObjs], bar_width, **alg2Style)
    # ax.bar(np.array(Selected_NumObjs)+1.0*bar_width, [Avg_Results[Compared_Methods[2]][Selected_Densities[0]][numObjs]['Time'] for numObjs in Selected_NumObjs], bar_width, **alg3Style)

    ax.set_xlim([Selected_NumObjs[0]-1.5 * bar_width, 
    Selected_NumObjs[-1]+1.5 * bar_width])
    ax.set_ylim([1e-4, 1e3])
    # Set x and y label. We use latex to generate text
    # ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 1)
    ax.set_xticks(Selected_NumObjs)
    # plt.xticks(x, mysticks)
    # ax.set_ylabel('MRB',fontsize = FONT_SIZE)
    # ax.set_xlabel('Number of Objects',fontsize = FONT_SIZE)
    ax.set_yscale('log')
    ax.yaxis.grid(True, alpha = 0.8)
    ax.tick_params(labelsize = FONT_SIZE)

    # handles, labels = plt.gca().get_legend_handles_labels()
    # newLabels, newHandles = [], []
    # for handle, label in zip(handles, labels):
    #     if label not in newLabels:
    #         newLabels.append(label)
    #         newHandles.append(handle)
    # plt.legend(newHandles, newLabels, fontsize = LEGEND_FONT_SIZE, ncol = 1)





    ###### 2 #####
    ax2 = fig.add_subplot(1,3,2)

    ax2.bar(np.array(Selected_NumObjs)-0.5*bar_width, [Avg_Results[Compared_Methods[0]][Selected_Densities[0]][numObjs]['Rate'] for numObjs in Selected_NumObjs], bar_width, **alg1Style)
    ax2.bar(np.array(Selected_NumObjs)+0.5*bar_width, [Avg_Results[Compared_Methods[1]][Selected_Densities[0]][numObjs]['Rate'] for numObjs in Selected_NumObjs], bar_width, **alg2Style)
    # ax2.bar(np.array(Selected_NumObjs)+1.0*bar_width, [Avg_Results[Compared_Methods[2]][Selected_Densities[0]][numObjs]['Rate'] for numObjs in Selected_NumObjs], bar_width, **alg3Style)

    ax2.set_xlim([Selected_NumObjs[0]-1.5 * bar_width, 
    Selected_NumObjs[-1]+1.5 * bar_width])
    # ax2.set_ylim([1e-4, 1e3])
    # Set x and y label. We use latex to generate text
    # ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 1)
    ax2.set_xticks(Selected_NumObjs)
    # plt.xticks(x, mysticks)
    # ax.set_ylabel('MRB',fontsize = FONT_SIZE)
    # ax.set_xlabel('Number of Objects',fontsize = FONT_SIZE)
    # ax2.set_yscale('log')
    ax2.set_ylim([0,1.2])
    ax2.yaxis.grid(True, alpha = 0.8)
    ax2.tick_params(labelsize = FONT_SIZE)

    # handles, labels = plt.gca().get_legend_handles_labels()
    # newLabels, newHandles = [], []
    # for handle, label in zip(handles, labels):
    #     if label not in newLabels:
    #         newLabels.append(label)
    #         newHandles.append(handle)
    # plt.legend(newHandles, newLabels, fontsize = LEGEND_FONT_SIZE, ncol = 1)




    ###### 3 #####
    ax2 = fig.add_subplot(1,3,3)

    ax2.bar(np.array(Selected_NumObjs)-0.5*bar_width, [Avg_Results[Compared_Methods[0]][Selected_Densities[0]][numObjs]['Ratio'] for numObjs in Selected_NumObjs], bar_width, **alg1Style)
    ax2.bar(np.array(Selected_NumObjs)+0.5*bar_width, [Avg_Results[Compared_Methods[1]][Selected_Densities[0]][numObjs]['Ratio'] for numObjs in Selected_NumObjs], bar_width, **alg2Style)
    # ax2.bar(np.array(Selected_NumObjs)+1.0*bar_width, [Avg_Results[Compared_Methods[2]][Selected_Densities[0]][numObjs]['Ratio'] for numObjs in Selected_NumObjs], bar_width, **alg3Style)

    ax2.set_xlim([Selected_NumObjs[0]-1.5 * bar_width, 
    Selected_NumObjs[-1]+1.5 * bar_width])
    # ax2.set_ylim([1e-4, 1e3])
    # Set x and y label. We use latex to generate text
    # ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 1)
    ax2.set_xticks(Selected_NumObjs)
    ax2.set_yticks([0.0,0.5,1.0,1.5])
    # plt.xticks(x, mysticks)
    # ax.set_ylabel('MRB',fontsize = FONT_SIZE)
    # ax.set_xlabel('Number of Objects',fontsize = FONT_SIZE)
    # ax2.set_yscale('log')
    ax2.set_ylim([0,1.6])
    ax2.yaxis.grid(True, alpha = 0.8)
    ax2.tick_params(labelsize = FONT_SIZE)

    # handles, labels = plt.gca().get_legend_handles_labels()
    # newLabels, newHandles = [], []
    # for handle, label in zip(handles, labels):
    #     if label not in newLabels:
    #         newLabels.append(label)
    #         newHandles.append(handle)
    # plt.legend(newHandles, newLabels, fontsize = LEGEND_FONT_SIZE, ncol = 1)

    # plt.legend(fontsize = LEGEND_FONT_SIZE, ncol = 3, loc='upper center', bbox_to_anchor=(-0.5, -0.15), fancybox=True, shadow=True)


    # plt.subplots_adjust(wspace=0.1, hspace=0)


    # Directly save the figure to a file.
    fig.savefig( os.path.dirname(my_path) + "/figures/buffer_05.eps", bbox_inches="tight", pad_inches=0.05)
    # plt.title("Density="+str(Density))
    plt.show()
    # plt.cla()


def fig_buffer_03():
    Compared_Methods = [
        "OS_SP_RB", 
        # "OS_SP_RB_Rb", 
        "OS_OPT_RB", 
        # "MCTS"
        ]

    Selected_Densities = [0.3]
    Selected_NumObjs = [10, 20, 30, 40]

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
                index = 7
                try:
                    num_collision = float(words[index])
                except Exception:
                    NotNum = True
                else:
                    NotNum = False
                while NotNum:
                    index += 1
                    try:
                        num_collision = float(words[index])
                    except Exception:
                        NotNum = True
                    else:
                        NotNum = False
                
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

    Avg_Results = deepcopy(Results)
    for method in Compared_Methods:
        for Density in Results[method].keys():
            for numObjs in Results[method][Density].keys():
                Avg_Results[method][Density][numObjs]["Rate"] = np.average(Avg_Results[method][Density][numObjs]["Rate"])
                Avg_Results[method][Density][numObjs]["Time"] = np.average([Avg_Results[method][Density][numObjs]["Time"][i] for i in range(30) if Results[method][Density][numObjs]["Rate"][i]>0.5])
                Avg_Results[method][Density][numObjs]["Ratio"] = np.average([Avg_Results[method][Density][numObjs]["Ratio"][i] for i in range(30) if Results[method][Density][numObjs]["Rate"][i]>0.5])
                Avg_Results[method][Density][numObjs]["Coll"] = np.average([Avg_Results[method][Density][numObjs]["Coll"][i] for i in range(30) if Results[method][Density][numObjs]["Rate"][i]>0.5])
                if Avg_Results[method][Density][numObjs]["Rate"] == np.average(Avg_Results[method][Density][numObjs]["Rate"]) <0.01:
                    Avg_Results[method][Density][numObjs]["Time"] = 1e-6
                    Avg_Results[method][Density][numObjs]["Ratio"] = 0
                    Avg_Results[method][Density][numObjs]["Coll"] = 0

    Joint_Results = {method:{} for method in Compared_Methods}
    for Density in Selected_Densities:
        for numObjs in Results[Compared_Methods[0]][Density].keys():
            Joint = True
            for method in Compared_Methods:
                if (Density not in Results[method]) or (numObjs not in Results[method][Density]):
                    Joint = False
                    break
            if not Joint:
                continue
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

    my_path = os.path.abspath(os.path.dirname(__file__))
    '''
    Style elements
        Here we can setup a drawing style for each algorithm.
        This helps to make the drawing style of an algorithm consistent.
    '''
    defaultStyle = {
        'label' : 'default',    # The name of the algorithm
        'ls' : '-',             # Line style, '-' means a solid line
        'linewidth' : 2,        # Line width
        "edgecolor" : 'black',
        'color' : 'k',          # Line color, 'k' means color
        'zorder' : 10,         # The 'height' of the plot. 
                                # Affects whether items are in the front of / behind each other.
        # You can add more style items here, e.g., markers.
    }
    # Here, we setup the style for an algorithm. Let's call it Alg.1.
    alg1Style = deepcopy(defaultStyle)          # First copy all default styles.
    alg1Style['label'] = r'\textsc{RBM-SP-OS}'    # Setup algorithm name. 
                                                # We use \textsc here which is a latex command. 
                                                # This is fine since we use latex to generate text.
    alg1Style['color'] = 'tab:green'           # Customized line color
                                                # https://matplotlib.org/3.1.0/gallery/color/named_colors.html
    
    if len(Compared_Methods)>=2:
        # Another algorithm
        alg2Style = deepcopy(defaultStyle)
        alg2Style['label'] = r'\textsc{RBM-OPT-OS}'
        alg2Style['color'] = 'tab:orange'

    if len(Compared_Methods)>=3:
        # Another algorithm
        alg3Style = deepcopy(defaultStyle)
        alg3Style['label'] = r'\textsc{RBM-OPT-OS}'
        alg3Style['color'] = 'tab:orange'

    if len(Compared_Methods)>=4:
        # Another algorithm
        alg4Style = deepcopy(defaultStyle)
        alg4Style['label'] = r'\textsc{'+ Compared_Methods[3] +'}'
        alg4Style['color'] = 'tab:green'


    ''' Some global variables '''
    FIGURE_SIZE = (20, 4)      # Figure width and height. 
                                    # This is a good value for 2-column paper.
    FONT_SIZE = 36                   # Size of text
    LEGEND_FONT_SIZE = 24            # We might need different font sizes for different text

    ''' 
    Parameters for legend to better utilize space.
    '''
    plt.rcParams["legend.labelspacing"] = 0.2
    plt.rcParams["legend.handlelength"] = 1.75
    plt.rcParams["legend.handletextpad"] = 0.5
    plt.rcParams["legend.columnspacing"] = 0.75

    # plt.rcParams.update({'figure.autolayout': True})

    '''
    Use latex to generate text
    Note that these params usually make the code slow. If you want to preview the figure without generating latex text, feel free to comment these. 
    '''
    plt.rcParams['ps.useafm'] = True
    plt.rcParams['pdf.use14corefonts'] = True
    plt.rcParams['text.usetex'] = True
    plt.rcParams['font.family'] = "serif"
    plt.rcParams['font.serif'] = "Times"

    ''' The real drawing part starts here. '''
    # Put your data over here.



    bar_width = 4
    # Start to create the figure
    fig = plt.figure(figsize = FIGURE_SIZE)

    ###### 1 #####
    ax = fig.add_subplot(1,3,1)

    ax.bar(np.array(Selected_NumObjs)-0.5*bar_width, [Avg_Results[Compared_Methods[0]][Selected_Densities[0]][numObjs]['Time'] for numObjs in Selected_NumObjs], bar_width, **alg1Style)
    ax.bar(np.array(Selected_NumObjs)+0.5*bar_width, [Avg_Results[Compared_Methods[1]][Selected_Densities[0]][numObjs]['Time'] for numObjs in Selected_NumObjs], bar_width, **alg2Style)
    # ax.bar(np.array(Selected_NumObjs)+1.0*bar_width, [Avg_Results[Compared_Methods[2]][Selected_Densities[0]][numObjs]['Time'] for numObjs in Selected_NumObjs], bar_width, **alg3Style)

    ax.set_xlim([Selected_NumObjs[0]-1.5 * bar_width, 
    Selected_NumObjs[-1]+1.5 * bar_width])
    ax.set_ylim([1e-4, 2e0])
    # Set x and y label. We use latex to generate text
    ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 1, loc='upper left')
    ax.set_xticks(Selected_NumObjs)
    # plt.xticks(x, mysticks)
    # ax.set_ylabel('MRB',fontsize = FONT_SIZE)
    # ax.set_xlabel('Number of Objects',fontsize = FONT_SIZE)
    ax.set_yscale('log')
    ax.yaxis.grid(True, alpha = 0.8)
    ax.tick_params(labelsize = FONT_SIZE)

    # handles, labels = plt.gca().get_legend_handles_labels()
    # newLabels, newHandles = [], []
    # for handle, label in zip(handles, labels):
    #     if label not in newLabels:
    #         newLabels.append(label)
    #         newHandles.append(handle)
    # plt.legend(newHandles, newLabels, fontsize = LEGEND_FONT_SIZE, ncol = 1)





    ###### 2 #####
    ax2 = fig.add_subplot(1,3,2)

    ax2.bar(np.array(Selected_NumObjs)-0.5*bar_width, [Avg_Results[Compared_Methods[0]][Selected_Densities[0]][numObjs]['Rate'] for numObjs in Selected_NumObjs], bar_width, **alg1Style)
    ax2.bar(np.array(Selected_NumObjs)+0.5*bar_width, [Avg_Results[Compared_Methods[1]][Selected_Densities[0]][numObjs]['Rate'] for numObjs in Selected_NumObjs], bar_width, **alg2Style)
    # ax2.bar(np.array(Selected_NumObjs)+1.0*bar_width, [Avg_Results[Compared_Methods[2]][Selected_Densities[0]][numObjs]['Rate'] for numObjs in Selected_NumObjs], bar_width, **alg3Style)

    ax2.set_xlim([Selected_NumObjs[0]-1.5 * bar_width, 
    Selected_NumObjs[-1]+1.5 * bar_width])
    # ax2.set_ylim([1e-4, 1e3])
    # Set x and y label. We use latex to generate text
    # ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 1)
    ax2.set_xticks(Selected_NumObjs)
    # plt.xticks(x, mysticks)
    # ax.set_ylabel('MRB',fontsize = FONT_SIZE)
    # ax.set_xlabel('Number of Objects',fontsize = FONT_SIZE)
    # ax2.set_yscale('log')
    ax2.set_ylim([0,1.2])
    ax2.yaxis.grid(True, alpha = 0.8)
    ax2.tick_params(labelsize = FONT_SIZE)

    # handles, labels = plt.gca().get_legend_handles_labels()
    # newLabels, newHandles = [], []
    # for handle, label in zip(handles, labels):
    #     if label not in newLabels:
    #         newLabels.append(label)
    #         newHandles.append(handle)
    # plt.legend(newHandles, newLabels, fontsize = LEGEND_FONT_SIZE, ncol = 1)




    ###### 3 #####
    ax2 = fig.add_subplot(1,3,3)

    ax2.bar(np.array(Selected_NumObjs)-0.5*bar_width, [Avg_Results[Compared_Methods[0]][Selected_Densities[0]][numObjs]['Ratio'] for numObjs in Selected_NumObjs], bar_width, **alg1Style)
    ax2.bar(np.array(Selected_NumObjs)+0.5*bar_width, [Avg_Results[Compared_Methods[1]][Selected_Densities[0]][numObjs]['Ratio'] for numObjs in Selected_NumObjs], bar_width, **alg2Style)
    # ax2.bar(np.array(Selected_NumObjs)+1.0*bar_width, [Avg_Results[Compared_Methods[2]][Selected_Densities[0]][numObjs]['Ratio'] for numObjs in Selected_NumObjs], bar_width, **alg3Style)

    ax2.set_xlim([Selected_NumObjs[0]-1.5 * bar_width, 
    Selected_NumObjs[-1]+1.5 * bar_width])
    # ax2.set_ylim([0, 1.3])
    # Set x and y label. We use latex to generate text
    # ax.legend(fontsize = LEGEND_FONT_SIZE, ncol = 1)
    ax2.set_xticks(Selected_NumObjs)
    ax2.set_yticks([0.0,0.5,1.0,1.5])
    # plt.xticks(x, mysticks)
    # ax.set_ylabel('MRB',fontsize = FONT_SIZE)
    # ax.set_xlabel('Number of Objects',fontsize = FONT_SIZE)
    # ax2.set_yscale('log')
    ax2.set_ylim([0,1.2])
    ax2.yaxis.grid(True, alpha = 0.8)
    ax2.tick_params(labelsize = FONT_SIZE)

    # handles, labels = plt.gca().get_legend_handles_labels()
    # newLabels, newHandles = [], []
    # for handle, label in zip(handles, labels):
    #     if label not in newLabels:
    #         newLabels.append(label)
    #         newHandles.append(handle)
    # plt.legend(newHandles, newLabels, fontsize = LEGEND_FONT_SIZE, ncol = 1)

    # plt.legend(fontsize = LEGEND_FONT_SIZE, ncol = 3, loc='upper center', bbox_to_anchor=(-0.5, -0.15), fancybox=True, shadow=True)


    # plt.subplots_adjust(wspace=0.1, hspace=0)


    # Directly save the figure to a file.
    fig.savefig( os.path.dirname(my_path) + "/figures/buffer_03.eps", bbox_inches="tight", pad_inches=0.05)
    # plt.title("Density="+str(Density))
    plt.show()
    # plt.cla()



if __name__ == '__main__':
    fig_buffer_03()
    fig_buffer_05()