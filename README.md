# Tabletop Rearrangement of Heterogenous Objects


## Repository Overview
This GitHub repository contains source codes for our ICRA 2022 work TRLB and its follow-up work in IROS 2023. Each branch is dedicated to a particular paper.

1. **main**: This branch contains the source codes related to **Fast High-Quality Tabletop Rearrangement in Bounded Workspace**, which is published in ICRA 2022.
2. **heteTORO**: This branch contains the source codes related to **Effectively Rearranging Heterogeneous Objects on Cluttered Tabletops**, which is published in IROS 2023.


## Setup Instruction

1. It is recommended to use a virtual environment with Python 3 for this project, e.g., `conda create -n HeteTORO python=3.8`.
2. Make sure you are in your virtual environment. 
3. Run `pip install -e .` to install the package and dependencies. 

## Run examples
`cd ./hetrogenous_objects`

`python run_experiments.py`

## Customize example code
Please go to ./hetrogenous_objects/run_experiments.py Line 23

Explanations and options of MCTS planners can be found in ./hetrogenous_objects/Python_MCTS/MCTS.py

