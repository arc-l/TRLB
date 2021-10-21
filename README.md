# Tabletop Object Rearrangement with Lazy Buffer Allocation (TRLB)
![scenarios2](https://user-images.githubusercontent.com/53358252/138315133-094c222a-6f61-4e7f-9e7f-58a0b4c63273.gif)


## Setup Instruction

1. It is recommended to use a virtual environment with Python 3 for this project, e.g., `conda create -n TRLB python=3.6.9`.
2. Make sure you are in your virtual environment. 
3. Run `pip install -e .` to install the TRLB package and dependencies. 

## Cylindrical Instances
We show a cylindrical instance where the discs with solid boundaries represent the start arrangement and the discs with dashed boundaries represent the goal arrangement.

<p align="center">
  <img src="https://user-images.githubusercontent.com/53358252/136728746-b6e40e51-d871-462f-928b-cc3fe5ba8729.png" alt="cylindrical_example"/>
</p>
  
For cylindrical objects, we present various component options for TRLB, including:
1. **Primitive plan computation**: running buffer minimization (RBM), total buffer minimization (TBM), random order (RO); 
2. **Buffer generation methods**: optimization (OPT), sampling (SP); 
3. **High level planners**: one-shot (OS), forward search tree (ST), bidirectional search tree (BST); 
4. With or without **preprocessing (PP)**.

We show a demo in `TRLB/disk_experiments/run_experiments.py`
Please note that the optimization for buffer generation is supported by [Gurobi](https://www.gurobi.com/). If you do not have a Gurobi license or do not want to apply for a Gurobi license, please always set `buffer_generation='SP'` in `run_experiments.py`.

### Run
To run the demo, just execute `TRLB/disk_experiments/run_experiments.py` with python. For example, in the root folder of the project run the following command in the terminal.
`python ./disk_experiments/run_experiments.py`

## Cuboid Instances
We show a cuboid instance where the rectangles with solid boundaries represent the start arrangement and the rectangles with dashed boundaries represent the goal arrangement.
<p align="center">
  <img src="https://user-images.githubusercontent.com/53358252/136731248-ad78ed65-506e-4a72-ab82-1dce1b6909b6.png" alt="cuboid_example"/>
</p>

For cuboid instances, we currently only support `RBM-SP-BST`, using the primitive plans that minimize running buffer size,  performing buffer allocation by sampling, maintaining a bidirectional search tree, and doing so without preprocessing. a demo is shown in `TRLB/stick_experiments/run_experiments.py`.

### Run
To run the demo, just execute `TRLB/stick_experiments/run_experiments.py` with python. For example, in the root folder of the project run the following command in the terminal.
`python ./stick_experiments/run_experiments.py`

## Hardware Demonstration
TRLB is proven to be efficient on our hardware platform in various scenarios.

### Comparitive Study: Cylindrical Objects

https://user-images.githubusercontent.com/53358252/138213916-cd2b9283-97b4-4a28-8962-90d9a5efddc8.mp4


### Comparitive Study: Cuboid Objects

https://user-images.githubusercontent.com/53358252/138214434-d10c7bd6-f8aa-4672-93d5-ca70b8f3ec3c.mp4
