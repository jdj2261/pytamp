# pytamp

[![PyPI version](https://badge.fury.io/py/pytamp.svg)](https://badge.fury.io/py/pytamp)  [![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)

Python Interface for the robot task and motion planning(TAMP) library

*Our paper named "Perturbation-Based Best Arm Identification for Efficient Task Planning with Monte-Carlo Tree Search", accepted to ICRA 2023.*

In the future, we plan to update it so that it can be operated in mujoco simulator using pytamp.

We want to provide an environment for applying reinforcement learning in the robot domain.

## Features

- Pure python library
- Support only kinematic world, so not consider dynamics in this repo
- Support simple motion planning (RRT* , PRM* , Cartesian planning)
- Render Robot mesh using matplotlib or trimesh.Scene
- Support 4 benchmarks for tamp
- Support Robot TAMP with MCTS

## Installation

### Requirements

You need [pygraphviz](https://github.com/pygraphviz/pygraphviz) and python3-tk package to see a MCTS Tree.

- On Ubuntu 18.04 or 20.04, Download graphviz and graphviz-dev and python3-tk using  `apt`

  `sudo apt install graphviz graphviz-dev python3-tk`

- On Mac, Download graphviz using `brew`

  `brew install graphviz`

### Install pytamp

~~~
pip install pytamp
~~~

## Usage

~~~shell
# Benchmark 1 with MCTS
$ sh scripts/run_benchmark1.sh
# Benchmark 2 with MCTS
$ sh scripts/run_benchmark2.sh
# Benchmark 3 with MCTS
$ sh scripts/run_benchmark3.sh
# Benchmark 4 with MCTS
$ sh scripts/run_benchmark4.sh
~~~

If you want to see the result of the output,  Run the benchmark#_result_level_2.ipynb file for that benchmark in the **results** directory.

## Examples

If you go to the examples/doosan directory, there are action, benchmark, heuristic, and mcts directory.

You can see 3D animation or image of pick, place, pick and place for each benchmark, 

### SceneManager

- **Scene** 

  You can manage entire scene using *SceneManager* class. We recommend using the *SceneManager*.  
  For example, You can manage robot, object, gripper pose or collision as well as visualize their geom.   
  In addition, You can compute cartesian or RRT-star motion planning.  
  You can see various examples in `examples/scene` directory. 

  |                           baxter                           |                           sawyer                           |                           iiwa14                           |
  | :--------------------------------------------------------: | :--------------------------------------------------------: | :--------------------------------------------------------: |
  | <img src="img/baxter_scene.png" width="300" height="200"/> | <img src="img/sawyer_scene.png" width="300" height="200"/> | <img src="img/iiwa14_scene.png" width="300" height="200"/> |
  
  |                           Panda                           |                           Doosan                           |                           UR5e                           |
  | :-------------------------------------------------------: | :--------------------------------------------------------: | :------------------------------------------------------: |
  | <img src="img/panda_scene.png" width="300" height="200"/> | <img src="img/doosan_scene.png" width="300" height="200"/> | <img src="img/ur5e_scene.png" width="300" height="200"/> |
  
- **Planning**

  You can see an animation of planning that visualizes trajectory in `examples/planning` directory. 


### Benchmark

You can use the 4 benchmarks.

|                          Benchmark1                          |                          Benchmark2                          |                          Benchmark3                          |                          Benchmark4                          |
| :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
| <img src="img/Benchmark1_Init.png" width="300" height="200"/> | <img src="img/Benchmark2_Init.png" width="300" height="200"/> | <img src="img/Benchmark3_Init.png" width="300" height="200"/> | <img src="img/Benchmark4_Init.png" width="300" height="200"/> |
