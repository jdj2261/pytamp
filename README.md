# pytamp

[![PyPI version](https://badge.fury.io/py/pytamp.svg)](https://badge.fury.io/py/pytamp)  [![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)

Python Interface for the robot task and motion planning(TAMP) library

*We will commit the code soon.*

## Features

- Pure python library
- Support only kinematic world, so not consider dynamics in this repo
- Support simple motion planning (RRT*, Cartesian planning)
- Render Robot mesh using matplotlib or trimesh.Scene
- Support 4 benchmarks for tamp
- Support Robot TAMP with MCTS [Ours]

## Installation

### Requirements

You need [pygraphviz](https://github.com/pygraphviz/pygraphviz) and python3-tk package to see a MCTS tree.

- On Ubuntu 18.04 or 20.04, Download graphviz and graphviz-dev and python3-tk using  `apt`

  `sudo apt install graphviz graphviz-dev python3-tk`

- On Mac, Download graphviz using `brew`

  `brew install graphviz`

### Install pytamp

~~~
pip install pytamp
~~~

## SceneManager

- **Scene** 

  You can manage entire scene using SceneManager class. We recommend using the SceneManager.  
  For example, You can manage robot, object, gripper pose or collision as well as visualize their geom.   
  In addition, You can compute cartesian or RRT-star motion planning.  
  You can see various examples in `examples/scene` directory. 

  - Render using trimesh.Scene

    |                           baxter                           |                           sawyer                           |                           iiwa14                           |
    | :--------------------------------------------------------: | :--------------------------------------------------------: | :--------------------------------------------------------: |
    | <img src="img/baxter_scene.png" width="300" height="200"/> | <img src="img/sawyer_scene.png" width="300" height="200"/> | <img src="img/iiwa14_scene.png" width="300" height="200"/> |

    |                           Panda                           |                           Doosan                           |                           UR5e                           |
    | :-------------------------------------------------------: | :--------------------------------------------------------: | :------------------------------------------------------: |
    | <img src="img/panda_scene.png" width="300" height="200"/> | <img src="img/doosan_scene.png" width="300" height="200"/> | <img src="img/ur5e_scene.png" width="300" height="200"/> |
  
- **Planning**

  You can see an animation of planning that visualizes trajectory in `examples/planning` directory. 

  |                        Cartesian (X1)                        |                       RRT-star (X1)                        |
  | :----------------------------------------------------------: | :--------------------------------------------------------: |
  | <img src="img/cartesian_planning.gif" width="400" height="300"/> | <img src="img/rrt_planning.gif" width="400" height="300"/> |

## Benchmark

You can use the 4 benchmarks.

|                        Benchmark1                        |                        Benchmark2                        |                        Benchmark3                        |                        Benchmark4                        |
| :------------------------------------------------------: | :------------------------------------------------------: | :------------------------------------------------------: | :------------------------------------------------------: |
| <img src="img/benchmark1.png" width="300" height="200"/> | <img src="img/benchmark2.png" width="300" height="200"/> | <img src="img/benchmark3.png" width="300" height="200"/> | <img src="img/benchmark4.png" width="300" height="200"/> |



