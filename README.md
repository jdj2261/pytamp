# pytamp

[![PyPI version](https://badge.fury.io/py/pytamp.svg)](https://badge.fury.io/py/pytamp)  [![MIT License](http://img.shields.io/badge/license-MIT-blue.svg?style=flat)](LICENSE)

Python Interface for the robot task and motion planning(TAMP) library

*We will commit the code soon.*

## Features

- Pure python library
- Support only URDF file
- Support only kinematic world, so not consider dynamics in this repo
- Compute robot's kinematics (Ex. forward kinematics, inverse kinematics, Jacobian)
- Enable to check collision about robot self-collision and collision between robot and objects
- Support simple motion planning (RRT*, Cartesian planning)
- Render Robot mesh using matplotlib or trimesh.Scene
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

When you clone this repository, be sure to use the **--recurse-submodules** option.
The download may take a long time because of the large urdf file size.

~~~
git clone --recurse-submodules https://github.com/jdj2261/pytamp.git
~~~

If you haven't followed the example above, Enter the command below directly.

~~~
$ git clone https://github.com/jdj2261/pytamp.git
$ cd pytamp
$ git submodule init
$ git submodule update
~~~

## Quick Start

You can see various examples (info,)

- Robot Info

  You can see 6 robot info.

  `baxter, sawyer, iiwa14, iiwa7, panda, ur5e`

  ~~~shell
  $ cd example/single_stage
  $ python robot_info.py $(robot_name)
  # baxter
  $ python robot_info.py baxter
  # saywer
  $ python robot_info.py sawyer
  ~~~

- Forward kinematics

  You can compute the forward kinematics as well as visualize the visual or collision geometry.

  ~~~shell
  $ cd example/single_stage/forward_kinematics
  $ python robot_fk_baxter_test.py
  ~~~

  |                            visual                            |                          collision                           |
  | :----------------------------------------------------------: | :----------------------------------------------------------: |
  | <img src="img/baxter_plot_visual.png" width="400" height="300"/> | <img src="img/baxter_plot_collision.png" width="400" height="300"/> |

- Inverse Kinematics

  You can compute the inverse kinematics using levenberg marquardt(LM) or newton raphson(NR) method

  ~~~shell
  $ cd example/single_stage/inverse_kinematics
  $ python robot_ik_baxter_test.py
  ~~~

- Collision check

  The image below shows the collision result as well as visualize robot using trimesh.Scene class

  ~~~shell
  $ cd example/single_stage/collision
  $ python sawyer_collision_test.py
  ~~~

  |                        trimesh.Scene                         |                            Result                            |
  | :----------------------------------------------------------: | :----------------------------------------------------------: |
  | <img src="img/sawyer_mesh_collision.png" width="200" height="200"/> | <img src="img/sawyer_collision_result.png" width="600" height="200"/> |

## Visualization

You can see visualization using matplotlib library or trimesh.Scene class.

- Visualize `simple urdf` using `matplotlib`


  |                        ur5e                        |                        sawyer                        |                        iiwa14                        |                        panda                        |
  | :------------------------------------------------: | :--------------------------------------------------: | :--------------------------------------------------: | :-------------------------------------------------: |
  | <img src="img/ur5e.png" width="200" height="200"/> | <img src="img/sawyer.png" width="200" height="200"/> | <img src="img/iiwa14.png" width="200" height="200"/> | <img src="img/panda.png" width="200" height="200"/> |


- Visualize `visual geometry` using `matplotlib`


  |                           ur5e                            |                           sawyer                            |                           iiwa14                            |                           panda                            |
  | :-------------------------------------------------------: | :---------------------------------------------------------: | :---------------------------------------------------------: | :--------------------------------------------------------: |
  | <img src="img/ur5e_visual.png" width="200" height="200"/> | <img src="img/sawyer_visual.png" width="200" height="200"/> | <img src="img/iiwa14_visual.png" width="200" height="200"/> | <img src="img/panda_visual.png" width="200" height="200"/> |


- Visualize `collision geometry` using `matplotlib`


  |                             ur5e                             |                            sawyer                            |                            iiwa14                            |                            panda                             |
  | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
  | <img src="img/ur5e_collision.png" width="200" height="200"/> | <img src="img/sawyer_collision.png" width="200" height="200"/> | <img src="img/iiwa14_collision.png" width="200" height="200"/> | <img src="img/panda_collision.png" width="200" height="200"/> |

- Visualize mesh about `visual/collision geometry` using `trimesh.Scene class`

  ![baxter](img/all_robot.png)

## SceneManager

- **Scene** 

  You can manage entire scene using SceneManager class. We recommend using the SceneManager.  
  For example, You can manage robot, object, gripper pose or collision as well as visualize their geom.   
  In addition, You can compute cartesian or motion planning.  
  You can see various examples in `example/scene` directory. 

  - Render using trimesh.Scene

    |                           baxter                           |                           sawyer                           |                           iiwa14                           |
    | :--------------------------------------------------------: | :--------------------------------------------------------: | :--------------------------------------------------------: |
    | <img src="img/baxter_scene.png" width="300" height="200"/> | <img src="img/sawyer_scene.png" width="300" height="200"/> | <img src="img/iiwa14_scene.png" width="300" height="200"/> |

  - Render using matplotlib

    |                            baxter                            |                            sawyer                            |                            iiwa14                            |
    | :----------------------------------------------------------: | :----------------------------------------------------------: | :----------------------------------------------------------: |
    | <img src="img/baxter_scene_matplotlib.png" width="300" height="200"/> | <img src="img/sawyer_scene_matplotlib.png" width="300" height="200"/> | <img src="img/iiwa14_scene_matplotlib.png" width="300" height="200"/> |

- **Attach object to robot**

  You can manage by attaching or detaching objects to the gripper.  
  These method will be used for motion planning. You can see a example in `example/scene/gripper` directory. 

  ~~~shell
  $ cd example/scene/gripper
  $ python scene_attach_detach_test.py
  ~~~

  You can see the attaching process as shown in the figure below.  
  If the object is attached, a color of the object will change black. And then, the object becomes a part of the robot.
  
  |                       Move a robot                       |                 Attach an object to robot                  |                         Move a robot                         |                      Detach an object                      |
  | :------------------------------------------------------: | :--------------------------------------------------------: | :----------------------------------------------------------: | :--------------------------------------------------------: |
  | <img src="img/panda_move.png" width="400" height="200"/> | <img src="img/panda_attach.png" width="400" height="200"/> | <img src="img/panda_move_attached.png" width="400" height="200"/> | <img src="img/panda_detach.png" width="400" height="200"/> |
  
- **Planning**

  You can see an animation of planning that visualizes trajectory in `example/planning` directory. 

  |                        Cartesian (X1)                        |                       RRT-star (X1)                        |
  | :----------------------------------------------------------: | :--------------------------------------------------------: |
  | <img src="img/cartesian_planning.gif" width="500" height="300"/> | <img src="img/rrt_planning.gif" width="500" height="300"/> |

- **Pick and Place**

  You can see an animation of pick and place that visualizes trajectory  in `example/action` directory.
  
  |                Scene arranged objects (X4)                 |                Scene stacked objects (X4)                 |
  | :--------------------------------------------------------: | :-------------------------------------------------------: |
  | <img src="img/pnp_arranged.gif" width="500" height="300"/> | <img src="img/pnp_stacked.gif" width="500" height="300"/> |
