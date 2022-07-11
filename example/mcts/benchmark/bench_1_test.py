import numpy as np
import argparse
import matplotlib.pyplot as plt

from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh

from pytamp.search.mcts import MCTS
from pytamp.scene.scene_manager import SceneManager

parser = argparse.ArgumentParser(description='Test Benchmark 1.')
parser.add_argument('--budgets', metavar='T', type=int, default=5000, help='an integer for the accumulator')
parser.add_argument('--max_depth', metavar='H', type=int, default=20, help='sum the integers (default: find the max)')
parser.add_argument('--seed', metavar='i', type=int, default=1, help='A random seed')
parser.add_argument('--algo', metavar='alg', type=str, default='bai_perturb', choices=['bai_perturb', 'bai_ucb', 'uct'], help='A type of noise')
parser.add_argument('--debug_mode', metavar='debug', type=bool, default=False, help='')
args = parser.parse_args()

debug_mode = args.debug_mode
seed = args.seed
np.random.seed(seed)

file_path = 'urdf/panda/panda.urdf'
robot = SingleArm(
    f_name=file_path, 
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), 
    has_gripper=True)
robot.setup_link_name("panda_link_0", "panda_right_hand")
robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])

A_box_pose = Transform(pos=np.array([0.6, 0.05, 0.77]))
B_box_pose = Transform(pos=np.array([0.6, 0.15, 0.77]))
C_box_pose = Transform(pos=np.array([0.6, 0.25, 0.77]))
D_box_pose = Transform(pos=np.array([0.5, 0.05, 0.77]))
E_box_pose = Transform(pos=np.array([0.5, 0.15, 0.77]))
F_box_pose = Transform(pos=np.array([0.5, 0.25, 0.77]))
table_pose = Transform(pos=np.array([1.0, -0.4, -0.03]))
ceiling_pose = Transform(pos=np.array([1.1, -0.4, 1.5]))
tray_red_pose = Transform(pos=np.array([0.6, -0.4-0.3, 0.9]))
tray_blue_pose = Transform(pos=np.array([0.6, 0.4, 0.9]))

box_meshes = []
for i in range(6):
    box_meshes.append(get_object_mesh('ben_cube.stl', 0.06))
goal_box_mesh = get_object_mesh('goal_box.stl', 0.001)
table_mesh = get_object_mesh('ben_table.stl')
ceiling_mesh = get_object_mesh('ben_table_ceiling.stl')
tray_red_mesh = get_object_mesh('ben_tray_red.stl')
tray_blue_mesh = get_object_mesh('ben_tray_blue.stl')

param = {'stack_num' : 6, 'goal_object' : "tray_red"}
benchmark_config = {1 : param}

scene_mngr = SceneManager("collision", is_pyplot=True, benchmark=benchmark_config)
scene_mngr.add_object(name="A_box", gtype="mesh", gparam=box_meshes[0], h_mat=A_box_pose.h_mat, color=[1.0, 0.0, 0.0])
scene_mngr.add_object(name="B_box", gtype="mesh", gparam=box_meshes[1], h_mat=B_box_pose.h_mat, color=[0.0, 1.0, 0.0])
scene_mngr.add_object(name="C_box", gtype="mesh", gparam=box_meshes[2], h_mat=C_box_pose.h_mat, color=[0.0, 0.0, 1.0])
scene_mngr.add_object(name="D_box", gtype="mesh", gparam=box_meshes[3], h_mat=D_box_pose.h_mat, color=[1.0, 1.0, 0.0])
scene_mngr.add_object(name="E_box", gtype="mesh", gparam=box_meshes[4], h_mat=E_box_pose.h_mat, color=[0.0, 1.0, 1.0])
scene_mngr.add_object(name="F_box", gtype="mesh", gparam=box_meshes[5], h_mat=F_box_pose.h_mat, color=[1.0, 0.0, 1.0])
scene_mngr.add_object(name="table", gtype="mesh", gparam=table_mesh, h_mat=table_pose.h_mat, color=[0.39, 0.263, 0.129])
scene_mngr.add_object(name="ceiling", gtype="mesh", gparam=ceiling_mesh, h_mat=ceiling_pose.h_mat, color=[0.39, 0.263, 0.129])
scene_mngr.add_object(name="tray_red", gtype="mesh", gparam=tray_red_mesh, h_mat=tray_red_pose.h_mat, color=[1.0, 0, 0])
scene_mngr.add_object(name="tray_blue", gtype="mesh", gparam=tray_blue_mesh, h_mat=tray_blue_pose.h_mat, color=[0, 0, 1.0])
scene_mngr.add_robot(robot, robot.init_qpos)

scene_mngr.set_logical_state("A_box", ("on", "table"))
scene_mngr.set_logical_state("B_box", ("on", "table"))
scene_mngr.set_logical_state("C_box", ("on", "table"))
scene_mngr.set_logical_state("D_box", ("on", "table"))
scene_mngr.set_logical_state("E_box", ("on", "table"))
scene_mngr.set_logical_state("F_box", ("on", "table"))
scene_mngr.set_logical_state("ceiling", (scene_mngr.scene.logical_state.static, True))
scene_mngr.set_logical_state("tray_red", (scene_mngr.scene.logical_state.static, True))
scene_mngr.set_logical_state("tray_blue", (scene_mngr.scene.logical_state.static, True))

scene_mngr.set_logical_state("table", (scene_mngr.scene.logical_state.static, True))
scene_mngr.set_logical_state(scene_mngr.gripper_name, (scene_mngr.scene.logical_state.holding, None))
scene_mngr.update_logical_states()

c_list = 10**np.linspace(0., 4., 10)
mcts_list = []

for c in c_list:
    mcts = MCTS(scene_mngr)
    mcts.debug_mode = debug_mode

    # 최대부터
    mcts.budgets = args.budgets
    mcts.max_depth = args.max_depth
    mcts.exploration_c = c
    mcts.sampling_method = args.algo

    nodes = mcts.do_planning()
    subtree = mcts.get_subtree()
    # mcts.visualize_tree("MCTS", subtree)

    best_nodes = mcts.get_best_node(subtree)

    rewards = mcts.rewards
    max_iter = np.argmax(rewards)
    print(max_iter)
    
    if debug_mode:
        plt.plot(rewards)
        plt.show()

    # Do planning
    best_nodes = mcts.get_best_node(subtree)
    pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(best_nodes)

    if debug_mode:
        mcts.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)