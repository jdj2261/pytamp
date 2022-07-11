import numpy as np
import matplotlib.pyplot as plt

from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh

from pytamp.search.mcts import MCTS
from pytamp.scene.scene_manager import SceneManager


file_path = 'urdf/panda/panda.urdf'
robot = SingleArm(
    f_name=file_path, 
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), 
    has_gripper=True)
robot.setup_link_name("panda_link_0", "panda_right_hand")
robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])

red_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77]), rot=[0, 0, np.pi/6])
blue_box_pose = Transform(pos=np.array([0.6, 0.35, 0.77]))
green_box_pose = Transform(pos=np.array([0.6, 0.05, 0.77]))
test1_box_pose = Transform(pos=np.array([0.5, 0.2, 0.77]))
test2_box_pose = Transform(pos=np.array([0.5, 0.35, 0.77]))
test3_box_pose = Transform(pos=np.array([0.5, 0.05, 0.77]))
test4_box_pose = Transform(pos=np.array([0.4, 0.2, 0.77]))
test5_box_pose = Transform(pos=np.array([0.4, 0.35, 0.77]))

support_box_pose = Transform(pos=np.array([0.6, -0.2, 0.77]), rot=np.array([0, np.pi/2, 0]))
table_pose = Transform(pos=np.array([0.4, 0.24, 0.0]))

red_cube_mesh = get_object_mesh('ben_cube.stl', 0.06)
blue_cube_mesh = get_object_mesh('ben_cube.stl', 0.06)
green_cube_mesh = get_object_mesh('ben_cube.stl', 0.06)
goal_box_mesh = get_object_mesh('goal_box.stl', 0.001)
table_mesh = get_object_mesh('custom_table.stl', 0.01)

param = {'stack_num' : 3, 'goal_object' : 'goal_box'}
benchmark_config = {1 : param}

scene_mngr = SceneManager("collision", is_pyplot=True, benchmark=benchmark_config)
scene_mngr.add_object(name="table", gtype="mesh", gparam=table_mesh, h_mat=table_pose.h_mat, color=[0.39, 0.263, 0.129])
scene_mngr.add_object(name="A_box", gtype="mesh", gparam=red_cube_mesh, h_mat=red_box_pose.h_mat, color=[1.0, 0.0, 0.0])
scene_mngr.add_object(name="B_box", gtype="mesh", gparam=blue_cube_mesh, h_mat=blue_box_pose.h_mat, color=[0.0, 0.0, 1.0])
scene_mngr.add_object(name="C_box", gtype="mesh", gparam=green_cube_mesh, h_mat=green_box_pose.h_mat, color=[0.0, 1.0, 0.0])
# scene_mngr.add_object(name="D_box", gtype="mesh", gparam=green_cube_mesh, h_mat=test1_box_pose.h_mat, color=[1.0, 1.0, 0.0])
# scene_mngr.add_object(name="E_box", gtype="mesh", gparam=green_cube_mesh, h_mat=test2_box_pose.h_mat, color=[0.0, 1.0, 1.0])
# scene_mngr.add_object(name="F_box", gtype="mesh", gparam=green_cube_mesh, h_mat=test3_box_pose.h_mat, color=[1.0, 0.0, 1.0])
# scene_mngr.add_object(name="G_box", gtype="mesh", gparam=green_cube_mesh, h_mat=test4_box_pose.h_mat, color=[0.3, 0.0, 1.0])
# scene_mngr.add_object(name="H_box", gtype="mesh", gparam=green_cube_mesh, h_mat=test5_box_pose.h_mat, color=[1.0, 0.3, 1.0])
scene_mngr.add_object(name="goal_box", gtype="mesh", gparam=goal_box_mesh, h_mat=support_box_pose.h_mat, color=[1.0, 0, 1.0])
scene_mngr.add_robot(robot, robot.init_qpos)
############################# Logical State #############################

scene_mngr.scene.logical_states["A_box"] = {scene_mngr.scene.logical_state.on : scene_mngr.scene.objs["table"]}
scene_mngr.scene.logical_states["B_box"] = {scene_mngr.scene.logical_state.on : scene_mngr.scene.objs["table"]}
scene_mngr.scene.logical_states["C_box"] = {scene_mngr.scene.logical_state.on : scene_mngr.scene.objs["table"]}
# scene_mngr.set_logical_state("D_box", ("on", "table"))
# scene_mngr.set_logical_state("E_box", ("on", "table"))
# scene_mngr.set_logical_state("F_box", ("on", "table"))
# scene_mngr.set_logical_state("G_box", ("on", "table"))
# scene_mngr.set_logical_state("H_box", ("on", "table"))
scene_mngr.scene.logical_states["goal_box"] = {scene_mngr.scene.logical_state.on : scene_mngr.scene.objs["table"]}
scene_mngr.scene.logical_states["table"] = {scene_mngr.scene.logical_state.static : True}
scene_mngr.scene.logical_states[scene_mngr.gripper_name] = {scene_mngr.scene.logical_state.holding : None}
scene_mngr.update_logical_states()

mcts = MCTS(scene_mngr)
mcts.debug_mode = False

# 최대부터
mcts.budgets = 100
mcts.max_depth = 20
# mcts.exploration_c = 30
mcts.exploration_c = 300
# mcts.sampling_method = 'bai_ucb' # 405
mcts.sampling_method = 'bai_perturb' # 58
# mcts.sampling_method = 'uct' # 369
nodes = mcts.do_planning()


subtree = mcts.get_subtree()
mcts.visualize_tree("MCTS", subtree)

best_nodes = mcts.get_best_node(subtree)
if best_nodes:
    print("\nBest Action Node")
    for node in best_nodes:
        mcts.show_logical_action(node)

rewards = mcts.rewards
max_iter = np.argmax(rewards)
print(max_iter)
plt.plot(rewards)
plt.show()

# Do planning
best_nodes = mcts.get_best_node(subtree)
pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(best_nodes)
mcts.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)