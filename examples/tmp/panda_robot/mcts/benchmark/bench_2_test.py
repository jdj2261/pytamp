import numpy as np
import matplotlib.pyplot as plt

from pykin.utils import plot_utils as p_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh

from pytamp.search.mcts import MCTS
from pytamp.scene.scene_manager import SceneManager


file_path = 'urdf/doosan/doosan_with_robotiq140.urdf'
robot = SingleArm(
    f_name=file_path, 
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), 
    has_gripper=True,
    gripper_name = "robotiq140")
robot.setup_link_name("base_0", "right_hand")
robot.init_qpos = np.array([ 0, -np.pi/3, np.pi/1.5, 0, np.pi/3,  np.pi/2])

shelf_pose = Transform(pos=np.array([0.75, 0, 1.41725156]),rot=np.array([0, 0, np.pi/2]))
bin_pose = Transform(pos=np.array([-0.1, 1.0, 0.3864222]))

bottle_meshes = []
for i in range(6):
    bottle_meshes.append(get_object_mesh('bottle.stl'))
bottle_pose1 = Transform(pos=np.array([0.75, 0.03, 1.29]))
bottle_pose2 = Transform(pos=np.array([0.70, 0.10, 1.29]))
bottle_pose3 = Transform(pos=np.array([0.70, -0.05,1.29]))
bottle_pose4 = Transform(pos=np.array([0.90, 0.1, 1.29]))
bottle_pose5 = Transform(pos=np.array([0.90, 0, 1.29]))
bottle_pose6 = Transform(pos=np.array([0.90, -0.1, 1.29]))

param = {'goal_object' : 'goal_bottle'}
benchmark_config = {2 : param}
scene_mngr = SceneManager("collision", is_pyplot=True, benchmark=benchmark_config)

"""
13, 8, 0
15,  9
17, 16, 2
"""

for i in range(20):
    shelf_name = 'shelf_' + str(i)
    shelf_mesh_test = get_object_mesh(shelf_name + '.stl', scale=0.9)
    scene_mngr.add_object(name=shelf_name, gtype="mesh", h_mat=shelf_pose.h_mat, gparam=shelf_mesh_test, color=[0.39, 0.263, 0.129])

for i in range(20):
    bin_name = 'bin_' + str(i)
    bin_mesh_test = get_object_mesh(bin_name + '.stl', scale=0.9)
    scene_mngr.add_object(name=bin_name, gtype="mesh", h_mat=bin_pose.h_mat, gparam=bin_mesh_test, color=[0.8 + i*0.01, 0.8 + i*0.01, 0.8 + i*0.01])

# scene_mngr.add_object(name="tray_red", gtype="mesh", gparam=tray_red_mesh, h_mat=tray_red_pose.h_mat, color=[1.0, 0, 0])

scene_mngr.add_object(name="goal_bottle", gtype="mesh", h_mat=bottle_pose1.h_mat, gparam=bottle_meshes[0], color=[1., 0., 0.])
scene_mngr.add_object(name="bottle_2", gtype="mesh", h_mat=bottle_pose2.h_mat, gparam=bottle_meshes[1], color=[0., 1., 0.])
scene_mngr.add_object(name="bottle_3", gtype="mesh", h_mat=bottle_pose3.h_mat, gparam=bottle_meshes[2], color=[0., 1., 0.])
scene_mngr.add_object(name="bottle_4", gtype="mesh", h_mat=bottle_pose4.h_mat, gparam=bottle_meshes[3], color=[0., 1., 0.])
scene_mngr.add_object(name="bottle_5", gtype="mesh", h_mat=bottle_pose5.h_mat, gparam=bottle_meshes[4], color=[0., 1., 0.])
scene_mngr.add_object(name="bottle_6", gtype="mesh", h_mat=bottle_pose6.h_mat, gparam=bottle_meshes[5], color=[0., 1., 0.])
scene_mngr.add_robot(robot, robot.init_qpos)

# scene_mngr.set_logical_state("tray_red", (scene_mngr.scene.logical_state.static, True))
scene_mngr.set_logical_state("goal_bottle", ("on", "shelf_9"))
scene_mngr.set_logical_state("bottle_2", ("on", "shelf_9"))
scene_mngr.set_logical_state("bottle_3", ("on", "shelf_9"))
scene_mngr.set_logical_state("bottle_4", ("on", "shelf_9"))
scene_mngr.set_logical_state("bottle_5", ("on", "shelf_9"))
scene_mngr.set_logical_state("bottle_6", ("on", "shelf_9"))

for i in range(20):
    scene_mngr.set_logical_state(f"shelf_"+str(i), (scene_mngr.scene.logical_state.static, True))
    scene_mngr.set_logical_state(f"bin_"+str(i), (scene_mngr.scene.logical_state.static, True))
scene_mngr.set_logical_state(scene_mngr.gripper_name, (scene_mngr.scene.logical_state.holding, None))
scene_mngr.update_logical_states(is_init=True)

scene_mngr.show_logical_states()

# fig, ax = p_utils.init_3d_figure(name="Benchmark 2")
# result, names = scene_mngr.collide_objs_and_robot(return_names=True)
# scene_mngr.render_scene(ax)
# scene_mngr.show()

mcts = MCTS(scene_mngr)
mcts.debug_mode = False

# 최대부터
mcts.budgets = 300
mcts.max_depth = 20
# mcts.c = 30
mcts.c = 300
# mcts.sampling_method = 'bai_ucb' 
mcts.sampling_method = 'bai_perturb' 
# mcts.sampling_method = 'uct' 
for i in range(mcts.budgets):
    mcts.do_planning(i)


subtree = mcts.get_success_subtree()
mcts.visualize_tree("MCTS", subtree)

best_nodes = mcts.get_best_node(subtree)

rewards = mcts.rewards_for_level_1
max_iter = np.argmax(rewards)
print(max_iter)
plt.plot(rewards)
plt.show()

best_nodes = mcts.get_best_node(subtree)

pnp_all_joint_path, pick_all_objects, place_all_object_poses = mcts.get_all_joint_path(best_nodes)
mcts.place_action.simulate_path(pnp_all_joint_path, pick_all_objects, place_all_object_poses)