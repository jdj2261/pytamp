import numpy as np
import sys, os
import yaml

from pykin import assets
from pykin.utils import plot_utils as p_utils

from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh
from pykin.utils.transform_utils import get_matrix_from_rpy

from pytamp.scene.scene_manager import SceneManager
from pytamp.planners.rrt_star_planner import RRTStarPlanner

fig, ax = p_utils.init_3d_figure()
asset_file_path = os.path.abspath(assets.__file__ + "/../")
file_path = "urdf/fanuc/fanuc_r2000ic_165f.urdf"
robot = SingleArm(
    f_name=file_path, offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0]), has_gripper=False
)
robot.setup_link_name("base_link", "link_6")

custom_fpath = asset_file_path + "/config/fanuc_init_params.yaml"
with open(custom_fpath) as f:
    controller_config = yaml.safe_load(f)
init_qpos = controller_config["init_qpos"]

red_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77]))
blue_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77 + 0.06]))
green_box_pose = Transform(pos=np.array([0.5, 0.5, 0.5]))
support_box_pose = Transform(pos=np.array([0.6, -0.2, 0.77]), rot=np.array([0, np.pi / 2, 0]))
table_pose = Transform(pos=np.array([0.4, 0.24, 0.0]))

# red_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
# blue_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
# green_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
# goal_box_mesh = get_object_mesh("goal_box.stl", 0.001)
# table_mesh = get_object_mesh("custom_table.stl", 0.01)

scene_mngr = SceneManager("collision", is_pyplot=True)
# scene_mngr.add_object(
#     name="table", gtype="mesh", gparam=table_mesh, h_mat=table_pose.h_mat, color=[0.823, 0.71, 0.55]
# )
# scene_mngr.add_object(
#     name="red_box",
#     gtype="mesh",
#     gparam=red_cube_mesh,
#     h_mat=red_box_pose.h_mat,
#     color=[1.0, 0.0, 0.0],
# )
# scene_mngr.add_object(
#     name="blue_box",
#     gtype="mesh",
#     gparam=blue_cube_mesh,
#     h_mat=blue_box_pose.h_mat,
#     color=[0.0, 0.0, 1.0],
# )
# scene_mngr.add_object(
#     name="green_box",
#     gtype="mesh",
#     gparam=green_cube_mesh,
#     h_mat=green_box_pose.h_mat,
#     color=[0.0, 1.0, 0.0],
# )
# scene_mngr.add_object(
#     name="goal_box",
#     gtype="mesh",
#     gparam=goal_box_mesh,
#     h_mat=support_box_pose.h_mat,
#     color=[1.0, 0, 1.0],
# )
init_qpos = [0, 0, 0, 0, 0, 0]
scene_mngr.add_robot(robot, init_qpos)

init_pose = scene_mngr.get_robot_eef_pose()
# print(init_pose)
grasp_pose = green_box_pose.h_mat
r_mat = get_matrix_from_rpy(np.array([0, np.pi / 2, 0]))
grasp_pose[:3, :3] = r_mat
grasp_pose[:3, 3] = grasp_pose[:3, 3]

print(init_qpos)
print(grasp_pose)
target_thetas = scene_mngr.scene.robot.get_result_qpos(init_qpos, grasp_pose)
scene_mngr.set_robot_eef_pose(target_thetas)
# scene_mngr.attach_object_on_gripper("green_box", False)

planner = RRTStarPlanner(delta_distance=0.05, epsilon=0.2, gamma_RRT_star=2, dimension=6)

planner.run(scene_mngr=scene_mngr, cur_q=init_qpos, goal_pose=grasp_pose, max_iter=300)

joint_path = planner.get_joint_path(n_step=15)
target_eef_poses = planner.get_target_eef_poses()

scene_mngr.animation(
    ax,
    fig,
    joint_path=joint_path,
    eef_poses=target_eef_poses,
    visible_text=True,
    alpha=1,
    interval=50,
    repeat=True,
    visible_body=True
)
