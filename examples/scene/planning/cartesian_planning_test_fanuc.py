import numpy as np
import sys, os
import yaml

from pykin import assets
from pykin.utils import plot_utils as p_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh
from pykin.utils.transform_utils import get_matrix_from_rpy

from pytamp.planners.cartesian_planner import CartesianPlanner
from pytamp.scene.scene_manager import SceneManager

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
init_qpos = [0, 45, 0, 0, 0, 0]
goal_pose = Transform(pos=np.array([1, -0.6, 0.5]))
scene_mngr = SceneManager("collision", is_pyplot=True)
scene_mngr.add_robot(robot, init_qpos)

init_pose = scene_mngr.get_robot_eef_pose()
print(init_pose)
grasp_pose = goal_pose.h_mat
r_mat = get_matrix_from_rpy(np.array([0, np.pi / 2, 0]))
grasp_pose[:3, :3] = r_mat
grasp_pose[:3, 3] = grasp_pose[:3, 3] - [0.10, 0, 0]

target_thetas = scene_mngr.scene.robot.get_result_qpos(init_qpos, grasp_pose)

scene_mngr.set_robot_eef_pose(target_thetas)
# scene_mngr.attach_object_on_gripper("green_box", False)

# scene_mngr.render_scene(ax, only_visible_geom=True, alpha=0.7)
# scene_mngr.show()


############################ Show collision info #############################
planner = CartesianPlanner(n_step=50, dimension=6)

planner.run(scene_mngr=scene_mngr, cur_q=target_thetas, goal_pose=init_pose)

joint_path = planner.get_joint_path()
eef_poses = planner.get_target_eef_poses()

# print(eef_poses)
scene_mngr.animation(
    ax,
    fig,
    joint_path=joint_path,
    eef_poses=eef_poses,
    visible_gripper=False,
    visible_text=True,
    alpha=1.0,
    interval=50,
    repeat=True,
)
