import numpy as np
import sys, os
import yaml

from pykin import assets

from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pytamp.scene.scene_manager import SceneManager
from pykin.utils.mesh_utils import get_object_mesh
from pykin.utils import plot_utils as p_utils


asset_file_path = os.path.abspath(assets.__file__ + "/../")
fig, ax = p_utils.init_3d_figure()
file_path = "urdf/ur5e/ur5e_with_robotiq140.urdf"
robot = SingleArm(file_path, Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]))
robot.setup_link_name("ur5e_base_link", "ur5e_right_hand")

red_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77]))
blue_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77 + 0.06]))
green_box_pose = Transform(pos=np.array([0.6, 0.2, 0.77 + 0.12]))
support_box_pose = Transform(
    pos=np.array([0.6, -0.2, 0.77]), rot=np.array([0, np.pi / 2, 0])
)
table_pose = Transform(pos=np.array([0.4, 0.24, 0.0]))

red_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
blue_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
green_cube_mesh = get_object_mesh("ben_cube.stl", 0.06)
goal_box_mesh = get_object_mesh("goal_box.stl", 0.001)
table_mesh = get_object_mesh("custom_table.stl", 0.01)

scene_mngr = SceneManager("visual", False)
scene_mngr.add_object(
    name="table",
    gtype="mesh",
    gparam=table_mesh,
    h_mat=table_pose.h_mat,
    color=[0.823, 0.71, 0.55],
)
scene_mngr.add_object(
    name="red_box",
    gtype="mesh",
    gparam=red_cube_mesh,
    h_mat=red_box_pose.h_mat,
    color=[1.0, 0.0, 0.0],
)
scene_mngr.add_object(
    name="blue_box",
    gtype="mesh",
    gparam=blue_cube_mesh,
    h_mat=blue_box_pose.h_mat,
    color=[0.0, 0.0, 1.0],
)
scene_mngr.add_object(
    name="green_box",
    gtype="mesh",
    gparam=green_cube_mesh,
    h_mat=green_box_pose.h_mat,
    color=[0.0, 1.0, 0.0],
)
scene_mngr.add_object(
    name="goal_box",
    gtype="mesh",
    gparam=goal_box_mesh,
    h_mat=support_box_pose.h_mat,
    color=[1.0, 0, 1.0],
)
scene_mngr.add_robot(robot)
############################# Render Test #############################
custom_fpath = asset_file_path + "/config/ur5e_init_params.yaml"
with open(custom_fpath) as f:
    controller_config = yaml.safe_load(f)
init_qpos = controller_config["init_qpos"]

scene_mngr.set_robot_eef_pose(init_qpos)

scene_mngr.render_scene(ax)
scene_mngr.show()
