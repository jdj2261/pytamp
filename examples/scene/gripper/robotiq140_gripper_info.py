import numpy as np
import sys, os
import yaml

from pykin import assets
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pytamp.scene.scene_manager import SceneManager
from pykin.utils.mesh_utils import get_object_mesh
from pykin.utils import plot_utils as p_utils

fig, ax = p_utils.init_3d_figure()
asset_file_path = os.path.abspath(assets.__file__ + "/../")

# file_path = 'urdf/panda/panda.urdf'
# robot = SingleArm(
#     f_name=file_path,
#     offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
#     has_gripper=True)
# robot.setup_link_name("panda_link_0", "right_hand")

# custom_fpath = asset_file_path + '/config/panda_init_params.yaml'
# with open(custom_fpath) as f:
#     controller_config = yaml.safe_load(f)
# init_qpos = controller_config["init_qpos"]

file_path = "urdf/doosan/doosan_with_robotiq140.urdf"
robot = SingleArm(
    f_name=file_path,
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
    has_gripper=True,
    gripper_name="robotiq140",
)

robot.setup_link_name("base_0", "link6")
robot.init_qpos = np.array([0, 0, 0, 0, 0, 0])

custom_fpath = asset_file_path + "/config/doosan_init_params.yaml"
with open(custom_fpath) as f:
    controller_config = yaml.safe_load(f)
init_qpos = controller_config["init_qpos"]

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

scene_mngr = SceneManager("collision", is_pyplot=True)
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
scene_mngr.add_robot(robot, init_qpos)

############################# Gripper Pose Test #############################
scene_mngr.set_gripper_pose(scene_mngr.scene.robot.init_fk["right_gripper"].h_mat)
scene_mngr.render_objects_and_gripper(ax)

scene_mngr.set_gripper_tcp_pose(scene_mngr.scene.robot.init_fk["right_gripper"].h_mat)
scene_mngr.render_objects_and_gripper(ax)
# scene_mngr.render_scene(ax)
print(scene_mngr.get_gripper_pose())
print(scene_mngr.get_gripper_tcp_pose())
scene_mngr.gripper_collision_mngr.show_collision_info()

scene_mngr.show()
