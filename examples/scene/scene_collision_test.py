import numpy as np
import sys, os
import yaml

from pykin import assets
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pytamp.scene.scene_manager import SceneManager
from pykin.utils.mesh_utils import get_object_mesh
from pykin.utils.transform_utils import get_matrix_from_rpy
from pykin.utils.kin_utils import ShellColors as sc
from pykin.utils import plot_utils as p_utils

asset_file_path = os.path.abspath(assets.__file__ + "/../")

fig, ax = p_utils.init_3d_figure()
file_path = "urdf/panda/panda.urdf"
robot = SingleArm(
    f_name=file_path,
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
    has_gripper=True,
)
robot.setup_link_name("panda_link_0", "right_hand")

custom_fpath = asset_file_path + "/config/panda_init_params.yaml"
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

scene_mngr = SceneManager("collision", is_pyplot=False)
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


############################# Show collision info #############################
# scene_mngr.robot_collision_mngr.show_collision_info()
# scene_mngr.obj_collision_mngr.show_collision_info("Object")
# scene_mngr.gripper_collision_mngr.show_collision_info("Gripper")

############################# Self Collision #############################
# target_thetas = np.random.randn(scene_mngr.scene.robot.arm_dof)
# scene_mngr.set_robot_eef_pose(target_thetas)
# scene_mngr.robot_collision_mngr.show_collision_info("Robot")

# # print(scene_mngr.collide_self_robot(return_names=True))
# result, names = scene_mngr.collide_self_robot(return_names=True)
# if result:
#     for obj1, obj2 in list(names):
#         print(f"{sc.FAIL}Collide!! {sc.ENDC}{obj1} and {obj2}")

# scene_mngr.render_scene(ax, robot_color='b')
# scene_mngr.show()

eef_pose = green_box_pose.h_mat
r_mat = get_matrix_from_rpy(np.array([0, np.pi / 2, 0]))
eef_pose[:3, :3] = r_mat
eef_pose[:3, 3] = eef_pose[:3, 3] - [0.05, 0, 0]
########################### Collide Robot and Object #############################
# target_thetas = scene_mngr.compute_ik(eef_pose)
# scene_mngr.set_robot_eef_pose(target_thetas)
# scene_mngr.robot_collision_mngr.show_collision_info("Robot")

# # print(scene_mngr.collide_self_robot(return_names=True))
# result, names = scene_mngr.collide_objs_and_robot(return_names=True)
# if result:
#     for obj1, obj2 in list(names):
#         print(f"{sc.FAIL}Collide!! {sc.ENDC}{obj1} and {obj2}")

# scene_mngr.render_scene(ax, robot_color='b')
# scene_mngr.show()

############################# Collide Gripper and Object #############################
scene_mngr.set_gripper_pose(eef_pose)
scene_mngr.gripper_collision_mngr.show_collision_info("Gripper")

result, names = scene_mngr.collide_objs_and_gripper(return_names=True)
if result:
    for obj1, obj2 in list(names):
        print(f"{sc.FAIL}Collide!! {sc.ENDC}{obj1} and {obj2}")

scene_mngr.render_objects_and_gripper(ax, visible_tcp=False)
scene_mngr.show()
