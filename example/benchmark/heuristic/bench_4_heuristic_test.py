import numpy as np
from copy import deepcopy

from pykin.utils import plot_utils as p_utils
from pykin.utils import transform_utils as t_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh, get_mesh_bounds
from pytamp.scene.scene_manager import SceneManager
from pytamp.action.pick import PickAction


file_path = 'urdf/panda/panda.urdf'
robot = SingleArm(
    f_name=file_path, 
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]), 
    has_gripper=True)
robot.setup_link_name("panda_link_0", "panda_right_hand")
robot.init_qpos = np.array([0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi/4])

disk_mesh = get_object_mesh('hanoi_disk.stl')

disk_mesh_bound = get_mesh_bounds(mesh=disk_mesh)
disk_heigh = disk_mesh_bound[1][2] - disk_mesh_bound[0][2]
disk_pose = Transform(pos=np.array([0.0, 0.0, 0.0]), rot=np.array([0, 0, 0]))


benchmark_config = {4 : None}
scene_mngr = SceneManager("visual", is_pyplot=True, benchmark=benchmark_config)
scene_mngr.add_object(name="hanoi_disk", gtype="mesh", gparam=disk_mesh, h_mat=disk_pose.h_mat, color=[0., 1., 0.])
scene_mngr.add_robot(robot)

pick = PickAction(scene_mngr, n_contacts=0, n_directions=0)

fig, ax = p_utils.init_3d_figure(name="Get contact points")
pose = list(pick.get_grasp_pose_from_heuristic(obj_name="hanoi_disk"))

pick.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
for i in range(len(pose)):
    pick.scene_mngr.render.render_axis(ax, pose[i][pick.move_data.MOVE_grasp])

pick.show()