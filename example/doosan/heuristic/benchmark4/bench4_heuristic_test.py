import numpy as np
from copy import deepcopy

from pykin.utils import plot_utils as p_utils
from pykin.utils import transform_utils as t_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh, get_mesh_bounds
from pytamp.scene.scene_manager import SceneManager
from pytamp.action.pick import PickAction


file_path = "urdf/panda/panda.urdf"
robot = SingleArm(
    f_name=file_path,
    offset=Transform(rot=[0.0, 0.0, 0.0], pos=[0, 0, 0.913]),
    has_gripper=True,
)
robot.setup_link_name("panda_link_0", "right_hand")
robot.init_qpos = np.array(
    [0, np.pi / 16.0, 0.00, -np.pi / 2.0 - np.pi / 3.0, 0.00, np.pi - 0.2, -np.pi / 4]
)

table_mesh = get_object_mesh("ben_table.stl")
peg_mesh = get_object_mesh("hanoi_peg.stl", scale=[0.3, 0.3, 1.0])
disk_mesh = get_object_mesh("hanoi_disk.stl")

peg_mesh_bound = get_mesh_bounds(mesh=peg_mesh)
disk_mesh_bound = get_mesh_bounds(mesh=disk_mesh)
disk_heigh = disk_mesh_bound[1][2] - disk_mesh_bound[0][2]
table_height = table_mesh.bounds[1][2] - table_mesh.bounds[0][2]

table_pose = Transform(pos=np.array([1.0, -0.4, -0.03]))
peg1_pose = Transform(pos=np.array([0.6, -0.25, table_height + peg_mesh_bound[1][2]]))
peg2_pose = Transform(pos=np.array([0.6, 0, table_height + peg_mesh_bound[1][2]]))
peg3_pose = Transform(pos=np.array([0.6, 0.25, table_height + peg_mesh_bound[1][2]]))

disk_num = 6
disk_pose = [Transform() for _ in range(disk_num)]
disk_object = [0 for _ in range(disk_num)]

param = {"disk_num": disk_num}
benchmark_config = {4: param}
scene_mngr = SceneManager("collision", is_pyplot=True, benchmark=benchmark_config)

theta = np.linspace(-np.pi, np.pi, disk_num)
for i in range(disk_num):
    disk_pos = np.array(
        [0.6, 0.25, table_height + disk_mesh_bound[1][2] + disk_heigh * i]
    )
    disk_ori = Transform._to_quaternion([0, 0, i])
    disk_pose[i] = Transform(pos=disk_mesh.center_mass + disk_pos, rot=disk_ori)
    disk_name = "hanoi_disk_" + str(i)
    hanoi_mesh = get_object_mesh(f"hanoi_disk.stl")
    scene_mngr.add_object(
        name=disk_name,
        gtype="mesh",
        gparam=hanoi_mesh,
        h_mat=disk_pose[i].h_mat,
        color=[0.0, 1.0, 0.0],
    )
scene_mngr.add_robot(robot)
pick = PickAction(scene_mngr, n_contacts=0, n_directions=0)

fig, ax = p_utils.init_3d_figure(name="Get contact points")


pick.scene_mngr.render_objects(ax)
p_utils.plot_basis(ax)
for obj in [
    "hanoi_disk_0",
    "hanoi_disk_1",
    "hanoi_disk_2",
    "hanoi_disk_3",
    "hanoi_disk_4",
]:
    pose = list(pick.get_grasp_pose_from_heuristic(obj_name=obj))
    for i in range(len(pose)):
        pick.scene_mngr.render_axis(ax, pose[i][pick.move_data.MOVE_grasp])

pick.show()
