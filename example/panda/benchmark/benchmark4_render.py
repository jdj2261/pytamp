import numpy as np

from pykin.utils import plot_utils as p_utils
from pykin.kinematics.transform import Transform
from pykin.robots.single_arm import SingleArm
from pykin.utils.mesh_utils import get_object_mesh, get_mesh_bounds

from pytamp.scene.scene_manager import SceneManager

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

benchmark_config = {4: None}
scene_mngr = SceneManager("collision", is_pyplot=False, benchmark=benchmark_config)

theta = np.linspace(-np.pi, np.pi, disk_num)
for i in range(disk_num):
    disk_pos = np.array(
        [0.6, 0.25, table_height + disk_mesh_bound[1][2] + disk_heigh * i]
    )
    disk_ori = Transform._to_quaternion([0, 0, theta[i]])
    disk_pose[i] = Transform(pos=disk_pos, rot=disk_ori)
    disk_name = "hanoi_disk_" + str(i + 1)
    hanoi_mesh = get_object_mesh(f"hanoi_disk.stl")
    scene_mngr.add_object(
        name=disk_name,
        gtype="mesh",
        gparam=hanoi_mesh,
        h_mat=disk_pose[i].h_mat,
        color=[0.0, 1.0, 0.0],
    )

scene_mngr.add_object(
    name="peg_1",
    gtype="mesh",
    gparam=peg_mesh,
    h_mat=peg1_pose.h_mat,
    color=[1, 0.0, 0.0],
)
scene_mngr.add_object(
    name="peg_2",
    gtype="mesh",
    gparam=peg_mesh,
    h_mat=peg2_pose.h_mat,
    color=[1, 0.0, 0.0],
)
scene_mngr.add_object(
    name="peg_3",
    gtype="mesh",
    gparam=peg_mesh,
    h_mat=peg3_pose.h_mat,
    color=[1, 0.0, 0.0],
)
scene_mngr.add_object(
    name="table",
    gtype="mesh",
    gparam=table_mesh,
    h_mat=table_pose.h_mat,
    color=[0.823, 0.71, 0.55],
)
scene_mngr.add_robot(robot)

fig, ax = p_utils.init_3d_figure(name="Benchmark 4")
scene_mngr.render_scene(ax)
scene_mngr.show()
