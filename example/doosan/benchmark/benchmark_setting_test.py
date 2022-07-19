import numpy as np
from pykin.utils.mesh_utils import get_object_mesh
from pytamp.benchmark import Benchmark3
from pykin.kinematics.transform import Transform
from pykin.utils import plot_utils as p_utils

# About benchmark33
benchmark3 = Benchmark3(robot_name="doosan", geom="collision", is_pyplot=False)

arch_box = get_object_mesh('arch_box.stl', [0.001, 0.001, 0.001])
arch_box.apply_translation(-arch_box.center_mass)
can = get_object_mesh('can.stl')
can.apply_translation(-can.center_mass)
rect_box = get_object_mesh('rect_box.stl', [0.001, 0.001, 0.001])
rect_box.apply_translation(-rect_box.center_mass)
half_cylinder_box = get_object_mesh('half_cylinder_box.stl', [0.001, 0.001, 0.001])
half_cylinder_box.apply_translation(-half_cylinder_box.center_mass)
square_box = get_object_mesh('square_box.stl', [0.001, 0.001, 0.001])
square_box.apply_translation(-square_box.center_mass)
box_mesh = get_object_mesh('ben_cube.stl', 0.06)

table_height = benchmark3.table_mesh.bounds[1][2] - benchmark3.table_mesh.bounds[0][2]

arch_box_pose = Transform(pos=np.array([0.6, -0.05,  table_height + arch_box.bounds[1][2]]))
can_pose = Transform(np.array([0.6, -0.2, table_height + can.bounds[1][2]]))
rect_box_pose = Transform(np.array([0.5, -0.2, table_height + rect_box.bounds[1][2]]))
half_cylinder_box_pose = Transform(np.array([0.7, -0.2, table_height + half_cylinder_box.bounds[1][2]]))
square_box_pose = Transform(np.array([0.7, -0.1, table_height + square_box.bounds[1][2]]))
box_mesh_pose = Transform(pos=np.array([0.8, -0.2, table_height + box_mesh.bounds[1][2]]))


#  error 
#  ValueError: Duplicate name: object can already exists
#  I commented out the 6 lines below.

#benchmark3.scene_mngr.add_object("arch_box", gtype="mesh", gparam=arch_box, h_mat=arch_box_pose.h_mat)
#benchmark3.scene_mngr.add_object("can", gtype="mesh", gparam=can, h_mat=can_pose.h_mat)
#benchmark3.scene_mngr.add_object("rect_box", gtype="mesh", gparam=rect_box, h_mat=rect_box_pose.h_mat)
#benchmark3.scene_mngr.add_object("half_cylinder_box", gtype="mesh", gparam=half_cylinder_box, h_mat=half_cylinder_box_pose.h_mat)
#benchmark3.scene_mngr.add_object("square_box", gtype="mesh", gparam=square_box, h_mat=square_box_pose.h_mat)
#benchmark3.scene_mngr.add_object("ben_cube", gtype="mesh", gparam=box_mesh, h_mat=box_mesh_pose.h_mat)

fig, ax = p_utils.init_3d_figure(name="benchmark3 Test")
benchmark3.scene_mngr.render_scene(ax, alpha=0.8)
benchmark3.scene_mngr.show()
