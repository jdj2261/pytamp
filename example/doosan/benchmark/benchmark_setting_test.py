import numpy as np
from pykin.utils.mesh_utils import get_object_mesh
from pytamp.benchmark import Benchmark
from pykin.kinematics.transform import Transform
from pykin.utils import plot_utils as p_utils

benchmark = Benchmark(robot_name="doosan", geom="collision", is_pyplot=True)

table = get_object_mesh('ben_table.stl')
arch_box = get_object_mesh('arch_box.stl', [0.001, 0.001, 0.001])
arch_box.apply_translation(-arch_box.center_mass)
triangle_box = get_object_mesh('triangle_box.stl', [0.001, 0.001, 0.001])
triangle_box.apply_translation(-triangle_box.center_mass)
rect_box = get_object_mesh('rect_box.stl', [0.001, 0.001, 0.001])
rect_box.apply_translation(-rect_box.center_mass)
half_cylinder_box = get_object_mesh('half_cylinder_box.stl', [0.001, 0.001, 0.001])
half_cylinder_box.apply_translation(-half_cylinder_box.center_mass)
square_box = get_object_mesh('square_box.stl', [0.001, 0.001, 0.001])
square_box.apply_translation(-square_box.center_mass)

table_pose = Transform(pos=np.array([1.0, -0.4, -0.03]))
arch_box_pose = Transform(pos=np.array([0.6, -0.2, table.bounds[1][2] + arch_box.bounds[0][2]]))
triangle_box_pose = Transform(np.array([0.6, -0.3, table.bounds[1][2] + triangle_box.bounds[0][2]]))
rect_box_pose = Transform(np.array([0.6, 0.1, table.bounds[1][2] ]))
half_cylinder_box_pose = Transform(np.array([0.6, 0.3, table.bounds[1][2] + half_cylinder_box.bounds[0][2]]))
square_box_pose = Transform(np.array([0.6, -0.1, table.bounds[1][2] + square_box.bounds[0][2]]))

benchmark.scene_mngr.add_object("table", gtype="mesh", gparam=table, h_mat=table_pose.h_mat, color=[0.39, 0.263, 0.129])
benchmark.scene_mngr.add_object("arch_box", gtype="mesh", gparam=arch_box, h_mat=arch_box_pose.h_mat)
benchmark.scene_mngr.add_object("triangle_box", gtype="mesh", gparam=triangle_box, h_mat=triangle_box_pose.h_mat)
benchmark.scene_mngr.add_object("rect_box", gtype="mesh", gparam=rect_box, h_mat=rect_box_pose.h_mat)
benchmark.scene_mngr.add_object("half_cylinder_box", gtype="mesh", gparam=half_cylinder_box, h_mat=half_cylinder_box_pose.h_mat)
benchmark.scene_mngr.add_object("square_box", gtype="mesh", gparam=square_box, h_mat=square_box_pose.h_mat)
benchmark.scene_mngr.add_robot(benchmark.robot)

fig, ax = p_utils.init_3d_figure(name="Benchmark Test")
benchmark.scene_mngr.render_scene(ax, alpha=0.8)
benchmark.scene_mngr.show()
