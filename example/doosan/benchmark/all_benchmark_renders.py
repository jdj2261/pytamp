from pytamp.benchmark import Benchmark, Benchmark1, Benchmark2, Benchmark3, Benchmark4
from pykin.utils import plot_utils as p_utils
from pykin.robots.single_arm import SingleArm
from pykin.kinematics.transform import Transform

benchmark = Benchmark(robot_name="doosan", geom="visual", is_pyplot=True)
fig, ax = p_utils.init_3d_figure(name="Benchmark")
robot = SingleArm(benchmark.urdf_file, offset=Transform(pos=[0, 0, 0.913]))
benchmark.scene_mngr.add_robot(robot)
benchmark.scene_mngr.render_scene(ax)
benchmark.scene_mngr.show()

benchmark1 = Benchmark1(robot_name="doosan", geom="visual", is_pyplot=False, box_num=6)
fig, ax = p_utils.init_3d_figure(name="Benchmark 1")
benchmark1.scene_mngr.render_scene(ax)
result, names = benchmark1.scene_mngr.collide_objs_and_robot(return_names=True)
print(names)
benchmark1.scene_mngr.render_scene(ax)
benchmark1.scene_mngr.show()

benchmark2 = Benchmark2(robot_name="doosan", geom="visual", is_pyplot=False)
fig, ax = p_utils.init_3d_figure(name="Benchmark 2")
result, names = benchmark2.scene_mngr.collide_objs_and_robot(return_names=True)
print(names)
benchmark2.scene_mngr.render_scene(ax)
benchmark2.scene_mngr.show()

benchmark3 = Benchmark3(robot_name="doosan", geom="visual", is_pyplot=False)
fig, ax = p_utils.init_3d_figure(name="Benchmark 3")
result, names = benchmark3.scene_mngr.collide_objs_and_robot(return_names=True)
print(names)
benchmark3.scene_mngr.render_scene(ax)
benchmark3.scene_mngr.show()

benchmark4 = Benchmark4(robot_name="doosan", geom="visual", is_pyplot=False)
fig, ax = p_utils.init_3d_figure(name="Benchmark 4")
result, names = benchmark4.scene_mngr.collide_objs_and_robot(return_names=True)
print(names)
benchmark4.scene_mngr.render_scene(ax)
benchmark4.scene_mngr.show()
