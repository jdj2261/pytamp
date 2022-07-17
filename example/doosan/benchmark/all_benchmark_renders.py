from pytamp.benchmark import Benchmark1, Benchmark2, Benchmark3, Benchmark4
from pykin.utils import plot_utils as p_utils

benchmark1 = Benchmark1(geom="collision", is_pyplot=True, box_num=3)
fig, ax = p_utils.init_3d_figure(name="Benchmark 1")
result, names = benchmark1.scene_mngr.collide_objs_and_robot(return_names=True)
print(names)
benchmark1.scene_mngr.render_scene(ax)
benchmark1.scene_mngr.show()

benchmark2 = Benchmark2(geom="collision", is_pyplot=True)
fig, ax = p_utils.init_3d_figure(name="Benchmark 2")
result, names = benchmark2.scene_mngr.collide_objs_and_robot(return_names=True)
print(names)
benchmark2.scene_mngr.render_scene(ax)
benchmark2.scene_mngr.show()

benchmark3 = Benchmark3(geom="collision", is_pyplot=True)
fig, ax = p_utils.init_3d_figure(name="Benchmark 3")
result, names = benchmark3.scene_mngr.collide_objs_and_robot(return_names=True)
print(names)
benchmark3.scene_mngr.render_scene(ax)
benchmark3.scene_mngr.show()

benchmark4 = Benchmark4(geom="collision", is_pyplot=True)
fig, ax = p_utils.init_3d_figure(name="Benchmark 4")
result, names = benchmark4.scene_mngr.collide_objs_and_robot(return_names=True)
print(names)
benchmark4.scene_mngr.render_scene(ax)
benchmark4.scene_mngr.show()
