from pytamp.benchmark import Benchmark4
from pykin.utils import plot_utils as p_utils

benchmark4 = Benchmark4(robot_name="doosan", geom="visual", is_pyplot=False, disk_num=3)
fig, ax = p_utils.init_3d_figure(name="Benchmark 4")

print(len(benchmark4.scene_mngr.scene.logical_states["peg_1"]["hung"]))
benchmark4.scene_mngr.render_scene(ax)
benchmark4.scene_mngr.show()
