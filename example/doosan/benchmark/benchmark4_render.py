from pytamp.benchmark import Benchmark4
from pykin.utils import plot_utils as p_utils

benchmark4 = Benchmark4(robot_name="doosan", geom="visual", is_pyplot=False)
fig, ax = p_utils.init_3d_figure(name="Benchmark 4")
benchmark4.scene_mngr.render_scene(ax)
benchmark4.scene_mngr.show()
