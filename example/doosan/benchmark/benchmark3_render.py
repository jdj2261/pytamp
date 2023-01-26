from pytamp.benchmark import Benchmark3
from pykin.utils import plot_utils as p_utils

benchmark3 = Benchmark3(robot_name="doosan", geom="visual", is_pyplot=False)
fig, ax = p_utils.init_3d_figure(name="Benchmark 3")

benchmark3.scene_mngr.render_scene(ax)
benchmark3.scene_mngr.show()
