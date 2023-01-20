from pytamp.benchmark import Benchmark1
from pykin.utils import plot_utils as p_utils

benchmark1 = Benchmark1(
    robot_name="doosan", geom="collision", is_pyplot=True, box_num=6
)
fig, ax = p_utils.init_3d_figure(name="Benchmark 1")

benchmark1.scene_mngr.render_scene(ax)
benchmark1.scene_mngr.show()
