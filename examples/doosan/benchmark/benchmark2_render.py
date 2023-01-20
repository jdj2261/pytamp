from pytamp.benchmark import Benchmark2
from pykin.utils import plot_utils as p_utils

benchmark2 = Benchmark2(
    robot_name="doosan",
    geom="visual",
    is_pyplot=False,
    bottle_num=6,
    is_only_render=True,
)
fig, ax = p_utils.init_3d_figure(name="Benchmark 2")
benchmark2.scene_mngr.render_scene(ax)
benchmark2.scene_mngr.show()
