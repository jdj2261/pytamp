import numpy as np

from pykin.utils import plot_utils as p_utils
from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh

from pytamp.scene.scene_manager import SceneManager
from pytamp.action.place import PlaceAction

tray_red_pose = Transform(pos=np.array([0.6, -0.5 - 0.3, 0.8]))
tray_blue_pose = Transform(pos=np.array([0.6, 0.5, 0.8]))

tray_red_mesh = get_object_mesh("ben_tray_red.stl")
tray_blue_mesh = get_object_mesh("ben_tray_blue.stl")

param = {"stack_num": 6}
benchmark_config = {1: param}

scene_mngr = SceneManager("visual", is_pyplot=False, benchmark=benchmark_config)
scene_mngr.add_object(
    name="tray_red",
    gtype="mesh",
    gparam=tray_red_mesh,
    h_mat=tray_red_pose.h_mat,
    color=[1.0, 0, 0],
)
scene_mngr.add_object(
    name="tray_blue",
    gtype="mesh",
    gparam=tray_blue_mesh,
    h_mat=tray_blue_pose.h_mat,
    color=[0, 0, 1.0],
)

place_action = PlaceAction(scene_mngr, n_samples_held_obj=0, n_samples_support_obj=100)
surface_points_for_support_obj = list(
    place_action.get_surface_points_for_support_obj("tray_red", alpha=1)
)
fig, ax = p_utils.init_3d_figure(figsize=(10, 6), dpi=120, name="Sampling Object")
p_utils.plot_basis(ax)
place_action.scene_mngr.render_objects(ax, alpha=0.1)
for point, normal, (min_x, max_x, min_y, max_y) in surface_points_for_support_obj:
    if not (min_x <= point[0] <= max_x):
        continue
    if not (min_y <= point[1] <= max_y):
        continue
    place_action.scene_mngr.render.render_point(
        ax, point, color=[0.0, 1.0, 0.0], radius=0.005
    )

place_action.show()
