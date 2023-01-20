import numpy as np

from pykin.utils import plot_utils as p_utils

from pykin.kinematics.transform import Transform
from pykin.utils.mesh_utils import get_object_mesh, get_mesh_bounds
from pytamp.scene.scene_manager import SceneManager
from pytamp.action.place import PlaceAction

table_mesh = get_object_mesh("ben_table.stl")
table_height = table_mesh.bounds[1][2] - table_mesh.bounds[0][2]

disk_mesh = get_object_mesh("hanoi_disk.stl")
disk_mesh_bound = get_mesh_bounds(mesh=disk_mesh)
disk_heigh = disk_mesh_bound[1][2] - disk_mesh_bound[0][2]

benchmark_config = {4: {"disk_num": 6}}
scene_mngr = SceneManager("visual", is_pyplot=False, benchmark=benchmark_config)
disk_pose = [Transform() for _ in range(3)]

for i in range(3):
    disk_pos = np.array(
        [0.69, 0.3, table_height + disk_mesh_bound[1][2] + disk_heigh * i]
    )
    disk_pose[i] = Transform(pos=disk_mesh.center_mass + disk_pos)
    disk_name = "hanoi_disk_" + str(i)
    print(disk_name)
    hanoi_mesh = get_object_mesh(
        f"hanoi_disk.stl", scale=[2.0 - 0.2 * i, 2.0 - 0.2 * i, 2.0]
    )
    scene_mngr.add_object(
        name=disk_name,
        gtype="mesh",
        gparam=hanoi_mesh,
        h_mat=disk_pose[i].h_mat,
        color=[0.0, 1.0, 0.0],
    )

place_action = PlaceAction(scene_mngr, n_samples_held_obj=0, n_samples_support_obj=10)
surface_points_for_support_obj = list(
    place_action.get_surface_points_for_support_obj("hanoi_disk_2")
)
fig, ax = p_utils.init_3d_figure(figsize=(10, 6), dpi=120, name="Sampling Object")
# p_utils.plot_basis(ax)
place_action.scene_mngr.render_objects(ax, alpha=0.5)

for point, normal, (min_x, max_x, min_y, max_y) in surface_points_for_support_obj:
    place_action.scene_mngr.render.render_point(ax, point, radius=0.01)

surface_points_for_support_obj = list(
    place_action.get_surface_points_for_support_obj("hanoi_disk_2")
)
for point, normal, (min_x, max_x, min_y, max_y) in surface_points_for_support_obj:
    place_action.scene_mngr.render.render_point(ax, point, radius=0.01)

place_action.show()
