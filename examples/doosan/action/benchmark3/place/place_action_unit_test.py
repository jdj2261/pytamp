from pykin.utils import plot_utils as p_utils
from pytamp.action.pick import PickAction
from pytamp.action.place import PlaceAction
from pytamp.benchmark import Benchmark3

benchmark3 = Benchmark3(robot_name="doosan", geom="collision", is_pyplot=True)
pick = PickAction(
    benchmark3.scene_mngr, n_contacts=3, n_directions=5, retreat_distance=0.1
)
place = PlaceAction(
    benchmark3.scene_mngr,
    n_samples_held_obj=0,
    n_samples_support_obj=3,
    n_directions=10,
)


###### Surface sampling held and support obj#######
fig, ax = p_utils.init_3d_figure(figsize=(10, 6), dpi=120, name="Sampling Object")
surface_points_for_support_obj = list(
    place.get_surface_points_for_support_obj("clearbox_1_8")
)
for point, normal, _ in surface_points_for_support_obj:
    place.scene_mngr.render.render_point(ax, point)
surface_points_for_held_obj = list(
    place.get_surface_points_for_held_obj("rect_bottom_box")
)
for point, normal in surface_points_for_held_obj:
    place.scene_mngr.render.render_point(ax, point)
p_utils.plot_basis(ax)
place.scene_mngr.render_objects(ax, alpha=0.5)

##### All Release Pose #######
fig, ax = p_utils.init_3d_figure(name="Get Release Pose")
eef_poses = list(pick.get_all_grasp_poses("rect_bottom_box"))

# Add heuristic
eef_poses.extend(list(pick.get_grasp_pose_from_heuristic("rect_bottom_box")))

all_release_poses = []
for eef_pose in eef_poses:
    release_poses = list(
        place.get_all_release_poses_and_obj_pose(
            "clearbox_1_8", "rect_bottom_box", eef_pose[pick.move_data.MOVE_grasp]
        )
    )
    # pick.scene_mngr.render.render_gripper(ax, benchmark3.robot, pose=eef_pose[place.move_data.MOVE_grasp])
    for release_pose, obj_pose in release_poses:
        all_release_poses.append((release_pose, obj_pose))
        place.scene_mngr.render.render_axis(
            ax, release_pose[place.move_data.MOVE_release]
        )
        place.scene_mngr.render.render_object(
            ax, place.scene_mngr.scene.objs["rect_bottom_box"], obj_pose
        )
p_utils.plot_basis(ax)
place.scene_mngr.render_objects(ax)

# # ###### Level wise - 1 #######
fig, ax = p_utils.init_3d_figure(name="Level wise 1")
release_poses_for_only_gripper = list(
    place.get_release_poses_not_collision(all_release_poses, False)
)
for release_pose_for_only_gripper, obj_pose in release_poses_for_only_gripper:
    place.scene_mngr.render.render_axis(
        ax, release_pose_for_only_gripper[place.move_data.MOVE_release], scale=0.05
    )
    place.scene_mngr.render.render_object(
        ax, place.scene_mngr.scene.objs["rect_bottom_box"], obj_pose
    )
p_utils.plot_basis(ax)
place.scene_mngr.render_objects(ax, alpha=0.1)
place.show()
